#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Return coordinator for the trajectory recorder and QGC link monitor.

Outputs are ROS goals and LAND intent; the shared bridge transports them.
"""
import copy
import math
import threading
import uuid

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, String, UInt32

from vio_homing.homing_logic import HomingLogic


def position(pose):
    p = pose.position
    return (p.x, p.y, p.z)


class HomingManager(object):

    def __init__(self):
        self.lock = threading.RLock()
        self.logic = HomingLogic(
            arrival_radius=float(rospy.get_param('~arrival_radius', 0.4)),
            wait_link_seconds=float(
                rospy.get_param('~wait_link_seconds', 60.0)
            ),
            odom_timeout=float(rospy.get_param('~odom_timeout', 1.0)),
            arrival_dwell=float(rospy.get_param('~arrival_dwell', 0.5)),
            arrival_speed=float(rospy.get_param('~arrival_speed', 0.5)),
            progress_timeout=float(rospy.get_param('~progress_timeout', 15.0)),
            waypoint_timeout=float(rospy.get_param('~waypoint_timeout', 60.0))
        )
        self.navigation_epoch = None
        self.candidate_path = None
        self.return_ready = False
        self.last_good_msg = None
        self.active_poses = []
        self.active_endpoint = None
        self.previous_state = None
        self.reason_pub = rospy.Publisher('/vio_homing/homing_reason', String, queue_size=1, latch=True)
        self.session_pub = rospy.Publisher(
            '/vio_homing/homing_session', String, queue_size=1, latch=True
        )
        self.session_pub.publish(String(data=str(uuid.uuid4())))
        self.state_pub = rospy.Publisher(
            '/vio_homing/homing_state', String, queue_size=10, latch=True
        )
        self.goal_pub = rospy.Publisher(
            '/vio_homing/return_goal', PoseStamped, queue_size=10
        )
        self.active_pub = rospy.Publisher(
            '/vio_homing/return_active', Bool, queue_size=10, latch=True
        )
        self.index_pub = rospy.Publisher(
            '/vio_homing/return_goal_index', UInt32, queue_size=10
        )
        self.land_pub = rospy.Publisher(
            '/vio_homing/land_command', Bool, queue_size=10, latch=True
        )
        self.subscribers = [
            rospy.Subscriber(
                '/vio_homing/link_state', String, self.on_link, queue_size=20
            ),
            rospy.Subscriber(
                '/vio_homing/return_path', Path, self.on_path, queue_size=10
            ),
            rospy.Subscriber(
                '/vio_homing/return_ready', Bool, self.on_ready, queue_size=10
            ),
            rospy.Subscriber(
                '/vio_homing/last_good_link_pose',
                PoseStamped,
                self.on_last_good,
                queue_size=10
            ),
            rospy.Subscriber(
                rospy.get_param('~odom_topic', '/vio/odom_frd'),
                Odometry,
                self.on_odom,
                queue_size=20
            )
        ]
        self.epoch_sub = rospy.Subscriber('/vio_homing/navigation_epoch', String, self.on_epoch, queue_size=1)
        self.publish_state()
        self.timer = rospy.Timer(rospy.Duration(0.05), self.tick)
        rospy.loginfo(
            'Homing manager: ROS goals, wait %.1f s, radius %.2f m',
            self.logic.wait_link_seconds, self.logic.arrival_radius
        )

    def on_epoch(self, msg):
        with self.lock:
            if self.navigation_epoch == msg.data:
                return
            self.navigation_epoch = msg.data
            old = self.logic
            self.logic = HomingLogic(old.arrival_radius, old.wait_link_seconds, old.odom_timeout,
                                     old.arrival_dwell, old.arrival_speed, old.progress_timeout,
                                     old.progress_distance, old.waypoint_timeout)
            self.candidate_path = self.last_good_msg = None
            self.return_ready = False
            self.active_poses = []
            self.active_endpoint = None
            self.session_pub.publish(String(data=str(uuid.uuid4())))
            self.publish_state()

    def publish_state(self):
        state = self.logic.state
        if state != self.previous_state:
            rospy.loginfo('HOMING: %s -> %s', self.previous_state, state)
            self.previous_state = state
        self.state_pub.publish(String(data=state))
        self.active_pub.publish(Bool(data=state in ('RETURN', 'WAIT_LINK')))
        self.land_pub.publish(Bool(data=state in ('LAND', 'ABORT')))
        self.reason_pub.publish(String(data=self.logic.reason))

    def on_link(self, msg):
        with self.lock:
            old_state = self.logic.state
            self.logic.set_link(msg.data, rospy.get_time())
            if msg.data == 'LINK_OK':
                self.candidate_path = None
                self.return_ready = False
                self.active_poses = []
                self.active_endpoint = None
            if self.logic.state != old_state:
                # Expose LINK_LOST even when a path was received before this
                # callback.
                self.publish_state()

    def on_path(self, msg):
        with self.lock:
            if self.logic.state in ('RETURN', 'WAIT_LINK', 'LAND', 'ABORT'):
                return  # The active episode uses an immutable route snapshot.
            if not msg.poses:
                self.candidate_path = None
            else:
                self.candidate_path = copy.deepcopy(msg)

    def on_ready(self, msg):
        with self.lock:
            self.return_ready = msg.data

    def on_last_good(self, msg):
        with self.lock:
            self.last_good_msg = copy.deepcopy(msg)

    def on_odom(self, msg):
        with self.lock:
            self.logic.set_odom(
                position(msg.pose.pose), msg.header.frame_id,
                msg.header.stamp.to_sec(), rospy.get_time(),
                math.sqrt(sum(v*v for v in (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z)))
            )

    def start_return(self):
        path = self.candidate_path
        endpoint = self.last_good_msg
        if not self.return_ready or path is None or endpoint is None:
            return False
        # This gate permits path/ready to arrive before LINK_LOST, but rejects
        # a latched route from before the latest healthy-link observation.
        if path.header.stamp.to_sec() <= self.logic.last_good_link_time:
            return False
        frame = path.header.frame_id
        if (
            not frame or endpoint.header.frame_id != frame
            or any(p.header.frame_id != frame for p in path.poses)
        ):
            return False
        points = [position(p.pose) for p in path.poses]
        if not self.logic.start_return(points, position(endpoint.pose), frame):
            return False
        self.active_poses = copy.deepcopy(path.poses)
        self.active_endpoint = copy.deepcopy(endpoint)
        rospy.loginfo('RETURN snapshot: %d points in %s', len(points), frame)
        return True

    def tick(self, event):
        with self.lock:
            started = self.logic.state == 'LINK_LOST' and self.start_return()
            if not started:
                self.logic.step(rospy.get_time())
            self.publish_state()
            if self.logic.state == 'RETURN':
                goal = copy.deepcopy(self.active_poses[self.logic.index])
            elif self.logic.state == 'WAIT_LINK':
                goal = copy.deepcopy(self.active_endpoint)
            else:
                return
            goal.header.stamp = rospy.Time.now()
            self.goal_pub.publish(goal)
            self.index_pub.publish(UInt32(data=self.logic.index))


def main():
    rospy.init_node('homing_manager')
    HomingManager()
    rospy.spin()


if __name__ == '__main__':
    main()
