#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import threading

import rospy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import UInt32


class TrajectoryRecorder(object):

    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/vio/odom_frd")
        self.link_state_topic = rospy.get_param("~link_state_topic", "/vio_homing/link_state")
        self.min_point_distance = float(rospy.get_param("~min_point_distance", 0.25))
        self.max_point_interval = float(rospy.get_param("~max_point_interval", 1.0))
        self.max_points = int(rospy.get_param("~max_points", 20000))
        self.path_publish_rate = float(rospy.get_param("~path_publish_rate", 1.0))

        self.lock = threading.Lock()
        self.points = []
        self.frame_id = "local_frd"
        self.link_state = "WAITING"
        self.last_good_index = None
        self.last_good_pose = None
        self.return_path = None
        self.return_ready = False
        self.last_record_time = None
        self.odom_count = 0
        self.recorded_count = 0
        self.loss_count = 0

        self.trajectory_pub = rospy.Publisher("/vio_homing/trajectory", Path, queue_size=1)
        self.return_path_pub = rospy.Publisher(
            "/vio_homing/return_path", Path, queue_size=1, latch=True
        )
        self.last_good_pose_pub = rospy.Publisher(
            "/vio_homing/last_good_link_pose", PoseStamped, queue_size=1, latch=True
        )
        self.return_ready_pub = rospy.Publisher(
            "/vio_homing/return_ready", Bool, queue_size=1, latch=True
        )
        self.point_count_pub = rospy.Publisher(
            "/vio_homing/trajectory_point_count", UInt32, queue_size=10
        )
        self.status_pub = rospy.Publisher("/vio_homing/trajectory_status", String, queue_size=10)
        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self.odom_callback, queue_size=100
        )
        self.link_sub = rospy.Subscriber(
            self.link_state_topic, String, self.link_state_callback, queue_size=20
        )

        if self.path_publish_rate > 0:
            self.publish_timer = rospy.Timer(
                rospy.Duration(1.0 / self.path_publish_rate), self.publish_periodic
            )
        else:
            self.publish_timer = None

        rospy.loginfo("Trajectory recorder started")
        rospy.loginfo("Odometry: %s", self.odom_topic)
        rospy.loginfo("Link state: %s", self.link_state_topic)
        rospy.loginfo(
            "Point spacing: %.2f m, max interval: %.2f s",
            self.min_point_distance,
            self.max_point_interval,
        )

    def pose_from_odometry(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = msg.pose.pose.position.x
        pose.pose.position.y = msg.pose.pose.position.y
        pose.pose.position.z = msg.pose.pose.position.z
        pose.pose.orientation.x = msg.pose.pose.orientation.x
        pose.pose.orientation.y = msg.pose.pose.orientation.y
        pose.pose.orientation.z = msg.pose.pose.orientation.z
        pose.pose.orientation.w = msg.pose.pose.orientation.w

        return pose

    def distance(self, a, b):
        dx = a.pose.position.x - b.pose.position.x
        dy = a.pose.position.y - b.pose.position.y
        dz = a.pose.position.z - b.pose.position.z

        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def should_record(self, pose):
        if len(self.points) == 0:
            return True

        last_pose = self.points[-1]
        distance = self.distance(pose, last_pose)

        if distance >= self.min_point_distance:
            return True

        if self.last_record_time is None:
            return True

        now = pose.header.stamp

        if now.secs == 0 and now.nsecs == 0:
            now = rospy.Time.now()

        elapsed = (now - self.last_record_time).to_sec()

        if elapsed >= self.max_point_interval:
            return True

        return False

    def trim_buffer(self):
        while len(self.points) > self.max_points:
            self.points.pop(0)

            if self.last_good_index is not None:
                self.last_good_index -= 1

                if self.last_good_index < 0:
                    self.last_good_index = 0

    def record_pose(self, pose):
        self.points.append(pose)
        self.recorded_count += 1

        if pose.header.stamp.secs == 0 and pose.header.stamp.nsecs == 0:
            self.last_record_time = rospy.Time.now()
        else:
            self.last_record_time = pose.header.stamp

        self.trim_buffer()

        if self.link_state == "LINK_OK":
            self.last_good_index = len(self.points) - 1
            self.last_good_pose = pose
            self.last_good_pose_pub.publish(pose)

    def odom_callback(self, msg):
        pose = self.pose_from_odometry(msg)

        with self.lock:
            self.odom_count += 1

            if msg.header.frame_id:
                self.frame_id = msg.header.frame_id

            if self.should_record(pose):
                self.record_pose(pose)

    def reset_return_state(self):
        self.return_path = None
        self.return_ready = False
        self.return_ready_pub.publish(Bool(data=False))
        empty_path = Path()
        empty_path.header.stamp = rospy.Time.now()
        empty_path.header.frame_id = self.frame_id
        self.return_path_pub.publish(empty_path)

        rospy.loginfo("Return path cleared after link recovery")

    def build_return_path(self):
        if self.last_good_index is None:
            rospy.logerr("Cannot build return path: " "no LINK_OK point has been recorded")

            self.return_ready = False
            self.return_ready_pub.publish(Bool(data=False))

            return

        if len(self.points) == 0:
            rospy.logerr("Cannot build return path: " "trajectory is empty")

            self.return_ready = False
            self.return_ready_pub.publish(Bool(data=False))

            return

        start_index = self.last_good_index

        if start_index >= len(self.points):
            start_index = len(self.points) - 1

        forward_segment = self.points[start_index:]
        reverse_segment = list(reversed(forward_segment))
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = self.frame_id
        path.poses = reverse_segment
        self.return_path = path
        self.return_ready = True
        self.loss_count += 1
        self.return_path_pub.publish(path)
        self.return_ready_pub.publish(Bool(data=True))

        rospy.logerr(
            "RETURN PATH READY: " "%d points, target index=%d, " "total trajectory=%d",
            len(reverse_segment),
            start_index,
            len(self.points),
        )

        if self.last_good_pose is not None:
            position = self.last_good_pose.pose.position

            rospy.logerr(
                "LAST GOOD LINK POINT: " "x=%.3f y=%.3f z=%.3f", position.x, position.y, position.z
            )

    def link_state_callback(self, msg):
        new_state = msg.data

        with self.lock:
            old_state = self.link_state

            if new_state == old_state:
                return

            self.link_state = new_state
            rospy.loginfo("Recorder link state: %s -> %s", old_state, new_state)

            if new_state == "LINK_OK":
                if self.return_ready:
                    self.reset_return_state()

                if len(self.points) > 0:
                    self.last_good_index = len(self.points) - 1
                    self.last_good_pose = self.points[-1]
                    self.last_good_pose_pub.publish(self.last_good_pose)

            if new_state == "LINK_LOST" and old_state != "LINK_LOST":
                self.build_return_path()

    def make_trajectory_path(self):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = self.frame_id
        path.poses = list(self.points)

        return path

    def publish_periodic(self, event):
        with self.lock:
            path = self.make_trajectory_path()
            point_count = len(self.points)
            link_state = self.link_state
            return_ready = self.return_ready

        self.trajectory_pub.publish(path)
        self.point_count_pub.publish(UInt32(data=point_count))
        status = "state=%s points=%d return_ready=%s" % (
            link_state,
            point_count,
            str(return_ready),
        )
        self.status_pub.publish(String(data=status))


def main():
    rospy.init_node("trajectory_recorder")
    TrajectoryRecorder()
    rospy.spin()


if __name__ == "__main__":
    main()
