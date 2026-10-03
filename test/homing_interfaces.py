#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Transport-level tests for callback ordering and invalid/stale inputs."""
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, String


class Interfaces(unittest.TestCase):
    def setUp(self):
        self.state = None
        self.active = None
        self.goal_x = None
        self.link = rospy.Publisher('/vio_homing/link_state', String, queue_size=10)
        self.path = rospy.Publisher('/vio_homing/return_path', Path, queue_size=10, latch=True)
        self.ready = rospy.Publisher('/vio_homing/return_ready', Bool, queue_size=10, latch=True)
        self.endpoint = rospy.Publisher('/vio_homing/last_good_link_pose', PoseStamped, queue_size=10, latch=True)
        self.odom = rospy.Publisher('/vio/odom_frd', Odometry, queue_size=10)
        self.subs = [
            rospy.Subscriber('/vio_homing/homing_state', String, self.on_state),
            rospy.Subscriber('/vio_homing/return_active', Bool, self.on_active),
            rospy.Subscriber('/vio_homing/return_goal', PoseStamped, self.on_goal)]
        self.wait(lambda: self.link.get_num_connections() and self.path.get_num_connections()
                  and self.ready.get_num_connections() and self.endpoint.get_num_connections()
                  and self.odom.get_num_connections() and self.state is not None, 10)

    def on_state(self, msg):
        self.state = msg.data

    def on_active(self, msg):
        self.active = msg.data

    def on_goal(self, msg):
        self.goal_x = msg.pose.position.x

    def wait(self, predicate, timeout=3):
        end = time.time() + timeout
        while time.time() < end:
            if predicate():
                return
            time.sleep(0.02)
        self.fail('Timeout: state=%s goal=%s' % (self.state, self.goal_x))

    def assert_stays(self, wanted, seconds=0.25):
        end = time.time() + seconds
        while time.time() < end:
            self.assertEqual(self.state, wanted)
            time.sleep(0.02)

    def reset(self):
        self.link.publish(String(data='LINK_OK'))
        self.wait(lambda: self.state == 'NORMAL' and self.active is False)
        time.sleep(0.1)
        self.goal_x = None

    def route(self, start=2.0, end=0.0, frame='local_frd', stamp=None):
        path = Path()
        path.header.frame_id = frame
        path.header.stamp = stamp or rospy.Time.now()
        for x in (start, end):
            pose = PoseStamped()
            pose.header.frame_id = frame
            pose.header.stamp = path.header.stamp
            pose.pose.position.x = x
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def send_odom(self, x, frame='local_frd', stamp=None):
        msg = Odometry()
        msg.header.frame_id = frame
        msg.header.stamp = stamp or rospy.Time.now()
        msg.pose.pose.position.x = x
        msg.pose.pose.orientation.w = 1.0
        self.odom.publish(msg)

    def test_transport_and_validation(self):
        self.link.publish(String(data='LINK_LOST'))
        self.assert_stays('NORMAL')  # No healthy link has been observed yet.

        # Cross-topic order: recorder output can arrive before loss callback.
        for order in ('path_before_loss', 'ready_before_path', 'path_before_ready'):
            self.reset()
            route = self.route()
            self.endpoint.publish(route.poses[-1])
            if order == 'path_before_loss':
                self.path.publish(route)
                self.ready.publish(Bool(data=True))
                time.sleep(0.1)
                self.link.publish(String(data='LINK_LOST'))
            else:
                self.link.publish(String(data='LINK_LOST'))
                self.wait(lambda: self.state == 'LINK_LOST')
                if order == 'ready_before_path':
                    self.ready.publish(Bool(data=True))
                    time.sleep(0.1)
                    self.path.publish(route)
                else:
                    self.path.publish(route)
                    time.sleep(0.1)
                    self.assert_stays('LINK_LOST')
                    self.ready.publish(Bool(data=True))
            self.wait(lambda: self.state == 'RETURN' and self.goal_x == 2.0)
            # Old timestamps, mismatched frames and NaN cannot advance points.
            for x, frame, stamp in [(2, 'local_frd', rospy.Time.now() - rospy.Duration(3)),
                                     (2, 'map', rospy.Time.now()),
                                     (float('nan'), 'local_frd', rospy.Time.now())]:
                self.send_odom(x, frame, stamp)
                self.assert_stays('RETURN')
                self.assertEqual(self.goal_x, 2.0)
            self.send_odom(2)
            self.wait(lambda: self.goal_x == 0)
            self.reset()

        # A latched old route and ready=true cannot start a new episode.
        self.reset()
        self.path.publish(self.route(stamp=rospy.Time.now() - rospy.Duration(5)))
        self.ready.publish(Bool(data=True))
        self.link.publish(String(data='LINK_LOST'))
        self.wait(lambda: self.state == 'LINK_LOST')
        self.assert_stays('LINK_LOST')

        # Empty path, endpoint mismatch and frame mismatch remain blocked.
        self.path.publish(Path())
        self.assert_stays('LINK_LOST')
        route = self.route()
        endpoint = self.route(end=9).poses[-1]
        self.endpoint.publish(endpoint)
        self.path.publish(route)
        self.assert_stays('LINK_LOST')
        endpoint = route.poses[-1]
        endpoint.header.frame_id = 'map'
        self.endpoint.publish(endpoint)
        self.assert_stays('LINK_LOST')

        # Singleton route at last-good pose enters WAIT_LINK, not LAND.
        endpoint.header.frame_id = 'local_frd'
        route.poses = [endpoint]
        self.endpoint.publish(endpoint)
        self.path.publish(route)
        self.send_odom(0)
        self.wait(lambda: self.state == 'WAIT_LINK')
        self.reset()


if __name__ == '__main__':
    rospy.init_node('homing_interfaces_test')
    rostest.rosrun('vio_homing', 'homing_interfaces', Interfaces)
