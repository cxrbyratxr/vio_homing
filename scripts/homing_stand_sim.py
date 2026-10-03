#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""End-to-end synthetic stand with the real UDP link monitor and recorder."""
import copy
import json
import socket
import sys
import threading
import time
import unittest

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, String


def xyz(pose):
    p = pose.position
    return (p.x, p.y, p.z)


class Stand(object):
    def __init__(self):
        self.lock = threading.RLock()
        self.heartbeat = False
        self.current_position = (0.0, 0.0, 0.0)
        self.link_state = None
        self.state = None
        self.active = None
        self.land = None
        self.goal = None
        self.endpoint = None
        self.path = None
        self.transitions = []
        self.goal_history = []
        self.land_history = []
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.port = int(rospy.get_param('~heartbeat_port', 15051))
        self.odom_pub = rospy.Publisher('/vio/odom_frd', Odometry, queue_size=10)
        self.subs = [
            rospy.Subscriber('/vio_homing/link_state', String, self.on_link),
            rospy.Subscriber('/vio_homing/homing_state', String, self.on_state),
            rospy.Subscriber('/vio_homing/return_active', Bool, self.on_active),
            rospy.Subscriber('/vio_homing/land_command', Bool, self.on_land),
            rospy.Subscriber('/vio_homing/return_goal', PoseStamped, self.on_goal),
            rospy.Subscriber('/vio_homing/return_path', Path, self.on_path),
            rospy.Subscriber('/vio_homing/last_good_link_pose', PoseStamped, self.on_endpoint)]
        self.timer = rospy.Timer(rospy.Duration(1.0 / 30.0), self.publish)

    def on_link(self, msg):
        with self.lock:
            self.link_state = msg.data

    def on_state(self, msg):
        with self.lock:
            if self.state != msg.data:
                self.transitions.append((time.time(), msg.data))
            self.state = msg.data

    def on_active(self, msg):
        with self.lock:
            self.active = msg.data

    def on_land(self, msg):
        with self.lock:
            if self.land != msg.data:
                self.land_history.append((time.time(), msg.data))
            self.land = msg.data

    def on_goal(self, msg):
        with self.lock:
            self.goal = copy.deepcopy(msg)
            self.goal_history.append((time.time(), xyz(msg.pose)))

    def on_path(self, msg):
        with self.lock:
            self.path = copy.deepcopy(msg)

    def on_endpoint(self, msg):
        with self.lock:
            self.endpoint = copy.deepcopy(msg)

    def publish(self, event):
        with self.lock:
            msg = Odometry()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'local_frd'
            msg.child_frame_id = 'body_frd'
            p = msg.pose.pose.position
            p.x, p.y, p.z = self.current_position
            msg.pose.pose.orientation.w = 1.0
            self.odom_pub.publish(msg)
            if self.heartbeat:
                self.sock.sendto(b'VIO_HOMING_HEARTBEAT stand', ('127.0.0.1', self.port))

    def wait(self, predicate, timeout, label):
        end = time.time() + timeout
        while not rospy.is_shutdown() and time.time() < end:
            with self.lock:
                if predicate():
                    return
            time.sleep(0.02)
        raise AssertionError('%s timed out: link=%s state=%s' %
                             (label, self.link_state, self.state))

    def hold(self, seconds, expected=None):
        end = time.time() + seconds
        while not rospy.is_shutdown() and time.time() < end:
            with self.lock:
                if expected is not None and self.state != expected:
                    raise AssertionError('Expected %s, got %s' % (expected, self.state))
            time.sleep(0.02)

    def recover(self):
        with self.lock:
            self.heartbeat = True
        self.wait(lambda: self.link_state == 'LINK_OK' and self.state == 'NORMAL'
                  and self.active is False and self.land is False, 5, 'recovery')
        # Allow already queued messages to drain; no goals may follow recovery.
        self.hold(0.2, 'NORMAL')
        with self.lock:
            count = len(self.goal_history)
        self.hold(0.4, 'NORMAL')
        with self.lock:
            if len(self.goal_history) != count:
                raise AssertionError('Return goals continued after recovery')

    def lose_link(self):
        self.recover()
        with self.lock:
            self.current_position = (0.0, 0.0, 0.0)
        self.hold(0.5, 'NORMAL')
        with self.lock:
            start = time.time()
            self.heartbeat = False
            self.goal = None
            first_goal = len(self.goal_history)
        self.wait(lambda: self.link_state == 'LINK_DEGRADED', 2, 'degradation')
        for x in (1.0, 2.0, 3.0, 4.0):
            with self.lock:
                self.current_position = (x, 0.0, 0.0)
            self.hold(0.25, 'NORMAL')
        self.wait(lambda: self.state == 'RETURN' and self.path is not None and
                  bool(self.path.poses) and self.goal is not None, 5, 'confirmed loss / return')
        elapsed = time.time() - start
        if elapsed < 2.8 or elapsed > 5.0:
            raise AssertionError('Return started outside confirmed-loss interval: %.3f s' % elapsed)
        with self.lock:
            snapshot = [xyz(p.pose) for p in self.path.poses]
            target = xyz(self.endpoint.pose)
        if snapshot[-1] != target:
            raise AssertionError('Recorded path endpoint does not match last-good pose')
        return snapshot, first_goal, elapsed

    def follow(self, snapshot, first_goal):
        end = time.time() + 15
        while not rospy.is_shutdown() and time.time() < end:
            with self.lock:
                if self.state == 'WAIT_LINK':
                    break
                if self.state != 'RETURN':
                    raise AssertionError('Unexpected state during return: %s' % self.state)
                if self.goal is not None:
                    self.current_position = xyz(self.goal.pose)
            time.sleep(0.02)
        self.wait(lambda: self.state == 'WAIT_LINK', 2, 'endpoint arrival')
        with self.lock:
            observed = [p for stamp, p in self.goal_history[first_goal:]]
        # Compare as an ordered subsequence, allowing republished goals and
        # adjacent repeated recorder points at a stationary location.
        cursor = 0
        for p in snapshot:
            while cursor < len(observed) and observed[cursor] != p:
                cursor += 1
            if cursor == len(observed):
                raise AssertionError('Missing/out-of-order waypoint %r in %r' % (p, observed))
            cursor += 1

    def run(self):
        self.wait(lambda: self.link_state == 'WAITING' and self.state == 'NORMAL'
                  and self.odom_pub.get_num_connections() >= 2, 15, 'stand startup')
        self.hold(0.3, 'NORMAL')
        snapshot, first, lost_delay = self.lose_link()
        self.recover()  # Recovery during RETURN.
        if self.land:
            raise AssertionError('LAND on recovery during RETURN')
        rospy.loginfo('PASS: recovery during RETURN cancels goals')

        snapshot, first, lost_delay = self.lose_link()
        self.follow(snapshot, first)
        self.hold(0.5, 'WAIT_LINK')
        self.recover()  # Recovery during WAIT_LINK.
        if self.land:
            raise AssertionError('LAND on recovery during WAIT_LINK')
        rospy.loginfo('PASS: recovery during WAIT_LINK cancels LAND')

        snapshot, first, lost_delay = self.lose_link()
        self.follow(snapshot, first)
        self.wait(lambda: self.land is False, 1, 'LAND initially false')
        # Use the received transition timestamp, not the end of follow().
        with self.lock:
            wait_start = [t for t, s in self.transitions if s == 'WAIT_LINK'][-1]
        self.hold(59.5, 'WAIT_LINK')
        self.wait(lambda: self.state == 'LAND' and self.land is True and
                  self.active is False, 5, 'LAND after 60 s')
        with self.lock:
            land_time = [t for t, value in self.land_history if value][-1]
            wait_duration = land_time - wait_start
        if wait_duration < 60.0:
            raise AssertionError('WAIT_LINK too short: %.6f s' % wait_duration)
        self.recover()
        lost_transition = [t for t, s in self.transitions if s == 'LINK_LOST'][-1]
        return_transition = [t for t, s in self.transitions if s == 'RETURN'][-1]
        result = dict(passed=True, wait_duration_seconds=wait_duration,
                      heartbeat_stop_to_return_seconds=lost_delay,
                      confirmed_loss_to_return_seconds=return_transition - lost_transition,
                      return_path_points=len(snapshot),
                      checks=['startup WAITING', 'confirmed UDP loss',
                              'existing recorder path and ordered goals',
                              'recovery during RETURN', 'recovery during WAIT_LINK',
                              'WAIT_LINK >= 60 s', 'LAND true', 'recovery after LAND'],
                      transitions=self.transitions)
        with open(rospy.get_param('~result_file', '/tmp/vio_homing_stand_result.json'), 'w') as handle:
            json.dump(result, handle, indent=2)
        rospy.loginfo('PASS: full stand, WAIT_LINK=%.3f s, path=%d points',
                      wait_duration, len(snapshot))
        return result

    def close(self):
        self.timer.shutdown()
        self.sock.close()


class StandTest(unittest.TestCase):
    def test_full_stand(self):
        stand = Stand()
        try:
            stand.run()
        finally:
            stand.close()


if __name__ == '__main__':
    rospy.init_node('homing_stand_sim')
    if '--rostest' in sys.argv:
        import rostest
        rostest.rosrun('vio_homing', 'homing_stand', StandTest)
    else:
        stand = Stand()
        try:
            stand.run()
        except Exception as exc:
            rospy.logerr('FAIL: %s', exc)
            sys.exit(1)
        finally:
            stand.close()
