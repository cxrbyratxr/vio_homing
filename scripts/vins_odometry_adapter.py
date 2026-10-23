#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Publish audited VINS poses for the existing FRD ROS interface."""
import threading
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from vio_homing.vins_frames import convert, angular_velocity, pose_covariance


class Adapter(object):
    def __init__(self):
        self.lock = threading.RLock()
        self.previous = None
        self.fault = None
        self.last_receipt = None
        self.timeout = float(rospy.get_param('~timeout', 1.0))
        self.max_age = float(rospy.get_param('~max_age', 0.5))
        self.max_step = float(rospy.get_param('~max_step', 1.0))
        self.stamp_offset = float(rospy.get_param('~stamp_offset', -0.03275171186939631))
        self.input = rospy.get_param('~input_topic', '/vins_estimator/odometry')
        self.output = rospy.get_param('~output_topic', '/vio/odom_frd')
        if self.input == self.output:
            raise ValueError('Input and output topics must differ')
        for value in [self.timeout, self.max_age, self.max_step]:
            if not np.isfinite(value) or value <= 0:
                raise ValueError('Invalid adapter limit')
        if not np.isfinite(self.stamp_offset):
            raise ValueError('Invalid stamp offset')
        self.pub = rospy.Publisher(self.output, Odometry, queue_size=10)
        self.status = rospy.Publisher('/vio_homing/vins_adapter_status', String,
                                      queue_size=1, latch=True)
        self.status.publish('WAITING_FOR_VINS')
        self.sub = rospy.Subscriber(self.input, Odometry, self.callback, queue_size=10)
        self.imu_health_sub = rospy.Subscriber('/vio_homing/imu_status', String, self.imu_health, queue_size=1)
        self.restart_sub = rospy.Subscriber('/vins_restart', Bool, self.restart)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.tick)

    def fail(self, reason):
        if self.fault is None:
            self.fault = reason
            self.status.publish('FAULT: ' + reason)
            rospy.logerr('VINS adapter stopped: %s. Restart the adapter and reset route before reuse.', reason)

    def imu_health(self, msg):
        with self.lock:
            if msg.data.startswith('FAULT'):
                self.fail('IMU session invalid: ' + msg.data)
            elif msg.data == 'WAITING_FOR_SYNC' and self.previous is not None:
                self.fail('IMU bridge restarted during VINS session')

    def restart(self, msg):
        if msg.data:
            with self.lock:
                self.fail('VINS restart requested')

    def tick(self, event):
        with self.lock:
            if self.last_receipt is not None and (rospy.Time.now()-self.last_receipt).to_sec() > self.timeout:
                self.fail('VINS stream timeout')

    def callback(self, msg):
        with self.lock:
            if self.fault:
                return
            now = rospy.Time.now()
            stamp = msg.header.stamp + rospy.Duration.from_sec(self.stamp_offset)
            age = (now-stamp).to_sec()
            if msg.header.frame_id != 'world' or msg.child_frame_id != 'world':
                self.fail('Unexpected VINS frame labels')
                return
            if stamp.to_sec() <= 0 or not np.isfinite(age):
                self.fail('Invalid timestamp')
                return
            if age < -0.1 or age > self.max_age:
                if self.previous is None and age > self.max_age:
                    # Initialization may release an old optimized pose. Never
                    # publish it, but wait for fresh data before starting.
                    self.status.publish('WAITING_FOR_FRESH_VINS')
                    return
                self.fail('Stale or future VINS pose')
                return
            p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
            orientation = [q.x, q.y, q.z, q.w]
            dt = None
            try:
                position, converted_q, velocity = convert([p.x, p.y, p.z], orientation, [v.x, v.y, v.z])
                covariance = pose_covariance(msg.pose.covariance)
                omega = np.zeros(3)
                if self.previous is not None:
                    old_stamp, old_position, old_orientation, old_seq = self.previous
                    dt = (stamp-old_stamp).to_sec()
                    if dt <= 0 or dt > self.timeout or msg.header.seq <= old_seq:
                        raise ValueError('Timestamp/sequence discontinuity')
                    step = float(np.linalg.norm(position-old_position))
                    if step > self.max_step:
                        rospy.logwarn_throttle(1.0,
                            'VINS position jump %.3f m in %.3f s; forwarding by operator request' % (step, dt))
                    omega = angular_velocity(old_orientation, orientation, dt)
            except (ValueError, TypeError, OverflowError) as error:
                self.fail(str(error))
                return
            self.previous = (stamp, position, orientation, msg.header.seq)
            self.last_receipt = now
            # First pose establishes the derivative baseline; do not invent angular velocity.
            if dt is None:
                self.status.publish('WAITING_FOR_SECOND_POSE')
                return
            out = Odometry()
            out.header.stamp = stamp
            out.header.frame_id = 'local_frd'
            out.child_frame_id = 'body_frd'
            out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = position
            out.pose.pose.orientation.x, out.pose.pose.orientation.y, out.pose.pose.orientation.z, out.pose.pose.orientation.w = converted_q
            out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = velocity
            out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z = omega
            out.pose.covariance = covariance
            # VINS exposes no angular covariance; zero = unspecified to existing bridge.
            self.pub.publish(out)
            self.status.publish('ACTIVE')


if __name__ == '__main__':
    rospy.init_node('vins_odometry_adapter')
    adapter = Adapter()
    rospy.spin()
