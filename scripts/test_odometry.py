#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry


def main():
    rospy.init_node("test_odometry")
    pub = rospy.Publisher("/vio/odom_frd", Odometry, queue_size=10)
    rate = rospy.Rate(30)
    rospy.loginfo("Publishing test odometry at 30 Hz")

    while not rospy.is_shutdown():
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "local_frd"
        msg.child_frame_id = "body_frd"

        # Static test position:
        # X = 1.23 m forward
        # Y = 0
        # Z = 0
        msg.pose.pose.position.x = 1.23
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0

        # Identity quaternion.
        msg.pose.pose.orientation.w = 1.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0

        # Zero velocity.
        msg.twist.twist.linear.x = 0.0
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0

        # Zero angular velocity.
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = 0.0

        # Leave covariance arrays as zero.
        # The MAVLink bridge will interpret
        # all-zero covariance as unknown.
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    main()
