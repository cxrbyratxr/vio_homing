#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from sensor_msgs.msg import Imu

timestamps = []
COUNT = 1000


def callback(msg):
    timestamps.append(msg.header.stamp.to_sec())

    if len(timestamps) >= COUNT:
        analyze()
        rospy.signal_shutdown("Done")


def analyze():
    dt = []

    for i in range(1, len(timestamps)):
        dt.append(timestamps[i] - timestamps[i - 1])

    mean = sum(dt) / len(dt)
    variance = sum((x - mean) ** 2 for x in dt) / len(dt)
    std = math.sqrt(variance)
    minimum = min(dt)
    maximum = max(dt)
    large_gaps = [x for x in dt if x > 0.015]
    backwards = [x for x in dt if x <= 0]

    print("")
    print("=== IMU TIMING ===")
    print("Samples:        %d" % len(timestamps))
    print("Mean dt:        %.6f s" % mean)
    print("Mean rate:      %.2f Hz" % (1.0 / mean))
    print("Std deviation:  %.6f s" % std)
    print("Min dt:         %.6f s" % minimum)
    print("Max dt:         %.6f s" % maximum)
    print("Gaps > 15 ms:   %d" % len(large_gaps))
    print("Bad timestamps: %d" % len(backwards))


def main():
    rospy.init_node("check_imu_timing")
    rospy.Subscriber("/pixhawk/imu", Imu, callback, queue_size=200)

    print("Collecting %d IMU messages..." % COUNT)

    rospy.spin()


if __name__ == "__main__":
    main()
