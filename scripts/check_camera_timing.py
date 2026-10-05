#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

import rospy
from sensor_msgs.msg import Image

COUNT = 300
timestamps = []


def callback(msg):
    timestamps.append(msg.header.stamp.to_sec())
    if len(timestamps) >= COUNT:
        analyze()
        rospy.signal_shutdown("Done")


def analyze():
    dt = [b - a for a, b in zip(timestamps, timestamps[1:])]
    mean = sum(dt) / len(dt)
    std = math.sqrt(sum((x - mean) ** 2 for x in dt) / len(dt))
    gaps = sum(1 for x in dt if x > 1.5 * mean)  # пропущенный кадр
    backwards = sum(1 for x in dt if x <= 0)

    print("")
    print("=== CAMERA TIMING ===")
    print("Samples:        %d" % len(timestamps))
    print("Mean dt:        %.6f s" % mean)
    print("Mean rate:      %.2f Hz" % (1.0 / mean))
    print("Std deviation:  %.6f s" % std)
    print("Min dt:         %.6f s" % min(dt))
    print("Max dt:         %.6f s" % max(dt))
    print("Gaps > 1.5*dt:  %d" % gaps)
    print("Bad timestamps: %d" % backwards)
    print("Rate >= 20 Hz:  %s" % ("YES" if 1.0 / mean >= 20.0 else "NO"))


def main():
    rospy.init_node("check_camera_timing")
    topic = rospy.get_param("~image_topic", "/camera/image_raw")
    rospy.Subscriber(topic, Image, callback, queue_size=50)

    print("Collecting %d frames from %s..." % (COUNT, topic))

    rospy.spin()


if __name__ == "__main__":
    main()
