#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import math

import rospy

from std_msgs.msg import Empty
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float32


class WifiLinkMonitor(object):

    def __init__(self):
        self.ok_timeout = float(rospy.get_param("~ok_timeout", 2.0))
        self.lost_timeout = float(rospy.get_param("~lost_timeout", 4.0))
        self.publish_rate = float(rospy.get_param("~publish_rate", 20.0))
        self.last_packet_time = None
        self.packet_count = 0
        self.current_state = "WAITING"
        self.previous_state = None

        self.state_pub = rospy.Publisher(
            "/vio_homing/link_state", String, queue_size=10
        )
        self.alive_pub = rospy.Publisher(
            "/vio_homing/link_alive", Bool, queue_size=10
        )
        self.age_pub = rospy.Publisher(
            "/vio_homing/link_age", Float32, queue_size=10
        )
        self.packet_count_pub = rospy.Publisher(
            "/vio_homing/link_packet_count", Float32, queue_size=10
        )

        self.qgc_sub = rospy.Subscriber(
            "/vio_homing/qgc_heartbeat", Empty, self.qgc_heartbeat, queue_size=10)
        if (not all(not math.isnan(v) and not math.isinf(v) for v in
                    (self.ok_timeout, self.lost_timeout, self.publish_rate))
                or not 0 < self.ok_timeout < self.lost_timeout or self.publish_rate <= 0):
            raise ValueError('Require 0 < ok_timeout < lost_timeout and positive publish_rate')
        rospy.loginfo("WiFi link monitor source: QGC heartbeat")
        rospy.loginfo(
            "LINK_OK < %.2f s, LINK_LOST >= %.2f s", self.ok_timeout,
            self.lost_timeout
        )

    def qgc_heartbeat(self, unused):
        self.last_packet_time = time.time()
        self.packet_count += 1

    def calculate_state(self):
        if self.last_packet_time is None:
            return ("WAITING", False, -1.0)

        age = time.time() - self.last_packet_time
        if age < self.ok_timeout:
            state = "LINK_OK"
            alive = True
        elif age < self.lost_timeout:
            state = "LINK_DEGRADED"
            alive = True
        else:
            state = "LINK_LOST"
            alive = False
        return (state, alive, age)

    def publish_state(self, state, alive, age):
        self.state_pub.publish(String(data=state))
        self.alive_pub.publish(Bool(data=alive))
        self.age_pub.publish(Float32(data=age))
        self.packet_count_pub.publish(Float32(data=float(self.packet_count)))

    def log_transition(self, state, age):
        if state == self.previous_state:
            return

        if state == "WAITING":
            rospy.logwarn("LINK STATE -> WAITING")
        elif state == "LINK_OK":
            rospy.loginfo("LINK STATE -> LINK_OK (age %.3f s)", age)
        elif state == "LINK_DEGRADED":
            rospy.logwarn("LINK STATE -> LINK_DEGRADED (age %.3f s)", age)
        elif state == "LINK_LOST":
            rospy.logerr("LINK STATE -> LINK_LOST (age %.3f s)", age)

        self.previous_state = state

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            state, alive, age = self.calculate_state()
            self.current_state = state
            self.log_transition(state, age)
            self.publish_state(state, alive, age)

            rate.sleep()


def main():
    rospy.init_node("wifi_link_monitor")
    monitor = WifiLinkMonitor()
    monitor.run()


if __name__ == "__main__":
    main()
