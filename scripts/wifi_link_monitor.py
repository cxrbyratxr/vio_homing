#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket
import time

import rospy

from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float32


class WifiLinkMonitor(object):

    def __init__(self):
        self.bind_address = rospy.get_param("~bind_address", "0.0.0.0")
        self.port = int(rospy.get_param("~port", 15050))
        self.ok_timeout = float(rospy.get_param("~ok_timeout", 0.5))
        self.lost_timeout = float(rospy.get_param("~lost_timeout", 3.0))
        self.publish_rate = float(rospy.get_param("~publish_rate", 20.0))
        self.expected_prefix = rospy.get_param("~heartbeat_prefix", "VIO_HOMING_HEARTBEAT")

        self.last_packet_time = None
        self.last_sender = None
        self.packet_count = 0
        self.current_state = "WAITING"
        self.previous_state = None

        self.state_pub = rospy.Publisher("/vio_homing/link_state", String, queue_size=10)
        self.alive_pub = rospy.Publisher("/vio_homing/link_alive", Bool, queue_size=10)
        self.age_pub = rospy.Publisher("/vio_homing/link_age", Float32, queue_size=10)
        self.packet_count_pub = rospy.Publisher(
            "/vio_homing/link_packet_count", Float32, queue_size=10
        )

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.bind_address, self.port))
        self.sock.setblocking(False)

        rospy.loginfo("WiFi link monitor listening on UDP %s:%d", self.bind_address, self.port)
        rospy.loginfo("LINK_OK < %.2f s, LINK_LOST >= %.2f s", self.ok_timeout, self.lost_timeout)

    def receive_packets(self):
        while True:
            try:
                data, address = self.sock.recvfrom(1024)
            except socket.error:
                break

            if not data:
                continue
            try:
                text = data.decode("utf-8")
            except Exception:
                continue
            if not text.startswith(self.expected_prefix):
                continue

            self.last_packet_time = time.time()
            self.last_sender = address
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
            self.receive_packets()

            state, alive, age = self.calculate_state()
            self.current_state = state
            self.log_transition(state, age)
            self.publish_state(state, alive, age)

            rate.sleep()

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass


def main():
    rospy.init_node("wifi_link_monitor")
    monitor = WifiLinkMonitor()
    try:
        monitor.run()
    finally:
        monitor.close()


if __name__ == "__main__":
    main()
