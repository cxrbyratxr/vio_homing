#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os

os.environ["MAVLINK20"] = "1"

import time
import threading
from collections import deque

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from pymavlink import mavutil
from vio_homing.mavlink_homing_adapter import MavlinkHomingAdapter


class MavlinkImuBridge(MavlinkHomingAdapter):

    def __init__(self):
        self.port = rospy.get_param("~serial_port", "/dev/ttyTHS1")
        self.baud = int(rospy.get_param("~baudrate", 115200))
        self.imu_topic = rospy.get_param("~imu_topic", "/pixhawk/imu")
        self.imu_frame_id = rospy.get_param("~imu_frame_id", "imu_link")
        self.odom_topic = rospy.get_param("~odom_topic", "/vio/odom_frd")
        self.odom_quality = int(rospy.get_param("~odom_quality", 0))
        self.timesync_rate = float(rospy.get_param("~timesync_rate", 10.0))
        self.max_rtt_ns = int(
            float(rospy.get_param("~max_timesync_rtt_ms", 50.0)) * 1000000.0
        )

        self.master = None
        # Protect all MAVLink TX operations.
        self.tx_lock = threading.RLock()

        # Protect TIMESYNC data shared between threads.
        self.timesync_lock = threading.Lock()
        self.offset_samples = deque(maxlen=30)
        self.clock_offset_ns = None
        self.last_timesync_request_ns = 0
        self.last_timesync_send = 0.0
        self.last_status_print = 0.0
        self.imu_count = 0
        self.odom_rx_count = 0
        self.odom_tx_count = 0
        self.odom_tx_error_count = 0
        self.last_imu_count = 0
        self.last_odom_rx_count = 0
        self.last_odom_tx_count = 0

        self.imu_pub = rospy.Publisher(self.imu_topic, Imu, queue_size=200)

        self.init_homing_adapter()

        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self.odom_callback, queue_size=50
        )

    def reset_timesync(self):
        with self.timesync_lock:
            self.offset_samples.clear()
            self.clock_offset_ns = None
            self.last_timesync_request_ns = 0
            self.last_timesync_send = 0.0

    def ros_now_ns(self):
        return int(rospy.Time.now().to_nsec())

    def median(self, values):
        values = sorted(values)
        count = len(values)

        if count == 0:
            return None

        middle = count // 2

        if count % 2:
            return values[middle]

        return (values[middle - 1] + values[middle]) // 2

    def connect(self):
        while time.time(
        ) < self.reconnect_not_before and not rospy.is_shutdown():
            time.sleep(0.1)
        self.reset_timesync()

        rospy.loginfo(
            "Opening MAVLink connection: %s @ %d", self.port, self.baud
        )

        self.master = mavutil.mavlink_connection(
            self.port,
            baud=self.baud,
            robust_parsing=True,
            source_system=245,
            source_component=191
        )

        rospy.loginfo("Waiting for PX4 heartbeat...")
        heartbeat = self.master.wait_heartbeat(timeout=15)

        if heartbeat is None:
            raise RuntimeError("PX4 heartbeat timeout")

        self.master.target_component = heartbeat.get_srcComponent()
        self.heartbeat_received(heartbeat)

        rospy.loginfo(
            "PX4 connected: system=%d component=%d",
            self.master.target_system,
            self.master.target_component,
        )

    def close(self):
        with self.control_lock:
            self.homing_disconnected()
            with self.tx_lock:
                if self.master is not None:
                    try:
                        self.master.close()
                    except Exception:
                        pass
                self.master = None

    def send_timesync_request(self):
        now_ns = self.ros_now_ns()

        with self.timesync_lock:
            self.last_timesync_request_ns = now_ns

        with self.tx_lock:
            self.master.mav.timesync_send(0, now_ns)

    def process_timesync(self, msg):
        now_ns = self.ros_now_ns()
        # Reply to a remote TIMESYNC request.
        if msg.tc1 == 0:
            with self.tx_lock:
                self.master.mav.timesync_send(now_ns, msg.ts1)
            return

        request_time_ns = int(msg.ts1)

        with self.timesync_lock:
            last_request_ns = self.last_timesync_request_ns

        if abs(request_time_ns - last_request_ns) > 1000000000:
            return

        rtt_ns = now_ns - request_time_ns
        if rtt_ns <= 0:
            return
        if rtt_ns > self.max_rtt_ns:
            return

        local_midpoint_ns = (request_time_ns + now_ns) // 2
        px4_time_ns = int(msg.tc1)
        # offset = PX4 clock - Jetson clock
        offset_ns = px4_time_ns - local_midpoint_ns

        with self.timesync_lock:
            self.offset_samples.append(offset_ns)

            if len(self.offset_samples) >= 5:
                self.clock_offset_ns = self.median(self.offset_samples)

    def get_clock_offset_ns(self):
        with self.timesync_lock:
            return self.clock_offset_ns

    def px4_time_to_ros(self, time_usec):
        clock_offset_ns = self.get_clock_offset_ns()

        if clock_offset_ns is None:
            return rospy.Time.now()

        px4_time_ns = int(time_usec) * 1000
        ros_time_ns = px4_time_ns - clock_offset_ns

        if ros_time_ns <= 0:
            return rospy.Time.now()

        secs = ros_time_ns // 1000000000
        nsecs = ros_time_ns % 1000000000

        return rospy.Time(secs, nsecs)

    def ros_time_to_px4_usec(self, stamp):
        clock_offset_ns = self.get_clock_offset_ns()

        if clock_offset_ns is None:
            return None

        if stamp.secs == 0 and stamp.nsecs == 0:
            ros_time_ns = self.ros_now_ns()
        else:
            ros_time_ns = int(stamp.to_nsec())

        px4_time_ns = ros_time_ns + clock_offset_ns

        if px4_time_ns <= 0:
            return None

        return px4_time_ns // 1000

    def publish_imu(self, msg):
        imu = Imu()
        imu.header.stamp = self.px4_time_to_ros(msg.time_usec)
        imu.header.frame_id = self.imu_frame_id
        # MAVLink body frame: FRD
        # ROS body frame: FLU
        imu.linear_acceleration.x = msg.xacc
        imu.linear_acceleration.y = -msg.yacc
        imu.linear_acceleration.z = -msg.zacc
        imu.angular_velocity.x = msg.xgyro
        imu.angular_velocity.y = -msg.ygyro
        imu.angular_velocity.z = -msg.zgyro
        imu.orientation_covariance[0] = -1.0
        self.imu_pub.publish(imu)
        self.imu_count += 1

    def covariance_to_mavlink(self, covariance):
        if covariance is None:
            return [float("nan")] + [0.0] * 20

        values = list(covariance)

        if len(values) != 36:
            return [float("nan")] + [0.0] * 20

        if not any(value != 0.0 for value in values):
            return [float("nan")] + [0.0] * 20

        result = []

        for row in range(6):
            for col in range(row, 6):
                result.append(float(values[row * 6 + col]))

        return result

    def odom_callback(self, msg):
        self.odom_rx_count += 1
        self.homing_odom_received(msg)

        if self.master is None:
            return

        time_usec = self.ros_time_to_px4_usec(msg.header.stamp)
        # Do not send odometry until
        # TIMESYNC has converged.
        if time_usec is None:
            return

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        quaternion = [
            orientation.w, orientation.x, orientation.y, orientation.z
        ]
        pose_covariance = self.covariance_to_mavlink(msg.pose.covariance)
        velocity_covariance = self.covariance_to_mavlink(msg.twist.covariance)
        frame_id = mavutil.mavlink.MAV_FRAME_LOCAL_FRD
        child_frame_id = mavutil.mavlink.MAV_FRAME_BODY_FRD
        estimator_type = getattr(mavutil.mavlink, "MAV_ESTIMATOR_TYPE_VIO", 3)
        base_args = [
            time_usec,
            frame_id,
            child_frame_id,
            position.x,
            position.y,
            position.z,
            quaternion,
            linear.x,
            linear.y,
            linear.z,
            angular.x,
            angular.y,
            angular.z,
            pose_covariance,
            velocity_covariance,
            0,
            estimator_type,
        ]

        try:
            with self.tx_lock:
                if self.master is None:
                    return
                try:
                    self.master.mav.odometry_send(
                        *(base_args + [self.odom_quality])
                    )

                except TypeError:
                    self.master.mav.odometry_send(*base_args)

            self.odom_tx_count += 1

        except Exception as error:
            self.odom_tx_error_count += 1

            rospy.logwarn_throttle(2.0, "ODOMETRY TX error: %s", str(error))

    def print_status(self):
        now = time.time()

        if (now - self.last_status_print) < 5.0:
            return

        self.last_status_print = now
        clock_offset_ns = self.get_clock_offset_ns()

        if clock_offset_ns is None:
            with self.timesync_lock:
                sample_count = len(self.offset_samples)

            rospy.logwarn("TIMESYNC not converged: %d samples", sample_count)

        else:
            with self.timesync_lock:
                sample_count = len(self.offset_samples)

            rospy.loginfo(
                "TIMESYNC OK: samples=%d offset=%.3f ms", sample_count,
                clock_offset_ns / 1000000.0
            )

        imu_delta = self.imu_count - self.last_imu_count
        odom_rx_delta = self.odom_rx_count - self.last_odom_rx_count
        odom_tx_delta = self.odom_tx_count - self.last_odom_tx_count

        rospy.loginfo(
            "Traffic: IMU RX=%d (+%d) "
            "ODOM RX=%d (+%d) "
            "ODOM TX=%d (+%d) "
            "TX ERR=%d",
            self.imu_count,
            imu_delta,
            self.odom_rx_count,
            odom_rx_delta,
            self.odom_tx_count,
            odom_tx_delta,
            self.odom_tx_error_count,
        )

        self.last_imu_count = self.imu_count
        self.last_odom_rx_count = self.odom_rx_count
        self.last_odom_tx_count = self.odom_tx_count

    def run(self):
        self.connect()

        while not rospy.is_shutdown():
            now = time.time()
            if self.master is None or not self.mavlink_alive():
                raise RuntimeError('PX4 heartbeat/connection lost')
            self.tick_homing_io()

            if self.timesync_rate > 0:
                timesync_period = 1.0 / self.timesync_rate

                if (now - self.last_timesync_send) >= timesync_period:
                    self.send_timesync_request()
                    self.last_timesync_send = now

            msg = self.master.recv_match(blocking=True, timeout=0.02)

            if msg is None:
                self.print_status()
                continue

            msg_type = msg.get_type()

            if msg_type == "BAD_DATA":
                continue

            if msg_type == "TIMESYNC":
                self.process_timesync(msg)

            elif msg_type == "HIGHRES_IMU":
                self.publish_imu(msg)

            elif msg_type == 'HEARTBEAT' and msg.get_srcSystem(
            ) == self.master.target_system:
                self.heartbeat_received(msg)

            elif msg_type == 'COMMAND_ACK' and msg.get_srcSystem(
            ) == self.master.target_system:
                self.command_ack_received(msg)

            elif msg_type == 'SERIAL_CONTROL' and self.bench_enabled:
                self.serial_control_received(msg)

            self.print_status()


def main():
    rospy.init_node("mavlink_imu_bridge")
    bridge = MavlinkImuBridge()

    while not rospy.is_shutdown():
        try:
            bridge.run()
        except rospy.ROSInterruptException:
            break

        except Exception as error:
            rospy.logerr("MAVLink bridge error: %s", str(error))

            bridge.close()
            if rospy.is_shutdown():
                break

            rospy.loginfo("Retrying MAVLink connection in 3 seconds...")

            time.sleep(3)

    bridge.close()


if __name__ == "__main__":
    main()
