# -*- coding: utf-8 -*-
"""Homing transport on the bridge's existing, locked MAVLink connection."""
import json
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from pymavlink import mavutil
from vio_homing.homing_transport import FrdToNed, HomingTransport
from vio_homing.flight_homing_adapter import FlightHomingAdapter


def xyz(pose):
    return (pose.position.x, pose.position.y, pose.position.z)


class MavlinkHomingAdapter(FlightHomingAdapter):

    def init_homing_adapter(self):
        self.control_lock = threading.RLock()
        self.transport = HomingTransport(
            FrdToNed(
                float(rospy.get_param('~frd_to_ned_yaw_deg', 0.0)),
                rospy.get_param('~frd_origin_ned', [0.0, 0.0, 0.0])
            ),
            enabled=bool(rospy.get_param('~enable_homing_commands', False)),
            alignment_valid=bool(
                rospy.get_param('~frd_to_ned_alignment_valid', False)
            )
        )
        self.last_px4_heartbeat = None
        self.px4_armed = None
        self.heartbeat_timeout = float(
            rospy.get_param('~px4_heartbeat_timeout', 2.5)
        )
        self.setpoint_count = 0
        self.land_count = 0
        self.last_setpoint_send = 0.0
        self.last_setpoint = None
        self.land_pending = None
        self.land_ack = None
        self.tx_event_pub = rospy.Publisher(
            '/vio_homing/mavlink_tx_event', String, queue_size=100
        )
        self.ack_pub = rospy.Publisher(
            '/vio_homing/mavlink_command_ack', String, queue_size=20
        )
        self.transport_pub = rospy.Publisher(
            '/vio_homing/mavlink_transport_status',
            String,
            queue_size=10,
            latch=True
        )
        self.init_flight_control()
        self.control_subs = [
            rospy.Subscriber(
                '/vio_homing/homing_session', String, self.homing_session_cb
            ),
            rospy.Subscriber(
                '/vio_homing/homing_state', String, self.homing_state_cb
            ),
            rospy.Subscriber(
                '/vio_homing/return_active', Bool, self.homing_active_cb
            ),
            rospy.Subscriber(
                '/vio_homing/return_goal', PoseStamped, self.homing_goal_cb
            ),
            rospy.Subscriber(
                '/vio_homing/land_command', Bool, self.homing_land_cb
            )
        ]
        self.transport_timer = rospy.Timer(
            rospy.Duration(0.2), self.publish_transport_status
        )

    def homing_session_cb(self, msg):
        with self.control_lock:
            self.transport.set_session(msg.data)

    def homing_state_cb(self, msg):
        with self.control_lock:
            self.transport.set_state(msg.data, rospy.get_time())

    def homing_active_cb(self, msg):
        with self.control_lock:
            self.transport.set_active(msg.data, rospy.get_time())

    def homing_goal_cb(self, msg):
        with self.control_lock:
            self.transport.set_target(
                xyz(msg.pose), msg.header.frame_id, msg.header.stamp.to_sec(),
                rospy.get_time()
            )

    def homing_land_cb(self, msg):
        with self.control_lock:
            self.transport.set_land(msg.data, rospy.get_time())

    def homing_odom_received(self, msg):
        with self.control_lock:
            self.transport.set_odom(
                xyz(msg.pose.pose), msg.header.frame_id,
                msg.header.stamp.to_sec(), rospy.get_time()
            )
            self.flight_local_pose(msg)

    def heartbeat_received(self, msg):
        with self.control_lock:
            self.last_px4_heartbeat = time.time()
            self.px4_armed = bool(
                msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            self.flight_heartbeat(msg)

    def mavlink_alive(self):
        return (
            self.master is not None and self.last_px4_heartbeat is not None
            and 0 <= time.time() - self.last_px4_heartbeat <=
            self.heartbeat_timeout
        )

    def homing_disconnected(self):
        with self.control_lock:
            self.last_px4_heartbeat = None
            self.transport.link_lost()
            self.flight_disconnected()
            if self.land_pending is not None:
                self.land_ack = 'LINK_LOST_BEFORE_ACK'
                self.land_pending = None

    def tx_event(self, event):
        """Serialize transient diagnostics only for connected subscribers."""
        if self.tx_event_pub.get_num_connections() == 0:
            return
        event['ros_time'] = rospy.get_time()
        self.tx_event_pub.publish(String(data=json.dumps(event)))

    def tick_homing_io(self):
        return self.tick_flight_io()

    def command_ack_received(self, msg):
        if (
            getattr(msg, 'target_system', 245) not in (0, 245)
            or getattr(msg, 'target_component', 191) not in (0, 191)
        ):
            return
        with self.control_lock:
            result = int(msg.result)
            self.flight.acknowledge(int(msg.command), result)
            entry = mavutil.mavlink.enums['MAV_RESULT'].get(result)
            name = entry.name if entry is not None else str(result)
            if int(msg.command) == 21 and self.land_pending is not None:
                self.land_ack = name
                if result != mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                    self.land_pending = None
            self.ack_pub.publish(
                String(
                    data=json.dumps(
                        dict(
                            command=int(msg.command),
                            result=result,
                            result_name=name,
                            source_system=msg.get_srcSystem(),
                            source_component=msg.get_srcComponent(),
                            ros_time=rospy.get_time()
                        )
                    )
                )
            )

    def publish_transport_status(self, event):
        with self.control_lock:
            data = dict(
                link_alive=self.mavlink_alive(),
                armed=self.px4_armed,
                timesync=self.get_clock_offset_ns() is not None,
                enabled=self.transport.enabled,
                alignment_valid=self.transport.alignment_valid,
                session=self.transport.session,
                homing_state=self.transport.state,
                setpoint_tx=self.setpoint_count,
                last_setpoint_ned=self.last_setpoint,
                land_tx=self.land_count,
                land_ack=self.land_ack,
                odom_rx=self.odom_rx_count,
                odom_tx=self.odom_tx_count,
                odom_tx_errors=self.odom_tx_error_count,
                imu_rx=self.imu_count
            )
            data.update(self.flight_status())
            self.transport_pub.publish(String(data=json.dumps(data)))
