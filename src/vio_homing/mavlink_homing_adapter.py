# -*- coding: utf-8 -*-
"""Homing transport on the bridge's existing, locked MAVLink connection."""
import json
import re
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
        self.original_mode = None
        self.heartbeat_timeout = float(
            rospy.get_param('~px4_heartbeat_timeout', 2.5)
        )
        self.setpoint_count = 0
        self.land_count = 0
        self.last_setpoint_send = 0.0
        self.last_setpoint = None
        self.land_pending = None
        self.land_ack = None
        self.reconnect_not_before = 0.0
        self.console_pending = None
        self.bench_enabled = bool(
            rospy.get_param('~enable_bench_console', False)
        )
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
        if self.bench_enabled:
            self.console_pub = rospy.Publisher(
                '/vio_homing/px4_console_response', String, queue_size=20
            )
            self.control_subs.extend(
                [
                    rospy.Subscriber(
                        '/vio_homing/px4_console_request', String,
                        self.console_request_cb
                    ),
                    rospy.Subscriber(
                        '/vio_homing/bench_control', String,
                        self.bench_control_cb
                    )
                ]
            )
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
            if self.original_mode is None:
                self.original_mode = (
                    (msg.custom_mode >> 16) & 255,
                    (msg.custom_mode >> 24) & 255
                )

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
        if self.execution_mode == 'flight':
            return self.tick_flight_io()
        return self.tick_bench_homing_io()

    def tick_bench_homing_io(self):
        with self.control_lock:
            now = rospy.get_time()
            alive = self.mavlink_alive()
            # This stage is a disarmed bench. No ARM command exists here.
            usable = alive and self.px4_armed is False
            point = self.transport.next_setpoint(
                now, usable,
                self.get_clock_offset_ns() is not None
            )
            if point is not None and time.time(
            ) - self.last_setpoint_send >= 0.05:
                boot_usec = self.ros_time_to_px4_usec(rospy.Time.now())
                if boot_usec is not None:
                    with self.tx_lock:
                        if self.master is not None:
                            self.master.mav.set_position_target_local_ned_send(
                                (boot_usec // 1000) & 0xffffffff,
                                self.master.target_system,
                                self.master.target_component,
                                mavutil.mavlink.MAV_FRAME_LOCAL_NED, 3576,
                                point[0], point[1], point[2], 0, 0, 0, 0, 0, 0,
                                0, 0
                            )
                            self.last_setpoint_send = time.time()
                            self.setpoint_count += 1
                            self.last_setpoint = point
                            self.transport.mark_setpoint_sent()
                            self.tx_event(
                                dict(
                                    message='SET_POSITION_TARGET_LOCAL_NED',
                                    position_ned=point,
                                    coordinate_frame=1,
                                    type_mask=3576,
                                    count=self.setpoint_count,
                                    session=self.transport.session
                                )
                            )
            if self.transport.should_land(now, usable):
                with self.tx_lock:
                    if self.master is not None:
                        nan = float('nan')
                        self.master.mav.command_long_send(
                            self.master.target_system,
                            self.master.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, nan,
                            nan, nan, nan
                        )
                        self.transport.mark_land_sent()
                        self.land_count += 1
                        self.land_pending = time.time()
                        self.land_ack = 'PENDING'
                        self.tx_event(
                            dict(
                                message='COMMAND_LONG',
                                command=21,
                                count=self.land_count,
                                session=self.transport.session
                            )
                        )
            if self.land_pending is not None and time.time(
            ) - self.land_pending > 3.0:
                self.land_pending = None
                self.land_ack = 'ACK_TIMEOUT_NO_AUTOMATIC_REPLAY'
            self.poll_console()

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

    def console_request_cb(self, msg):
        try:
            request = json.loads(msg.data)
            command = request['command']
            allowed = command in ('ver all', 'mavlink status') or re.match(
                r'^listener (trajectory_setpoint|offboard_control_mode|'
                r'vehicle_command|'
                r'vehicle_command_ack|vehicle_status|vehicle_local_position|'
                r'vehicle_visual_odometry|vehicle_attitude) [1-5]$', command
            )
            if not allowed or len(command) > 68:
                raise ValueError('Read-only bench command not allowed')
            with self.control_lock:
                if self.console_pending is not None or self.health_probe.pending or not self.mavlink_alive(
                ):
                    raise ValueError('Console busy or link unavailable')
                data = bytearray((command + '\n').encode('utf-8'))
                self.console_pending = dict(
                    id=request['id'], text='', started=time.time(), last=None
                )
                with self.tx_lock:
                    self.master.mav.serial_control_send(
                        mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL,
                        mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND
                        | mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
                        | mavutil.mavlink.SERIAL_CONTROL_FLAG_MULTI, 0, 0,
                        len(data),
                        list(data) + [0] * (70 - len(data))
                    )
        except Exception as error:
            self.console_pub.publish(
                String(
                    data=json.dumps(
                        dict(
                            id=locals().get('request', {}).get('id'),
                            error=str(error)
                        )
                    )
                )
            )

    def serial_control_received(self, msg):
        with self.control_lock:
            if self.console_pending is not None and msg.count:
                self.console_pending['text'] += bytearray(
                    msg.data[:msg.count]
                ).decode('utf-8', 'replace')
                self.console_pending['last'] = time.time()

    def poll_console(self):
        pending = self.console_pending
        if pending is None:
            return
        quiet = pending['last'] is not None and time.time() - \
            pending['last'] > 0.5
        if quiet or time.time() - pending['started'] > 8.0:
            self.console_pub.publish(
                String(
                    data=json.
                    dumps(dict(id=pending['id'], text=pending['text']))
                )
            )
            with self.tx_lock:
                if self.master is not None:
                    self.master.mav.serial_control_send(
                        10, 0, 0, 0, 0, [0] * 70
                    )
            self.console_pending = None

    def bench_control_cb(self, msg):
        try:
            request = json.loads(msg.data)
            with self.control_lock:
                if not self.mavlink_alive() or self.px4_armed is not False:
                    raise ValueError(
                        'Bench control requires disarmed PX4 and live link'
                    )
                action = request['action']
                if action in ('offboard', 'restore'):
                    mode = (
                        6, 0
                    ) if action == 'offboard' else self.original_mode
                    with self.tx_lock:
                        self.master.mav.command_long_send(
                            self.master.target_system,
                            self.master.target_component,
                            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, mode[0],
                            mode[1], 0, 0, 0, 0
                        )
                    self.tx_event(
                        dict(
                            message='COMMAND_LONG',
                            command=176,
                            bench_action=action,
                            mode=mode
                        )
                    )
                elif action == 'disconnect':
                    seconds = float(request.get('seconds', 4.0))
                    if not 1 <= seconds <= 10:
                        raise ValueError('Disconnect interval must be 1..10 s')
                    self.reconnect_not_before = time.time() + seconds
                    self.close()
                    self.tx_event(
                        dict(
                            message='BENCH_SERIAL_DISCONNECT', seconds=seconds
                        )
                    )
                else:
                    raise ValueError('Unknown bench action')
        except Exception as error:
            rospy.logerr('Bench control refused: %s', error)
