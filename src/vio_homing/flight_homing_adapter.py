# -*- coding: utf-8 -*-
"""PX4 telemetry, automatic frame registration and offboard mode handover."""
import struct
import time
from collections import deque
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from pymavlink import mavutil
from vio_homing.px4_alignment import Alignment
from vio_homing.flight_control import FlightControl, OFFBOARD, HOLD


class FlightHomingAdapter(object):
    def init_flight_control(self):
        self.execution_mode = rospy.get_param('~homing_execution_mode', 'bench')
        if self.execution_mode not in ('bench', 'flight'):
            raise ValueError('homing_execution_mode must be bench or flight')
        self.auto_alignment = bool(rospy.get_param('~auto_frd_alignment', False))
        self.alignment = Alignment()
        self.flight = FlightControl()
        self.local_poses = deque(maxlen=50)
        self.px4_pose = None
        self.px4_pose_received = None
        self.px4_pose_boot = None
        self.px4_mode = None
        self.px4_critical = False
        self.estimator_flags = 0
        self.estimator_received = None
        self.landed_state = None
        self.landed_received = None
        self.last_flight_phase = None
        self.profile_values = {}
        self.profile_times = {}
        self.profile_names = ['EKF2_EN', 'EKF2_EV_CTRL', 'EKF2_HGT_REF', 'EKF2_EV_NOISE_MD',
                              'COM_DLL_EXCEPT', 'COM_RCL_EXCEPT', 'NAV_DLL_ACT',
                              'COM_OBL_RC_ACT', 'COM_OF_LOSS_T']
        self.profile_requested = 0.
        self.feedback_pub = rospy.Publisher('/vio_homing/px4_odometry', Odometry, queue_size=10)
        self.vins_health_sub = rospy.Subscriber('/vio_homing/vins_adapter_status', String, self.vins_health_cb)
        if self.auto_alignment:
            self.transport.alignment_valid = False

    def vins_health_cb(self, msg):
        if msg.data.startswith('FAULT'):
            with self.control_lock:
                self.alignment.invalidate('VINS_ADAPTER_FAULT')
                if self.auto_alignment:
                    self.transport.alignment_valid = False

    def request_flight_telemetry(self):
        if self.execution_mode != 'flight' and not self.auto_alignment:
            return
        for message, hz in [(331, 10), (230, 5), (245, 2)]:
            with self.tx_lock:
                self.master.mav.command_long_send(
                    self.master.target_system, self.master.target_component,
                    511, 0, message, 1000000.0/hz, 0, 0, 0, 0, 0)
            time.sleep(.05)

    def request_control_profile(self):
        if self.master is None:
            return
        self.profile_requested = time.time()
        for name in self.profile_names:
            with self.tx_lock:
                self.master.mav.param_request_read_send(self.master.target_system,
                    self.master.target_component, name.encode('ascii'), -1)

    def profile_ready(self):
        values = self.profile_values
        exact = {'EKF2_EN': 1, 'EKF2_EV_CTRL': 7, 'EKF2_HGT_REF': 3,
                 'EKF2_EV_NOISE_MD': 1, 'NAV_DLL_ACT': 3, 'COM_OBL_RC_ACT': 4}
        return (all(name in values for name in self.profile_names)
                and all(0 <= time.time()-self.profile_times.get(k, 0) < 30. for k in self.profile_names)
                and all(abs(values[k]-v) < 1e-5 for k, v in exact.items())
                and int(values['COM_DLL_EXCEPT']) & 4 == 4
                and int(values['COM_RCL_EXCEPT']) & 6 == 6
                and .1 <= values['COM_OF_LOSS_T'] <= 1.)

    def flight_heartbeat(self, msg):
        self.px4_mode = ((msg.custom_mode >> 16) & 255, (msg.custom_mode >> 24) & 255)
        self.px4_critical = msg.system_status in (mavutil.mavlink.MAV_STATE_CRITICAL,
                                                 mavutil.mavlink.MAV_STATE_EMERGENCY,
                                                 getattr(mavutil.mavlink, 'MAV_STATE_FLIGHT_TERMINATION', 8))

    def estimator_healthy(self, now):
        # ATTITUDE, POS_HORIZ_REL, POS_VERT_ABS; reject constant-position mode.
        required = 1 | 8 | 32
        return (self.estimator_received is not None and 0 <= now-self.estimator_received < 1.
                and self.estimator_flags & required == required
                and not self.estimator_flags & 128)

    def flight_telemetry_received(self, msg):
        with self.control_lock:
            now = rospy.get_time()
            kind = msg.get_type()
            if kind == 'PARAM_VALUE':
                name = msg.param_id.rstrip('\x00')
                if name in self.profile_names:
                    self.profile_times[name] = time.time()
                    if msg.param_type == 6:
                        self.profile_values[name] = struct.unpack('<i', struct.pack('<f', msg.param_value))[0]
                    elif msg.param_type == 9 and np.isfinite(msg.param_value):
                        self.profile_values[name] = float(msg.param_value)
            elif kind == 'ESTIMATOR_STATUS':
                self.estimator_flags = int(msg.flags)
                self.estimator_received = now
            elif kind == 'EXTENDED_SYS_STATE':
                self.landed_state = int(msg.landed_state)
                self.landed_received = now
            elif kind == 'ODOMETRY':
                if msg.frame_id != mavutil.mavlink.MAV_FRAME_LOCAL_NED or msg.estimator_type != getattr(mavutil.mavlink, 'MAV_ESTIMATOR_TYPE_AUTOPILOT', 8):
                    return
                q = [msg.q[1], msg.q[2], msg.q[3], msg.q[0]]
                p = [msg.x, msg.y, msg.z]
                if not np.isfinite(p+q).all() or abs(np.linalg.norm(q)-1) > .05:
                    return
                if self.get_clock_offset_ns() is None:
                    return
                if self.px4_pose_boot is not None and msg.time_usec < self.px4_pose_boot:
                    self.alignment.invalidate('PX4_CLOCK_RESTART')
                self.px4_pose_boot = msg.time_usec
                stamp = self.px4_time_to_ros(msg.time_usec).to_sec()
                if not 0 <= now-stamp <= .5:
                    return
                self.px4_pose = (stamp, p, q, int(msg.reset_counter))
                self.px4_pose_received = now
                out = Odometry()
                out.header.stamp = rospy.Time.from_sec(stamp)
                out.header.frame_id = 'px4_local_ned'
                out.child_frame_id = 'body_frd'
                out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = p
                out.pose.pose.orientation.x, out.pose.pose.orientation.y, out.pose.pose.orientation.z, out.pose.pose.orientation.w = q
                self.feedback_pub.publish(out)
            self.update_auto_alignment(now)

    def flight_local_pose(self, msg):
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        self.local_poses.append((msg.header.stamp.to_sec(), [p.x, p.y, p.z], [q.x, q.y, q.z, q.w]))
        self.update_auto_alignment(rospy.get_time())

    def update_auto_alignment(self, now):
        if (self.alignment.fault and self.alignment.reason.startswith('MAVLINK_')
                and self.px4_armed is False and self.landed_state == 1
                and self.landed_received is not None and 0 <= now-self.landed_received < 2.):
            self.alignment = Alignment()
        if not self.auto_alignment or self.px4_pose is None or not self.local_poses:
            return
        if not self.estimator_healthy(now) or not 0 <= now-self.px4_pose[0] < .5:
            return
        closest = min(self.local_poses, key=lambda p: abs(p[0]-self.px4_pose[0]))
        if abs(closest[0]-self.px4_pose[0]) > .12:
            return
        self.alignment.observe(closest, self.px4_pose, self.px4_armed is False)
        self.transport.alignment_valid = self.alignment.valid
        if self.alignment.valid:
            self.transport.alignment = self.alignment.transform

    def flight_disconnected(self):
        self.px4_pose = None
        self.px4_pose_received = None
        self.estimator_received = None
        self.profile_values = {}
        self.profile_times = {}
        self.profile_requested = 0.
        self.landed_received = None
        self.local_poses.clear()
        if self.auto_alignment:
            self.alignment.invalidate('MAVLINK_RECONNECTED_REQUIRES_NEW_SESSION')
            self.transport.alignment_valid = False

    def flight_status(self):
        transform = self.transport.alignment
        return dict(execution_mode=self.execution_mode, control_profile_ready=self.profile_ready(),
                    control_profile_values=self.profile_values, control_phase=self.flight.phase,
                    control_reason=self.flight.reason, mode=self.px4_mode,
                    landed_state=self.landed_state, estimator_flags=self.estimator_flags,
                    alignment_reason=self.alignment.reason,
                    alignment_yaw_deg=float(np.degrees(transform.yaw)),
                    alignment_origin=list(transform.origin))

    def send_flight_position(self, point):
        boot = self.ros_time_to_px4_usec(rospy.Time.now())
        if boot is None:
            return
        with self.tx_lock:
            if self.master is None:
                return
            self.master.mav.set_position_target_local_ned_send(
                (boot//1000) & 0xffffffff, self.master.target_system,
                self.master.target_component, 1, 3576,
                point[0], point[1], point[2], 0, 0, 0, 0, 0, 0, 0, 0)
        self.setpoint_count += 1
        self.last_setpoint = point
        self.tx_event(dict(message='SET_POSITION_TARGET_LOCAL_NED', position_ned=point,
                           coordinate_frame=1, type_mask=3576, count=self.setpoint_count,
                           control_phase=self.flight.phase))

    def send_flight_command(self, command):
        with self.tx_lock:
            if self.master is None:
                return
            if command == 'LAND':
                nan = float('nan')
                self.master.mav.command_long_send(self.master.target_system,
                    self.master.target_component, 21, 0, 0, 0, 0, nan, nan, nan, nan)
                self.land_count += 1
                self.land_pending = time.time()
                self.land_ack = 'PENDING'
                self.transport.mark_land_sent()
            else:
                mode = OFFBOARD if command == 'OFFBOARD' else HOLD
                self.master.mav.command_long_send(self.master.target_system,
                    self.master.target_component, 176, 0, 1, mode[0], mode[1], 0, 0, 0, 0)
        self.tx_event(dict(message='COMMAND_LONG', command=21 if command == 'LAND' else 176,
                           purpose=command, control_phase=self.flight.phase))

    def tick_flight_io(self):
        with self.control_lock:
            now = rospy.get_time()
            alive = self.mavlink_alive()
            if alive and time.time()-self.profile_requested > 10.:
                self.request_control_profile()
            profile_ok = self.profile_ready()
            if not profile_ok and not self.flight.owned and self.flight.phase not in ('REQUEST_OFFBOARD', 'REQUEST_LAND', 'RELEASING'):
                self.flight.reason = 'BLOCKED_PX4_PROFILE_NOT_CONFIRMED'
                return
            nav = (self.estimator_healthy(now) and self.px4_pose is not None
                   and 0 <= now-self.px4_pose[0] <= .5)
            state_fresh = self.transport.fresh(self.transport.state_time, now, .5)
            healthy = (profile_ok and nav and self.transport.alignment_valid and self.transport.odom_fresh(now)
                       and self.get_clock_offset_ns() is not None and state_fresh)
            target = self.transport.next_setpoint(now, alive, self.get_clock_offset_ns() is not None)
            airborne = (self.landed_state == 2 and self.landed_received is not None
                        and 0 <= now-self.landed_received < 2.)
            if not self.transport.enabled:
                return
            point, command = self.flight.step(
                now, self.transport.session, self.px4_armed is True, airborne, alive,
                self.px4_mode, self.transport.state in ('RETURN', 'WAIT_LINK'),
                self.transport.state == 'NORMAL' and state_fresh,
                self.transport.state == 'LAND' and self.transport.eligible_land,
                healthy and (target is not None or self.transport.state in ('LAND', 'NORMAL')),
                self.px4_pose[1] if nav else None, target, self.px4_critical)
            if point is not None and time.time()-self.last_setpoint_send >= .05:
                self.send_flight_position(point)
                self.last_setpoint_send = time.time()
                if self.flight.phase == 'ACTIVE':
                    self.transport.mark_setpoint_sent()
            if command:
                self.send_flight_command(command)
            if self.land_pending is not None and time.time()-self.land_pending > 5.:
                self.land_ack = 'ACK_TIMEOUT_NO_AUTOMATIC_REPLAY'
                self.land_pending = None
            if self.flight.phase != self.last_flight_phase:
                rospy.logwarn('Homing control: %s (%s)', self.flight.phase, self.flight.reason)
                self.last_flight_phase = self.flight.phase
            self.poll_console()
