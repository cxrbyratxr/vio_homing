# -*- coding: utf-8 -*-
"""Position conversion and command gates, independent of ROS/MAVLink I/O."""
import math
from vio_homing.homing_logic import finite, valid_point


class FrdToNed(object):
    """Fixed local FRD -> NED: p_N = Rz(yaw) p_F + origin; both have Z down."""

    def __init__(self, yaw_degrees, origin):
        if not finite(yaw_degrees) or not valid_point(origin):
            raise ValueError('Invalid FRD to NED alignment')
        self.yaw = math.radians(yaw_degrees)
        self._cos_yaw = math.cos(self.yaw)
        self._sin_yaw = math.sin(self.yaw)
        self.origin = tuple(origin)

    def convert(self, point):
        if not valid_point(point):
            raise ValueError('Nonfinite position')
        x, y, z = point
        c, s = self._cos_yaw, self._sin_yaw
        return (
            self.origin[0] + c * x - s * y, self.origin[1] + s * x + c * y,
            self.origin[2] + z
        )


class HomingTransport(object):

    def __init__(
        self,
        alignment,
        enabled=False,
        alignment_valid=False,
        frame='local_frd',
        input_timeout=0.5,
        odom_timeout=1.0
    ):
        self.alignment = alignment
        self.enabled = enabled
        self.alignment_valid = alignment_valid
        self.frame = frame
        self.input_timeout = input_timeout
        self.odom_timeout = odom_timeout
        self.session = None
        self.reset()

    def reset(self):
        self.state = 'NORMAL'
        self.state_time = None
        self.active = False
        self.active_time = None
        self.target = None
        self.target_time = None
        self.target_stamp = None
        self.odom_time = None
        self.odom_stamp = None
        self.land = False
        self.land_time = None
        self.eligible_land = False
        self.land_sent = False

    def set_session(self, value):
        if value and value != self.session:
            self.session = value
            self.reset()

    def set_state(self, value, now):
        self.state = value
        self.state_time = now
        if value == 'NORMAL':
            self.eligible_land = False
            self.land_sent = False
            self.land = False
            self.target = None

    def set_active(self, value, now):
        self.active, self.active_time = bool(value), now

    def set_target(self, point, frame, stamp, now):
        if frame != self.frame or not valid_point(point):
            self.target = None
            return
        self.target = tuple(point)
        self.target_stamp, self.target_time = stamp, now

    def set_odom(self, point, frame, stamp, now):
        if frame != self.frame or not valid_point(point):
            self.odom_time = None
            return
        self.odom_stamp, self.odom_time = stamp, now

    def set_land(self, value, now):
        self.land, self.land_time = bool(value), now

    def fresh(self, stamp, now, timeout):
        return stamp is not None and finite(
            stamp
        ) and 0 <= now - stamp <= timeout

    def odom_fresh(self, now):
        return (
            self.fresh(self.odom_time, now, self.odom_timeout)
            and self.fresh(self.odom_stamp, now, self.odom_timeout)
        )

    def next_setpoint(self, now, link_ok, timesync_ok):
        if not (
            self.enabled and self.alignment_valid and self.session and link_ok
            and timesync_ok and self.active
            and self.state in ('RETURN', 'WAIT_LINK')
            and self.fresh(self.state_time, now, self.input_timeout)
            and self.fresh(self.active_time, now, self.input_timeout)
            and self.fresh(self.target_time, now, self.input_timeout)
            and self.fresh(self.target_stamp, now, self.input_timeout)
            and self.odom_fresh(now) and self.target is not None
        ):
            return None
        return self.alignment.convert(self.target)

    def mark_setpoint_sent(self):
        self.eligible_land = True

    def should_land(self, now, link_ok):
        return (
            self.enabled and self.session and link_ok and self.state == 'LAND'
            and self.land and self.eligible_land and not self.land_sent
            and self.odom_fresh(now)
            and self.fresh(self.state_time, now, self.input_timeout)
            and self.fresh(self.land_time, now, self.input_timeout)
        )

    def mark_land_sent(self):
        self.land_sent = True

    def link_lost(self):
        # Discard unsent LAND; reconnection must not replay an old command.
        self.eligible_land = False
        self.target = None
        self.target_time = None
        self.odom_time = None
