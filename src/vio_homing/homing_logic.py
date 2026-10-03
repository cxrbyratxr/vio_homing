# -*- coding: utf-8 -*-
"""ROS independent state machine, compatible with Python 2.7.

The caller supplies time in seconds and serializes access. Inputs and route
coordinates must use one local frame. A return episode ends only on LINK_OK.
"""
import math


def finite(value):
    return not (math.isnan(value) or math.isinf(value))


def valid_point(point):
    return len(point) == 3 and all(finite(float(v)) for v in point)


def distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


class HomingLogic(object):
    def __init__(self, arrival_radius=0.4, wait_link_seconds=60.0,
                 odom_timeout=1.0):
        values = (arrival_radius, wait_link_seconds, odom_timeout)
        if (not all(finite(v) for v in values) or arrival_radius <= 0 or
                odom_timeout <= 0 or wait_link_seconds < 60.0):
            raise ValueError('radius/timeout must be positive; wait must be >= 60 s')
        self.arrival_radius = arrival_radius
        self.wait_link_seconds = wait_link_seconds
        self.odom_timeout = odom_timeout
        self.state = 'NORMAL'
        self.link = 'WAITING'
        self.seen_link_ok = False
        self.last_good_link_time = None
        self.path = ()
        self.frame = None
        self.last_good = None
        self.index = 0
        self.odom = None
        self.odom_frame = None
        self.odom_received = None
        self.odom_stamp = None
        self.wait_started = None
        self.last_tick = None

    def set_link(self, state, now):
        if state not in ('WAITING', 'LINK_OK', 'LINK_DEGRADED', 'LINK_LOST'):
            return
        self.link = state
        if state == 'LINK_OK':
            self.seen_link_ok = True
            self.last_good_link_time = now
            self.state = 'NORMAL'
            self.path = ()
            self.frame = None
            self.last_good = None
            self.index = 0
            self.wait_started = None
        elif (state == 'LINK_LOST' and self.seen_link_ok and
              self.state == 'NORMAL'):
            self.state = 'LINK_LOST'

    def start_return(self, path, last_good, frame):
        if self.state != 'LINK_LOST' or not path or not frame:
            return False
        if not valid_point(last_good) or not all(valid_point(p) for p in path):
            return False
        # Recorder includes the frozen last-good pose as the final waypoint.
        if distance(path[-1], last_good) > 1e-6:
            return False
        self.path = tuple(tuple(p) for p in path)
        self.last_good = tuple(last_good)
        self.frame = frame
        self.index = 0
        self.state = 'RETURN'
        return True

    def set_odom(self, point, frame, stamp, received):
        if not valid_point(point) or not finite(stamp) or stamp <= 0:
            # Invalid input also invalidates any previous apparently fresh pose.
            self.odom_received = None
            return
        self.odom = tuple(point)
        self.odom_frame = frame
        self.odom_stamp = stamp
        self.odom_received = received

    def odom_fresh(self, now):
        if self.odom_received is None or self.odom_frame != self.frame:
            return False
        return (0 <= now - self.odom_received <= self.odom_timeout and
                0 <= now - self.odom_stamp <= self.odom_timeout)

    def step(self, now):
        if self.last_tick is not None and now < self.last_tick:
            self.odom_received = None
            if self.state == 'WAIT_LINK':
                self.wait_started = None
        self.last_tick = now
        if self.state == 'RETURN' and self.odom_fresh(now):
            if distance(self.odom, self.path[self.index]) <= self.arrival_radius:
                if self.index + 1 < len(self.path):
                    # Advance one point per publication, preserving route order.
                    self.index += 1
                elif distance(self.odom, self.last_good) <= self.arrival_radius:
                    self.state = 'WAIT_LINK'
                    self.wait_started = now
        elif self.state == 'WAIT_LINK':
            # Count only continuous time with a fresh pose at the endpoint.
            if (not self.odom_fresh(now) or
                    distance(self.odom, self.last_good) > self.arrival_radius):
                self.wait_started = None
            elif self.wait_started is None:
                self.wait_started = now
            elif now - self.wait_started >= self.wait_link_seconds:
                self.state = 'LAND'
        return self.state

    def goal(self):
        if self.state == 'RETURN':
            return self.path[self.index]
        if self.state == 'WAIT_LINK':
            return self.last_good
        return None
