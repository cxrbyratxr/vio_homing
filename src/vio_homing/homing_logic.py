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
    return math.sqrt(sum((x - y)**2 for x, y in zip(a, b)))


class HomingLogic(object):

    def __init__(
        self, arrival_radius=0.4, wait_link_seconds=60.0, odom_timeout=1.0, arrival_dwell=0.5, arrival_speed=0.5,
        progress_timeout=15.0, progress_distance=0.15, waypoint_timeout=60.0
    ):
        values = (arrival_radius, wait_link_seconds, odom_timeout, arrival_dwell,
                  arrival_speed, progress_timeout, progress_distance, waypoint_timeout)
        if (
            not all(finite(v) for v in values) or arrival_radius <= 0
            or odom_timeout <= 0 or wait_link_seconds < 60.0
            or min(arrival_dwell, arrival_speed, progress_timeout, progress_distance, waypoint_timeout) <= 0
        ):
            raise ValueError(
                'radius/timeout must be positive; wait must be >= 60 s'
            )
        self.arrival_radius = arrival_radius
        self.wait_link_seconds = wait_link_seconds
        self.odom_timeout = odom_timeout
        self.arrival_dwell, self.arrival_speed = arrival_dwell, arrival_speed
        self.progress_timeout, self.progress_distance = progress_timeout, progress_distance
        self.waypoint_timeout = waypoint_timeout
        self.reason = 'NORMAL'
        self.arrival_since = None
        self.progress_since = None
        self.best_distance = None
        self.waypoint_since = None
        self.loss_since = None
        self.speed = None
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
            self.arrival_since = self.progress_since = self.waypoint_since = None
            self.best_distance = None
            self.loss_since = None
            self.reason = 'LINK_RECOVERED'
        elif (
            state == 'LINK_LOST' and self.seen_link_ok
            and self.state == 'NORMAL'
        ):
            self.state = 'LINK_LOST'
            self.loss_since = now

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
        self.reason = 'FOLLOWING_PATH'
        self.arrival_since = self.progress_since = self.waypoint_since = None
        self.best_distance = None
        return True

    def set_odom(self, point, frame, stamp, received, speed=None):
        if not valid_point(point) or not finite(stamp) or stamp <= 0:
            # Invalid input also invalidates any previous apparently fresh
            # pose.
            self.odom_received = None
            return
        if speed is None and self.odom is not None and self.odom_stamp is not None and stamp > self.odom_stamp:
            speed = distance(point, self.odom)/(stamp-self.odom_stamp)
        self.speed = speed if speed is not None and finite(speed) and speed >= 0 else None
        self.odom = tuple(point)
        self.odom_frame = frame
        self.odom_stamp = stamp
        self.odom_received = received

    def odom_fresh(self, now):
        if self.odom_received is None or self.odom_frame != self.frame:
            return False
        return (
            0 <= now - self.odom_received <= self.odom_timeout
            and 0 <= now - self.odom_stamp <= self.odom_timeout
        )

    def abort(self, reason):
        self.state, self.reason = 'ABORT', reason
        self.wait_started = self.arrival_since = None

    def step(self, now):
        if self.last_tick is not None and now < self.last_tick:
            self.odom_received = None
            if self.state in ('RETURN', 'WAIT_LINK'):
                self.abort('CLOCK_RESET')
        self.last_tick = now
        if self.state == 'LINK_LOST' and self.loss_since is not None and now-self.loss_since > 3.:
            self.abort('NO_USABLE_RETURN_PATH')
        if self.state not in ('RETURN', 'WAIT_LINK'):
            return self.state
        if not self.odom_fresh(now):
            self.abort('ODOMETRY_LOST')
            return self.state
        if self.state == 'RETURN':
            remaining = distance(self.odom, self.path[self.index])
            if self.progress_since is None:
                self.progress_since = self.waypoint_since = now
                self.best_distance = remaining
            if remaining <= self.best_distance-self.progress_distance:
                self.best_distance, self.progress_since = remaining, now
            arrived = remaining <= self.arrival_radius and self.speed is not None and self.speed <= self.arrival_speed
            self.arrival_since = (now if self.arrival_since is None else self.arrival_since) if arrived else None
            if self.arrival_since is not None and now-self.arrival_since >= self.arrival_dwell:
                self.arrival_since = self.progress_since = self.waypoint_since = None
                self.best_distance = None
                if self.index+1 < len(self.path):
                    self.index += 1
                elif distance(self.odom, self.last_good) <= self.arrival_radius:
                    self.state, self.reason = 'WAIT_LINK', 'AT_LAST_GOOD_POINT'
                    self.wait_started = now
            elif now-self.progress_since >= self.progress_timeout or now-self.waypoint_since >= self.waypoint_timeout:
                self.abort('RETURN_NO_PROGRESS')
        elif self.state == 'WAIT_LINK':
            at_rest = (distance(self.odom, self.last_good) <= self.arrival_radius
                       and self.speed is not None and self.speed <= self.arrival_speed)
            if not at_rest:
                self.wait_started = None
                if self.progress_since is None:
                    self.progress_since = now
                elif now-self.progress_since >= self.progress_timeout:
                    self.abort('ENDPOINT_NOT_HELD')
            else:
                self.progress_since = None
                if self.wait_started is None:
                    self.wait_started = now
                elif now-self.wait_started >= self.wait_link_seconds:
                    self.state, self.reason = 'LAND', 'WAIT_LINK_FINISHED'
        return self.state

    def goal(self):
        if self.state == 'RETURN':
            return self.path[self.index]
        if self.state == 'WAIT_LINK':
            return self.last_good
        return None
