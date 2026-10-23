# -*- coding: utf-8 -*-
"""Estimate one fixed yaw/origin from timestamp-matched VIO and EKF poses."""
import math
import numpy as np
from vio_homing.vins_frames import rotation
from vio_homing.homing_transport import FrdToNed


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Alignment(object):
    def __init__(self):
        self.valid = False
        self.transform = None
        self.samples = []
        self.reset_counter = None
        self.reason = 'WAITING_FOR_EKF_AND_VINS'
        self.fault = False
        self.last_stamp = None
        self.bad_since = None

    def invalidate(self, reason):
        self.valid = False
        self.samples = []
        self.reason = reason
        self.fault = True

    def observe(self, local, px4, disarmed):
        # Entries: stamp, xyz, xyzw. PX4 additionally carries reset_counter.
        if self.fault:
            return
        if self.reset_counter is not None and px4[3] != self.reset_counter:
            self.invalidate('PX4_ESTIMATOR_RESET')
            return
        self.reset_counter = px4[3]
        if abs(local[0]-px4[0]) > .12:
            return
        if self.valid:
            residual = np.linalg.norm(np.array(self.transform.convert(local[1]))-np.array(px4[1]))
            difference = rotation(px4[2]).dot(rotation(local[2]).T)
            if residual > 1. or difference[2, 2] < math.cos(math.radians(15)):
                if self.bad_since is None:
                    self.bad_since = local[0]
                elif local[0]-self.bad_since >= .5:
                    self.invalidate('VIO_EKF_DISAGREEMENT')
            else:
                self.bad_since = None
            return
        if not disarmed:
            return
        if self.last_stamp is not None and local[0] <= self.last_stamp:
            return
        self.last_stamp = local[0]
        difference = rotation(px4[2]).dot(rotation(local[2]).T)
        if difference[2, 2] < math.cos(math.radians(10)):
            self.samples = []
            self.reason = 'GRAVITY_AXES_DISAGREE'
            return
        yaw = math.atan2(difference[1, 0], difference[0, 0])
        rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                       [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        origin = np.array(px4[1])-rz.dot(local[1])
        if not np.isfinite(origin).all():
            return
        if self.samples and (abs(wrap(yaw-self.samples[0][1])) > math.radians(3)
                             or np.linalg.norm(origin-self.samples[0][2]) > .2):
            self.samples = []
        self.samples.append((local[0], yaw, origin))
        self.reason = 'COLLECTING_ALIGNMENT'
        if len(self.samples) >= 10 and self.samples[-1][0]-self.samples[0][0] >= 1.:
            yaw = math.atan2(sum(math.sin(s[1]) for s in self.samples),
                             sum(math.cos(s[1]) for s in self.samples))
            origin = np.mean([s[2] for s in self.samples], axis=0)
            self.transform = FrdToNed(math.degrees(yaw), origin.tolist())
            self.valid = True
            self.reason = 'ALIGNED_DISARMED'
