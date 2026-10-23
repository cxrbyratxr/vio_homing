# -*- coding: utf-8 -*-
"""Physical plausibility checks, not a proof of VIO accuracy."""
import math


def norm(v):
    return math.sqrt(sum(x*x for x in v))


def finite_vector(v):
    return len(v) == 3 and all(not math.isnan(x) and not math.isinf(x) for x in v)


class VioGuard(object):
    def __init__(self, max_speed=5., max_acceleration=12., max_step=1., settle=3.):
        self.max_speed = max_speed
        self.max_acceleration = max_acceleration
        self.max_step = max_step
        self.settle = settle
        if any(not (0 < x < float('inf')) for x in
               (max_speed, max_acceleration, max_step, settle)):
            raise ValueError('Invalid VIO safety limit')
        self.previous = None
        self.stable_since = None
        self.active = False
        self.reason = 'WAITING_FOR_STABLE_VINS'

    def observe(self, p, v, stamp):
        if not finite_vector(p) or not finite_vector(v):
            raise ValueError('Nonfinite VIO data')
        reason = None
        speed = norm(v)
        if speed > self.max_speed:
            reason = 'VIO reported speed %.2f > %.2f m/s' % (speed, self.max_speed)
        if self.previous is not None:
            old_p, old_v, old_t = self.previous
            dt = stamp-old_t
            if dt <= 0:
                raise ValueError('VIO clock discontinuity')
            step = norm([p[i]-old_p[i] for i in range(3)])
            acceleration = norm([v[i]-old_v[i] for i in range(3)])/dt
            if step > self.max_step or step > self.max_speed*dt+.15:
                reason = 'VIO position jump %.3f m in %.3f s' % (step, dt)
            elif acceleration > self.max_acceleration:
                reason = 'VIO velocity discontinuity %.2f m/s2' % acceleration
        self.previous = (tuple(p), tuple(v), stamp)
        if reason:
            self.stable_since = None
            self.reason = reason
            if self.active:
                raise ValueError(reason)
            return False
        if self.stable_since is None:
            self.stable_since = stamp
        self.active = self.active or stamp-self.stable_since >= self.settle
        self.reason = 'ACTIVE' if self.active else 'WAITING_FOR_STABLE_VINS'
        return self.active
