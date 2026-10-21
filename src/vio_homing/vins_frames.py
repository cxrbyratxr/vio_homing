# -*- coding: utf-8 -*-
"""VINS world/IMU-FLU to local-FRD/body-FRD (arbitrary heading)."""
import math
import numpy as np

C = np.diag([1., -1., -1.])


def quaternion(values):
    q = np.asarray(values, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all():
        raise ValueError('Invalid quaternion')
    norm = np.linalg.norm(q)
    if abs(norm - 1.) > 0.05:
        raise ValueError('Quaternion is not unit length')
    return q / norm


def rotation(q):
    x, y, z, w = quaternion(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def convert(position, orientation, world_velocity):
    p = np.asarray(position, dtype=float)
    v = np.asarray(world_velocity, dtype=float)
    q = quaternion(orientation)
    if p.shape != (3,) or v.shape != (3,) or not np.isfinite([p, v]).all():
        raise ValueError('Invalid position or velocity')
    # R_out = C R_in C. Quaternion layout is ROS xyzw.
    return C.dot(p), q * np.array([1., -1., -1., 1.]), C.dot(rotation(q).T.dot(v))


def angular_velocity(previous, current, dt):
    if dt <= 0:
        raise ValueError('Nonpositive dt')
    a = quaternion(previous)
    b = quaternion(current)
    # conjugate(previous) * current, shortest rotation in body coordinates.
    av, aw = -a[:3], a[3]
    bv, bw = b[:3], b[3]
    v = aw*bv + bw*av + np.cross(av, bv)
    w = aw*bw - np.dot(av, bv)
    if w < 0:
        v, w = -v, -w
    n = np.linalg.norm(v)
    if n < 1e-12:
        return np.zeros(3)
    return C.dot(v * (2*math.atan2(n, w)/(n*dt)))


def pose_covariance(values):
    covariance = np.asarray(values, dtype=float).reshape(6, 6)
    transform = np.zeros((6, 6))
    transform[:3, :3] = C
    transform[3:, 3:] = C
    return transform.dot(covariance).dot(transform.T).ravel().tolist()
