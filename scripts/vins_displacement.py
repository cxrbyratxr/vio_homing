#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Read-only terminal display of raw VINS displacement; ROS Melodic/Python 2."""
from __future__ import print_function
import argparse
import math
import select
import sys
import threading
import time

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from vio_homing.vins_frames import rotation


class Display(object):
    def __init__(self, args):
        self.args = args
        self.lock = threading.RLock()
        self.sample = None
        self.origin = None
        self.yaw = 0.
        self.previous = None
        self.warning = ''
        self.adapter = 'no status received'
        self.sub = rospy.Subscriber(args.topic, Odometry, self.callback, queue_size=1)
        self.health = rospy.Subscriber('/vio_homing/vins_adapter_status', String, self.status, queue_size=1)

    def status(self, msg):
        with self.lock:
            self.adapter = msg.data

    def callback(self, msg):
        with self.lock:
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            v = msg.twist.twist.linear
            xyz = np.array([p.x, p.y, p.z])
            stamp = msg.header.stamp.to_sec()
            try:
                r = rotation([q.x, q.y, q.z, q.w])
                if not np.isfinite(xyz).all() or not np.isfinite(stamp) or stamp <= 0:
                    raise ValueError('invalid position/timestamp')
                if msg.header.frame_id != 'world' or msg.child_frame_id != 'world':
                    raise ValueError('expected raw VINS world/world frames')
            except (ValueError, TypeError) as error:
                self.warning = 'INVALID DATA: ' + str(error)
                return
            if self.previous is not None:
                old_stamp, old_seq, old_xyz = self.previous
                if stamp <= old_stamp or msg.header.seq <= old_seq:
                    self.warning = 'STREAM RESET: press r + Enter to set a new origin'
                elif np.linalg.norm(xyz-old_xyz) > 1.:
                    self.warning = 'POSITION JUMP > 1 m: raw estimate is unreliable'
            self.previous = (stamp, msg.header.seq, xyz.copy())
            age = rospy.get_time()-stamp
            if self.origin is None and -.1 <= age <= self.args.stale:
                self.origin = xyz.copy()
                self.yaw = math.atan2(r[1, 0], r[0, 0])
            speed = np.linalg.norm([v.x, v.y, v.z])
            self.sample = (stamp, xyz, speed, time.time())

    def reset(self):
        with self.lock:
            self.origin = None
            self.previous = None
            self.warning = ''

    def render(self):
        with self.lock:
            lines = ['VINS RAW DISPLACEMENT (estimate, not ground truth)',
                     'Axes: forward/right/up relative to heading at origin; not North/East.',
                     'r + Enter: reset origin | q + Enter / Ctrl+C: exit',
                     'Adapter: ' + self.adapter]
            if self.warning:
                lines.append('WARNING: ' + self.warning)
            if self.sample is None:
                return '\n'.join(lines + ['Waiting for ' + self.args.topic])
            stamp, xyz, speed, receipt = self.sample
            age = rospy.get_time()-stamp
            fresh = -.1 <= age <= self.args.stale and time.time()-receipt <= self.args.stale
            lines.append('Data: %s | age: %.3f s | receipt age: %.3f s' %
                         ('FRESH' if fresh else 'STALE / INVALID TIME', age, time.time()-receipt))
            lines.append('World XYZ: %+10.3f %+10.3f %+10.3f m' % tuple(xyz))
            if self.origin is None:
                return '\n'.join(lines + ['Waiting for a fresh pose to set origin.'])
            d = xyz-self.origin
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            forward, right, up = c*d[0]+s*d[1], s*d[0]-c*d[1], d[2]
            lines += ['Forward(+)/back(-): %+10.3f m' % forward,
                      'Right(+)/left(-):   %+10.3f m' % right,
                      'Up(+)/down(-):     %+10.3f m' % up,
                      'Distance from origin: %.3f m | VINS speed: %.3f m/s' % (np.linalg.norm(d), speed),
                      'Distance is straight-line displacement, not total travelled path.',
                      'Adapter FAULT or stale data: do not trust these coordinates.']
            return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/vins_estimator/odometry')
    parser.add_argument('--rate', type=float, default=5.)
    parser.add_argument('--stale', type=float, default=.5)
    args = parser.parse_args(rospy.myargv()[1:])
    if not np.isfinite([args.rate, args.stale]).all() or not 0 < args.rate <= 20 or args.stale <= 0:
        parser.error('rate must be (0,20], stale must be positive')
    rospy.init_node('vins_displacement', anonymous=True, disable_signals=True)
    display = Display(args)
    interactive = sys.stdout.isatty()
    keyboard = sys.stdin.isatty()
    try:
        while not rospy.is_shutdown():
            if interactive:
                sys.stdout.write('\033[2J\033[H')
            print(display.render())
            sys.stdout.flush()
            if keyboard and select.select([sys.stdin], [], [], 0)[0]:
                command = sys.stdin.readline()
                if not command or command.strip().lower() == 'q':
                    break
                if command.strip().lower() == 'r':
                    display.reset()
            time.sleep(1./args.rate)
    except KeyboardInterrupt:
        pass
    finally:
        rospy.signal_shutdown('display closed')


if __name__ == '__main__':
    main()
