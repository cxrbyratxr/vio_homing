#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Offline checks of actual bridge methods, without ROS, UART or numpy."""
from __future__ import print_function
import ast
import json
import numbers
import os
import threading
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

class Stub(object):
    def __init__(self, **values):
        self.__dict__.update(values)

class Capture(object):
    def __init__(self):
        self.calls = []
    def __getattr__(self, name):
        return lambda *args: self.calls.append((name, args))


def load_method(path, name):
    with open(os.path.join(ROOT, path)) as source:
        tree = ast.parse(source.read())
    method = next(node for cls in tree.body if isinstance(cls, ast.ClassDef)
                  for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == name)
    module = ast.Module(body=[method], type_ignores=[]) if hasattr(ast, 'Constant') else ast.Module(body=[method])
    namespace = dict(json=json, numbers=numbers, String=lambda **kw: Stub(**kw),
                     time=Stub(sleep=lambda seconds: None))
    exec(compile(ast.fix_missing_locations(module), path, 'exec'), namespace)
    return namespace[name]

set_origin = load_method('scripts/mavlink_imu_bridge.py', 'set_global_origin')
request_streams = load_method('src/vio_homing/flight_homing_adapter.py', 'request_flight_telemetry')

class Checks(unittest.TestCase):
    def bridge(self):
        mav = Capture()
        pub = Capture()
        epochs = []
        return Stub(control_lock=threading.RLock(), tx_lock=threading.RLock(),
                    mavlink_alive=lambda: True, px4_armed=False, landed_state=1,
                    known_global_origin=None, requested_global_origin=None,
                    new_navigation_epoch=lambda reason: epochs.append(reason) or True,
                    master=Stub(mav=mav, target_system=1, target_component=1),
                    origin_pub=pub, epochs=epochs)
    def message(self, **changes):
        values = dict(latitude=558102490, longitude=375008160, altitude=168000)
        values.update(changes)
        return Stub(data=json.dumps(values))
    def test_retry_only_reads_confirmation(self):
        b = self.bridge()
        set_origin(b, self.message())
        set_origin(b, self.message())
        self.assertEqual(len(b.epochs), 1)
        self.assertEqual([name for name, args in b.master.mav.calls],
                         ['set_gps_global_origin_send', 'command_long_send', 'command_long_send'])
    def test_armed_rejected(self):
        b = self.bridge(); b.px4_armed = True
        set_origin(b, self.message())
        self.assertFalse(b.master.mav.calls)
        self.assertIn('error', json.loads(b.origin_pub.calls[0][1][0].data))
    def test_conflicting_origin_rejected(self):
        b = self.bridge(); b.known_global_origin = (0, 0, 0)
        set_origin(b, self.message())
        self.assertFalse(b.master.mav.calls)
    def test_changed_request_rejected(self):
        b = self.bridge(); set_origin(b, self.message())
        set_origin(b, self.message(latitude=558100000))
        self.assertEqual(len(b.epochs), 1)
        self.assertIn('error', json.loads(b.origin_pub.calls[-1][1][0].data))
    def test_confirmed_is_idempotent(self):
        b = self.bridge(); b.known_global_origin = (558102490, 375008160, 168000)
        set_origin(b, self.message())
        self.assertFalse(b.master.mav.calls)
        self.assertFalse(b.epochs)
        self.assertEqual(json.loads(b.origin_pub.calls[0][1][0].data)['latitude'], 558102490)
    def test_map_streams_in_bench_and_flight(self):
        for mode in ('bench', 'flight'):
            b = self.bridge(); b.execution_mode = mode; b.auto_alignment = False
            request_streams(b)
            intervals = dict((args[4], args[5]) for name, args in b.master.mav.calls)
            self.assertEqual(intervals[33], 200000.)
            self.assertEqual(intervals[49], 1000000.)
            self.assertEqual(intervals[245], 500000.)
            self.assertEqual(331 in intervals, mode == 'flight')

if __name__ == '__main__':
    unittest.main(verbosity=2)
