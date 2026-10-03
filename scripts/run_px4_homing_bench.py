#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Run isolated UART bench, always resume the previously running bridge."""
from __future__ import print_function
import os
import json
import re
import signal
import subprocess
import sys
import time

rows = subprocess.check_output(['ps', '-eo', 'pid,args']).decode().splitlines()
pids = [int(row.split(None, 1)[0]) for row in rows
        if re.search(r' python /home/jetson/catkin_ws/src/vio_homing/scripts/mavlink_imu_bridge.py ', row)]
if len(pids) != 1:
    sys.exit('Expected one running companion UART bridge; found %r' % pids)
process = None
started = time.time()
log = open('/tmp/vio_homing_px4_bench_launch.log', 'w')
env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:11331')
env.pop('ROS_IP', None)
env.pop('ROS_HOSTNAME', None)
try:
    os.kill(pids[0], signal.SIGSTOP)
    process = subprocess.Popen(['roslaunch', '-p', '11331', 'vio_homing',
                                'px4_homing_bench.launch', 'serial_port:=/dev/ttyTHS1'],
                               env=env, stdout=log, stderr=subprocess.STDOUT,
                               preexec_fn=os.setsid)
    deadline = time.time()+420
    while process.poll() is None and time.time() < deadline:
        time.sleep(0.5)
    if process.poll() is None:
        raise RuntimeError('Hardware bench timeout')
    result_path = '/tmp/vio_homing_px4_bench/latest.json'
    if os.path.getmtime(result_path) < started:
        raise RuntimeError('No result from this bench run')
    result = json.load(open(result_path))
    if not result.get('passed'):
        raise RuntimeError('Bench failed: '+str(result.get('error')))
    print('PASS: hardware bench; WAIT_LINK %.3f s, LAND %s' %
          (result['wait_link_seconds'], result['land_ack']))
finally:
    if process is not None and process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        for unused in range(20):
            if process.poll() is not None:
                break
            time.sleep(0.5)
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait()
    os.kill(pids[0], signal.SIGCONT)
    log.close()
    print('Companion UART bridge resumed:', pids[0])
