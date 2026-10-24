#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Set a configured geographic EKF origin while PX4 is disarmed on ground."""
from __future__ import print_function
import json
import math
import threading
import time
import rospy
from std_msgs.msg import String


def main():
    rospy.init_node('set_bench_origin', anonymous=True)
    latitude = float(rospy.get_param('~latitude_deg', 56.098828))
    longitude = float(rospy.get_param('~longitude_deg', 35.869226))
    altitude = float(rospy.get_param('~altitude_msl_m', 153.0))
    if (not all(not math.isnan(v) and not math.isinf(v) for v in (latitude, longitude, altitude))
            or not -90 <= latitude <= 90 or not -180 <= longitude <= 180
            or not -1000 <= altitude <= 10000):
        raise RuntimeError('Invalid configured origin (degrees / metres MSL)')
    expected = dict(latitude=int(round(latitude*1e7)), longitude=int(round(longitude*1e7)),
                    altitude=int(round(altitude*1000)))
    automatic = bool(rospy.get_param('~automatic', False))
    if automatic:
        # Allow incoming origin/landed telemetry to arrive before the first request.
        time.sleep(5)
    event = threading.Event()
    result = [None]
    def response(msg):
        try:
            value = json.loads(msg.data)
        except ValueError:
            return
        if all(value.get(k) == v for k, v in expected.items()):
            result[0] = value
            event.set()
        elif value.get('error'):
            result[0] = value
            event.set()
    sub = rospy.Subscriber('/vio_homing/global_origin_response', String, response, queue_size=10)
    pub = rospy.Publisher('/vio_homing/global_origin_request', String, queue_size=1)
    deadline = time.time()+5
    while pub.get_num_connections() == 0 and time.time() < deadline and not rospy.is_shutdown():
        time.sleep(.1)
    if not pub.get_num_connections():
        raise RuntimeError('Updated MAVLink bridge is not running.')
    print('Configured map origin: %.7f / %.7f, %.3f m MSL.' % (latitude, longitude, altitude))
    if rospy.get_param('~synthetic_reference', False):
        print('BENCH ONLY: synthetic reference; replace it for the actual launch location.')
    print('This does not establish true vehicle location, yaw alignment or flight readiness.')
    deadline = time.time() + (120 if automatic else 15)
    while not rospy.is_shutdown() and time.time() < deadline:
        event.clear()
        result[0] = None
        pub.publish(String(data=json.dumps(expected)))
        if event.wait(3):
            if not result[0].get('error'):
                break
            if not automatic or 'requires live disarmed' not in result[0]['error']:
                raise RuntimeError(result[0]['error'])
            time.sleep(1)  # Avoid flooding the bridge while waiting for ground state.
        if automatic:
            rospy.logwarn_throttle(10, 'Waiting for disarmed PX4 / origin confirmation; map position is not confirmed')
    else:
        raise RuntimeError('No matching GPS_GLOBAL_ORIGIN received from PX4; acceptance NOT confirmed.')
    if rospy.is_shutdown():
        return
    if result[0].get('error'):
        raise RuntimeError(result[0]['error'])
    print('PX4 GPS_GLOBAL_ORIGIN confirmed: '+json.dumps(result[0],sort_keys=True))
    print('Now check vehicle_local_position / vehicle_global_position and commander check in QGC.')


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, rospy.ROSException) as error:
        raise SystemExit(str(error))
