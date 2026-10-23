#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Set an explicitly synthetic geographic origin on a disarmed bench."""
from __future__ import print_function
import json
import threading
import time
import rospy
from std_msgs.msg import String


def main():
    rospy.init_node('set_bench_origin', anonymous=True)
    expected = dict(latitude=558102490, longitude=375008160, altitude=168000)
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
    print('BENCH ONLY: reference near MAI building 3, 55.810249 / 37.500816, terrain 168 m MSL.')
    print('This does not establish true vehicle location, yaw alignment or flight readiness.')
    pub.publish(String(data=json.dumps(expected)))
    if not event.wait(8):
        raise RuntimeError('No matching GPS_GLOBAL_ORIGIN received from PX4; acceptance NOT confirmed.')
    if result[0].get('error'):
        raise RuntimeError(result[0]['error'])
    print('PX4 GPS_GLOBAL_ORIGIN confirmed: '+json.dumps(result[0],sort_keys=True))
    print('Now check vehicle_local_position / vehicle_global_position and commander check in QGC.')


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, rospy.ROSException) as error:
        raise SystemExit(str(error))
