#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Offline ROS-message integration checks. No master, UART or PX4 connection."""
from __future__ import print_function
import imp
import json
import os
import time
import unittest
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
class Clock(object):
    value = 100.
clock = Clock()
class Pub(object):
    def __init__(self,*args,**kwargs):self.messages=[]
    def publish(self,msg):self.messages.append(msg)
    def get_num_connections(self):return 0
PARAMS={'~homing_execution_mode':'flight','~enable_homing_commands':True,'~auto_frd_alignment':True}
rospy.get_param=lambda key,default=None:PARAMS.get(key,default)
rospy.get_time=lambda:clock.value
rospy.Time.now=staticmethod(lambda:rospy.Time.from_sec(clock.value))
rospy.Publisher=Pub
rospy.Subscriber=lambda *a,**k:None
rospy.Timer=lambda *a,**k:None
rospy.loginfo=rospy.logwarn=rospy.logerr=rospy.logwarn_throttle=lambda *a,**k:None
bridge_module=imp.load_source('offline_bridge',ROOT+'/scripts/mavlink_imu_bridge.py')
recorder_module=imp.load_source('offline_recorder',ROOT+'/scripts/trajectory_recorder.py')
manager_module=imp.load_source('offline_manager',ROOT+'/scripts/homing_manager.py')
adapter_module=imp.load_source('offline_adapter',ROOT+'/scripts/vins_odometry_adapter.py')
time.time=lambda:clock.value

class Mav(object):
    def __init__(self):self.calls=[]
    def __getattr__(self,name):
        return lambda *args,**kwargs:self.calls.append((name,args,kwargs))
class Master(object):
    target_system=1
    target_component=1
    def __init__(self):self.mav=Mav()

def pose(stamp=100.,x=0.):
    m=Odometry();m.header.stamp=rospy.Time.from_sec(stamp);m.header.seq=int(stamp*100)
    m.header.frame_id='local_frd';m.child_frame_id='body_frd';m.pose.pose.orientation.w=1
    m.pose.pose.position.x=x
    return m

class Integration(unittest.TestCase):
    def setUp(self):clock.value=100.
    def bridge(self):
        b=bridge_module.MavlinkImuBridge();b.master=Master();b.last_px4_heartbeat=100.
        b.px4_armed=False;b.landed_state=1;b.landed_received=100.;b.clock_offset_ns=0
        b.estimator_flags=431;b.estimator_received=100.;b.profile_requested=100.
        b.profile_values=dict(EKF2_EN=1,EKF2_EV_CTRL=7,EKF2_HGT_REF=0,EKF2_BARO_CTRL=1,
            EKF2_EV_NOISE_MD=1,COM_DLL_EXCEPT=4,COM_RCL_EXCEPT=6,NAV_DLL_ACT=3,
            COM_OBL_RC_ACT=4,COM_OF_LOSS_T=1,COM_DL_LOSS_T=10,COM_FAIL_ACT_T=5)
        b.profile_times={k:100. for k in b.profile_values}
        return b
    def test_stationary_px4_bit_is_not_navigation_failure(self):
        self.assertTrue(self.bridge().estimator_healthy(100.))
    def test_no_active_aiding_is_blocked(self):
        b=self.bridge();b.estimator_flags=175;self.assertFalse(b.estimator_healthy(100.))
    def test_failsafe_timing_budget(self):
        b=self.bridge();self.assertTrue(b.profile_ready());b.profile_values['COM_DL_LOSS_T']=5
        self.assertFalse(b.profile_ready())
    def test_epoch_clears_old_route_and_session(self):
        r=recorder_module.TrajectoryRecorder();r.link_state='LINK_OK';r.odom_callback(pose())
        self.assertEqual(len(r.points),1);r.on_epoch(String(data='new'))
        self.assertEqual(r.points,[]);self.assertIsNone(r.last_good_pose);self.assertFalse(r.return_ready)
        m=manager_module.HomingManager();old=m.session_pub.messages[-1].data
        m.on_epoch(String(data='new'));self.assertNotEqual(old,m.session_pub.messages[-1].data)
        self.assertEqual(m.logic.state,'NORMAL');self.assertIsNone(m.candidate_path)
    def test_no_ground_command_on_abort(self):
        b=self.bridge();b.transport.session='s';b.transport.state='ABORT';b.transport.state_time=100.
        b.tick_flight_io();self.assertEqual(b.master.mav.calls,[])
    def test_missing_heading_aborts_takeover_without_arm(self):
        b=self.bridge();b.px4_armed=True;b.landed_state=2;b.transport.alignment_valid=True
        b.transport.session='s';b.transport.state='RETURN';b.transport.active=True
        b.health_probe.values={k:True for k in ('xy_valid','z_valid','v_xy_valid','v_z_valid')}
        b.health_probe.values['heading_good_for_control']=False
        for t in (100.,103.1):
            clock.value=t;b.last_px4_heartbeat=t;b.landed_received=t;b.estimator_received=t
            b.px4_pose=(t,(0,0,-2),(0,0,0,1),0);b.health_probe.received=t;b.readiness_since=97.
            b.transport.state_time=b.transport.active_time=b.transport.target_time=b.transport.target_stamp=t
            b.transport.odom_time=b.transport.odom_stamp=t;b.transport.target=(1,0,-2)
            b.tick_flight_io()
        commands=[c[1][2] for c in b.master.mav.calls if c[0]=='command_long_send']
        self.assertEqual(commands,[21])
    def test_probe_fixed_read_only_and_releases_shell(self):
        b=self.bridge();b.tick_health_probe();call=b.master.mav.calls[-1]
        self.assertEqual(call[0],'serial_control_send');self.assertEqual(call[1][0],10)
        text=bytearray(call[1][5][:call[1][4]]).decode('ascii')
        self.assertEqual(text,'listener vehicle_local_position 1\n')
        reply='TOPIC: vehicle_local_position\n timestamp: 100000000\n'
        reply+='\n'.join(k+': True' for k in ('heading_good_for_control','xy_valid','z_valid','v_xy_valid','v_z_valid'))
        b.health_probe.feed(reply,100.01);clock.value=100.2;b.tick_health_probe()
        self.assertTrue(b.health_probe.ready(100.2));self.assertFalse(b.health_probe.pending)
        self.assertEqual(b.master.mav.calls[-1][1][:5],(10,0,0,0,0))
    def test_adapter_settle_then_latched_jump(self):
        a=adapter_module.Adapter()
        for i in range(40):
            clock.value=100.+i*.1;m=pose(clock.value+.03275171186939631)
            m.header.seq=i+1;m.header.frame_id=m.child_frame_id='world';a.callback(m)
        self.assertTrue(len(a.pub.messages)>0);count=len(a.pub.messages)
        clock.value+=.1;m=pose(clock.value+.03275171186939631,50.)
        m.header.seq=41;m.header.frame_id=m.child_frame_id='world';a.callback(m)
        self.assertIsNotNone(a.fault);self.assertEqual(len(a.pub.messages),count)
    def test_ground_ready_does_not_require_inflight_mag_alignment(self):
        b=self.bridge();b.transport.alignment_valid=True
        b.px4_pose=(100.,[0,0,0],[0,0,0,1],0)
        b.transport.odom_time=b.transport.odom_stamp=100.
        b.health_probe.values={k:True for k in ('xy_valid','z_valid','v_xy_valid','v_z_valid')}
        b.health_probe.values['heading_good_for_control']=False
        b.health_probe.received=100.;b.readiness_since=96.
        status=b.flight_status()
        self.assertTrue(status['ground_navigation_ready']);self.assertFalse(status['return_ready'])
    def test_ekf_yaw_reset_before_return_can_reregister_without_erasing_vio_route(self):
        b=self.bridge();b.px4_armed=True;b.landed_state=2;b.last_vins_health='ACTIVE'
        b.last_odometry_reset=0;b.transport.state='NORMAL';epoch=b.navigation_epoch
        class Message(object):
            frame_id=1;estimator_type=8;q=(1,0,0,0);x=y=z=0.;reset_counter=1
            def get_type(self):return 'ODOMETRY'
        msg=Message()
        for i in range(14):
            t=100.+i*.1;clock.value=t;b.estimator_received=t
            b.health_probe.received=t
            b.health_probe.values={k:True for k in ('heading_good_for_control','xy_valid','z_valid','v_xy_valid','v_z_valid')}
            b.local_poses.append((t,[0,0,0],[0,0,0,1]));msg.time_usec=int(t*1e6)
            b.flight_telemetry_received(msg)
        self.assertTrue(b.transport.alignment_valid);self.assertEqual(b.navigation_epoch,epoch)
    def test_inflight_return_reset_never_reregisters(self):
        b=self.bridge();b.px4_armed=True;b.landed_state=2;b.transport.state='RETURN'
        self.assertFalse(b.new_navigation_epoch('PX4_ESTIMATOR_RESET'))
        self.assertFalse(b.transport.alignment_valid);self.assertTrue(b.alignment.fault)
    def test_registered_frame_disagreement_latches(self):
        from vio_homing.px4_alignment import Alignment
        from vio_homing.homing_transport import FrdToNed
        a=Alignment();a.valid=True;a.transform=FrdToNed(0,[0,0,0]);a.reset_counter=0
        a.observe((100,[0,0,0],[0,0,0,1]),(100,[2,0,0],[0,0,0,1],0),False)
        self.assertTrue(a.valid)
        a.observe((100.6,[0,0,0],[0,0,0,1]),(100.6,[2,0,0],[0,0,0,1],0),False)
        self.assertFalse(a.valid);self.assertEqual(a.reason,'VIO_EKF_DISAGREEMENT')
    def test_fault_invalidates_alignment(self):
        b=self.bridge();b.transport.alignment_valid=True;b.vins_health_cb(String(data='FAULT: test'))
        self.assertFalse(b.transport.alignment_valid)

if __name__=='__main__':unittest.main(verbosity=2)
