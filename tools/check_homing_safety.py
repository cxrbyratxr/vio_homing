from __future__ import print_function
import unittest
from vio_homing.homing_logic import HomingLogic
from vio_homing.flight_control import FlightControl, OFFBOARD, HOLD, LAND
from vio_homing.vio_safety import VioGuard
from vio_homing.px4_health import Px4Health

class Safety(unittest.TestCase):
    def new_route(self):
        g=HomingLogic();g.set_link('LINK_OK',0);g.set_link('LINK_LOST',4)
        self.assertTrue(g.start_return([(0,0,0)],(0,0,0),'local_frd'))
        return g
    def tick(self,g,t,p=(0,0,0),speed=0):
        g.set_odom(p,'local_frd',t,t,speed);return g.step(t)
    def fc(self,f,t,mode=OFFBOARD,normal=False,land=False,healthy=True,armed=True,air=True):
        return f.step(t,'one',armed,air,True,mode,not normal and not land,normal,land,healthy,(0,0,-2),(1,0,-2))
    def active(self):
        f=FlightControl();self.fc(f,0,HOLD);self.fc(f,1.21,HOLD);self.fc(f,1.3);self.assertEqual(f.phase,'ACTIVE');return f
    def test_dwell_and_speed(self):
        g=self.new_route()
        for i in range(10):self.assertEqual(self.tick(g,5+i*.1,speed=2),'RETURN')
        self.assertEqual(self.tick(g,6),'RETURN');self.assertEqual(self.tick(g,6.49),'RETURN');self.assertEqual(self.tick(g,6.51),'WAIT_LINK')
    def test_wait_at_least_60_in_state_clock(self):
        g=self.new_route();self.tick(g,5);self.tick(g,5.51)
        for i in range(1,600):self.assertEqual(self.tick(g,5.51+i*.1),'WAIT_LINK')
        self.assertEqual(self.tick(g,65.52),'LAND')
    def test_wait_recovery_cancels_old_timer(self):
        g=self.new_route();self.tick(g,5);self.tick(g,5.51);g.set_link('LINK_OK',10)
        self.assertEqual(self.tick(g,100),'NORMAL')
    def test_return_recovery(self):
        g=self.new_route();g.set_link('LINK_OK',5);self.assertEqual(self.tick(g,6),'NORMAL')
    def test_wait_movement_resets_timer(self):
        g=self.new_route();self.tick(g,5);self.tick(g,5.51);self.tick(g,50,p=(1,0,0))
        self.tick(g,51);self.assertEqual(self.tick(g,66),'WAIT_LINK')
    def test_stuck(self):
        g=self.new_route();self.tick(g,5,p=(4,0,0));self.assertEqual(self.tick(g,20.01,p=(4,0,0)),'ABORT');self.assertEqual(g.reason,'RETURN_NO_PROGRESS')
    def test_missing_and_stale_odom(self):
        g=self.new_route();self.assertEqual(g.step(5),'ABORT')
        g=self.new_route();self.tick(g,5);self.assertEqual(g.step(7),'ABORT')
    def test_empty_path(self):
        g=HomingLogic();g.set_link('LINK_OK',1);g.set_link('LINK_LOST',5)
        self.assertFalse(g.start_return([], (0,0,0),'local_frd'));self.assertEqual(g.step(8.1),'ABORT')
    def test_never_connected(self):
        g=HomingLogic();g.set_link('LINK_LOST',5);self.assertEqual(g.step(100),'NORMAL')
    def test_clock_backwards(self):
        g=self.new_route();self.tick(g,5);self.assertEqual(self.tick(g,4),'ABORT')
    def test_full_control_nominal_land_once(self):
        f=self.active();self.assertEqual(self.fc(f,65,land=True)[1],'LAND')
        self.assertIsNone(self.fc(f,66,land=True)[1]);self.fc(f,67,mode=LAND,land=True);self.assertEqual(f.phase,'LANDING')
    def test_disarmed_and_ground_no_commands(self):
        for armed,air in [(False,False),(True,False),(False,True)]:
            f=FlightControl();self.assertEqual(self.fc(f,1,land=True,armed=armed,air=air),(None,None))
    def test_no_path_abort_land_from_idle(self):
        f=FlightControl();self.assertEqual(self.fc(f,1,HOLD,land=True,healthy=False)[1],'LAND');self.assertIsNone(self.fc(f,2,HOLD,land=True)[1])
    def test_invalid_nav_active(self):
        f=self.active();self.assertEqual(self.fc(f,2,healthy=False)[1],'LAND')
    def test_offboard_timeout(self):
        f=FlightControl();self.fc(f,0,HOLD);self.fc(f,1.3,HOLD);self.assertEqual(self.fc(f,4.4,HOLD)[1],'LAND')
    def test_manual_mode_override(self):
        f=self.active();self.assertIsNone(self.fc(f,2,HOLD,land=True)[1]);self.assertEqual(f.reason,'PX4_MODE_CHANGED')
    def test_link_recovered_hold(self):
        f=self.active();self.assertEqual(self.fc(f,2,normal=True)[1],'HOLD');self.assertEqual(self.fc(f,3,HOLD,normal=True),(None,None))
    def test_prestream_cancel(self):
        f=FlightControl();self.fc(f,0,HOLD);self.assertEqual(self.fc(f,.2,HOLD,normal=True),(None,None));self.assertEqual(f.phase,'IDLE')
    def test_mavlink_loss(self):
        f=self.active();self.assertEqual(f.step(2,'one',True,True,False,OFFBOARD,True,False,False,True,(0,0,0),(1,0,0)),(None,None))
    def test_manager_restart(self):
        f=self.active();self.assertEqual(f.step(2,'two',True,True,True,OFFBOARD,True,False,False,True,(0,0,0),(1,0,0))[1],'LAND')
    def test_guard_settle(self):
        g=VioGuard();self.assertFalse(g.observe((0,0,0),(0,0,0),1));self.assertFalse(g.observe((0,0,0),(0,0,0),3.9));self.assertTrue(g.observe((0,0,0),(0,0,0),4))
    def test_guard_startup_jump_not_published(self):
        g=VioGuard();g.observe((0,0,0),(0,0,0),1);self.assertFalse(g.observe((100,0,0),(0,0,0),1.1));self.assertFalse(g.active)
    def test_guard_active_jump_and_speed(self):
        for p,v in [((2,0,0),(0,0,0)),((0,0,0),(55,0,0)),((0,0,0),(2,0,0))]:
            g=VioGuard();g.observe((0,0,0),(0,0,0),1);g.observe((0,0,0),(0,0,0),4)
            with self.assertRaises(ValueError):g.observe(p,v,4.1)
    def test_guard_nonfinite(self):
        with self.assertRaises(ValueError):VioGuard().observe((float('nan'),0,0),(0,0,0),1)
    def text(self,heading='True'):
        return 'TOPIC: vehicle_local_position\n timestamp: 1000000\n'+'\n'.join(k+': '+(heading if k=='heading_good_for_control' else 'True') for k in ('xy_valid','z_valid','v_xy_valid','v_z_valid','heading_good_for_control'))
    def test_health_ground_vs_air(self):
        p=Px4Health();p.begin(1);p.feed(self.text('False'),1);p.finish(1,lambda t:t/1e6,1)
        self.assertTrue(p.ready(1,False));self.assertFalse(p.ready(1));self.assertFalse(p.ready(6,False))
    def test_health_air_and_malformed(self):
        p=Px4Health();p.begin(1);p.feed(self.text(),1);p.finish(1,lambda t:t/1e6,1);self.assertTrue(p.ready(1))
        p.begin(2);p.feed('garbage',2);p.finish(2,lambda t:t/1e6,2);self.assertFalse(p.ready(2))
    def test_health_old_timestamp(self):
        p=Px4Health();p.begin(10);p.feed(self.text(),10);p.finish(10,lambda t:t/1e6,10);self.assertFalse(p.ready(10))

if __name__=='__main__':unittest.main(verbosity=2)
