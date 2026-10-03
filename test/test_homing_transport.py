import unittest
from vio_homing.homing_transport import FrdToNed, HomingTransport


class TransportTests(unittest.TestCase):
    def ready(self):
        t = HomingTransport(FrdToNed(90, (10, -20, 2)), True, True)
        t.set_session('manager-one')
        t.set_state('RETURN', 100)
        t.set_active(True, 100)
        t.set_target((4, 1, -3), 'local_frd', 100, 100)
        t.set_odom((4, 0, 0), 'local_frd', 100, 100)
        return t

    def test_rotation_translation_and_down_axis(self):
        f = FrdToNed(90, (10, -20, 2))
        self.assertEqual(f.convert((0, 0, 0)), (10, -20, 2))
        p = f.convert((4, 1, -3))
        for a, b in zip(p, (9, -16, -1)):
            self.assertAlmostEqual(a, b)
        self.assertEqual(FrdToNed(0, (0, 0, 0)).convert((1, 2, 3)), (1, 2, 3))

    def test_send_only_when_explicitly_enabled_and_aligned(self):
        for attribute in ('enabled', 'alignment_valid'):
            t = self.ready()
            setattr(t, attribute, False)
            self.assertIsNone(t.next_setpoint(100.1, True, True))

    def test_missing_stale_wrong_frame_and_nan_odometry(self):
        t = self.ready()
        for point, frame, stamp, received in [((4, 0, 0), 'local_frd', 90, 100),
                                              ((4, 0, 0), 'map', 100, 100),
                                              ((float('nan'), 0, 0), 'local_frd', 100, 100)]:
            t.set_odom(point, frame, stamp, received)
            self.assertIsNone(t.next_setpoint(100.1, True, True))
        t.odom_time = None
        self.assertIsNone(t.next_setpoint(100.1, True, True))

    def test_stale_manager_or_target_stops_stream(self):
        self.assertIsNone(self.ready().next_setpoint(100.6, True, True))
        t = self.ready()
        t.set_target((1, 0, 0), 'local_frd', 90, 100)
        self.assertIsNone(t.next_setpoint(100.1, True, True))

    def test_no_timesync_or_mavlink_no_setpoint(self):
        self.assertIsNone(self.ready().next_setpoint(100.1, False, True))
        self.assertIsNone(self.ready().next_setpoint(100.1, True, False))

    def test_recovery_stops_setpoint_and_cancels_land(self):
        t = self.ready()
        t.mark_setpoint_sent()
        t.set_state('NORMAL', 100.1)
        t.set_land(True, 100.1)
        self.assertIsNone(t.next_setpoint(100.1, True, True))
        self.assertFalse(t.should_land(100.1, True))

    def test_land_once_per_episode(self):
        t = self.ready()
        t.mark_setpoint_sent()
        t.set_state('LAND', 100.1)
        t.set_land(True, 100.1)
        self.assertTrue(t.should_land(100.1, True))
        t.mark_land_sent()
        for now in (100.2, 100.3):
            t.set_land(True, now)
            t.set_state('LAND', now)
            self.assertFalse(t.should_land(now, True))

    def test_latched_land_after_bridge_start_is_not_replayed(self):
        t = self.ready()
        t.set_state('LAND', 100.1)
        t.set_land(True, 100.1)
        self.assertFalse(t.should_land(100.1, True))

    def test_link_loss_discards_unsent_land(self):
        t = self.ready()
        t.mark_setpoint_sent()
        t.link_lost()
        t.set_odom((0, 0, 0), 'local_frd', 100.1, 100.1)
        t.set_state('LAND', 100.1)
        t.set_land(True, 100.1)
        self.assertFalse(t.should_land(100.1, True))

    def test_manager_restart_clears_old_episode(self):
        t = self.ready()
        t.mark_setpoint_sent()
        t.set_session('manager-two')
        self.assertIsNone(t.next_setpoint(100.1, True, True))
        t.set_state('LAND', 100.1)
        t.set_land(True, 100.1)
        self.assertFalse(t.should_land(100.1, True))


if __name__ == '__main__':
    unittest.main()
