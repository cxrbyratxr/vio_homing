import unittest

from vio_homing.homing_logic import HomingLogic


class HomingLogicTest(unittest.TestCase):
    def armed(self):
        h = HomingLogic()
        h.set_link('LINK_OK', 100.0)
        h.set_link('LINK_DEGRADED', 101.0)
        h.set_link('LINK_LOST', 103.0)
        return h

    def returning(self):
        h = self.armed()
        self.assertTrue(h.start_return([(2, 0, 0), (1, 0, 0), (0, 0, 0)],
                                       (0, 0, 0), 'local_frd'))
        return h

    def waiting(self):
        h = self.returning()
        for now, x in [(104, 2), (105, 1), (106, 0)]:
            h.set_odom((x, 0, 0), 'local_frd', now, now)
            h.step(now)
        self.assertEqual(h.state, 'WAIT_LINK')
        return h

    def test_confirmed_loss_only(self):
        h = HomingLogic()
        h.set_link('LINK_LOST', 100)
        self.assertEqual(h.state, 'NORMAL')
        h.set_link('LINK_OK', 101)
        h.set_link('LINK_DEGRADED', 102)
        self.assertEqual(h.state, 'NORMAL')
        h.set_link('LINK_LOST', 103)
        self.assertEqual(h.state, 'LINK_LOST')

    def test_sequential_route_and_endpoint(self):
        h = self.returning()
        h.set_odom((0, 0, 0), 'local_frd', 104, 104)
        h.step(104)
        self.assertEqual(h.goal(), (2, 0, 0))
        for now, x, goal in [(105, 2, (1, 0, 0)), (106, 1, (0, 0, 0))]:
            h.set_odom((x, 0, 0), 'local_frd', now, now)
            h.step(now)
            self.assertEqual(h.goal(), goal)
        h.set_odom((0, 0, 0), 'local_frd', 107, 107)
        self.assertEqual(h.step(107), 'WAIT_LINK')

    def test_wait_at_least_sixty_seconds(self):
        h = self.waiting()
        for now in range(107, 166):
            h.set_odom((0, 0, 0), 'local_frd', now, now)
            self.assertEqual(h.step(now), 'WAIT_LINK')
        h.set_odom((0, 0, 0), 'local_frd', 165.99, 165.99)
        self.assertEqual(h.step(165.99), 'WAIT_LINK')
        h.set_odom((0, 0, 0), 'local_frd', 166, 166)
        self.assertEqual(h.step(166), 'LAND')
        self.assertIsNone(h.goal())

    def test_recovery_cancels_every_phase(self):
        for h in [self.armed(), self.returning(), self.waiting()]:
            h.set_link('LINK_OK', 108)
            self.assertEqual(h.step(200), 'NORMAL')
            self.assertIsNone(h.goal())
            h.set_link('LINK_LOST', 201)
            self.assertEqual(h.step(202), 'LINK_LOST')
            self.assertFalse(h.path)

    def test_repeated_loss_does_not_restart_return(self):
        h = self.returning()
        h.set_odom((2, 0, 0), 'local_frd', 104, 104)
        h.step(104)
        h.set_link('LINK_DEGRADED', 105)
        h.set_link('LINK_LOST', 106)
        self.assertEqual(h.index, 1)
        self.assertEqual(h.state, 'RETURN')

    def test_stale_wrong_frame_and_nonfinite_odometry(self):
        for point, frame, stamp in [((2, 0, 0), 'local_frd', 100),
                                     ((2, 0, 0), 'map', 104),
                                     ((float('nan'), 0, 0), 'local_frd', 104),
                                     ((2, 0, 0), 'local_frd', 0),
                                     ((2, 0, 0), 'local_frd', 105)]:
            h = self.returning()
            h.set_odom(point, frame, stamp, 104)
            h.step(104)
            self.assertEqual(h.index, 0)

    def test_wait_restarts_on_stale_pose_or_drift(self):
        for point in [(0, 0, 0), (1, 0, 0)]:
            h = self.waiting()
            if point[0] == 1:
                h.set_odom(point, 'local_frd', 110, 110)
            self.assertEqual(h.step(110), 'WAIT_LINK')
            self.assertIsNone(h.wait_started)
            h.set_odom((0, 0, 0), 'local_frd', 111, 111)
            h.step(111)
            self.assertEqual(h.wait_started, 111)

    def test_clock_rewind_does_not_land(self):
        h = self.waiting()
        h.step(105)
        self.assertIsNone(h.wait_started)
        self.assertEqual(h.state, 'WAIT_LINK')

    def test_invalid_paths_and_immutable_snapshot(self):
        h = self.armed()
        for path, endpoint, frame in [([], (0, 0, 0), 'local_frd'),
                                      ([(1, 0, 0)], (0, 0, 0), 'local_frd'),
                                      ([(0, 0, 0)], (0, 0, 0), ''),
                                      ([(float('inf'), 0, 0)], (0, 0, 0), 'local_frd')]:
            self.assertFalse(h.start_return(path, endpoint, frame))
        path = [[0, 0, 0]]
        self.assertTrue(h.start_return(path, (0, 0, 0), 'local_frd'))
        path[0][0] = 5
        self.assertEqual(h.goal(), (0, 0, 0))

    def test_wait_configuration_cannot_be_shortened(self):
        for kwargs in [dict(wait_link_seconds=59), dict(arrival_radius=0),
                       dict(odom_timeout=0), dict(wait_link_seconds=float('nan'))]:
            self.assertRaises(ValueError, HomingLogic, **kwargs)


if __name__ == '__main__':
    unittest.main()
