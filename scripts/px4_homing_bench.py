#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Disarmed hardware bench: real MAVLink, uORB and ACK; synthetic odometry."""
from __future__ import print_function
import copy
import json
import os
import re
import sys
import time

import rospy
import rosnode
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, String

from homing_stand_sim import Stand, xyz
from vio_homing.homing_transport import FrdToNed


class Px4Bench(Stand):

    def __init__(self):
        self.odom_enabled = False
        self.odom_stale = False
        self.status = {}
        self.acks = []
        self.responses = {}
        self.proofs = []
        self.checks = []
        self.request_id = 0
        self.phase = 'startup'
        self.artifact_dir = '/tmp/vio_homing_px4_bench/run_%d' % int(
            time.time() * 1000000
        )
        os.makedirs(self.artifact_dir)
        self.events_file = open(
            os.path.join(self.artifact_dir, 'events.jsonl'), 'w'
        )
        self.console_file = open(
            os.path.join(self.artifact_dir, 'console.log'), 'w'
        )
        self.alignment = FrdToNed(90, (10, -20, 2))
        super(Px4Bench, self).__init__()
        self.console_request = rospy.Publisher(
            '/vio_homing/px4_console_request', String, queue_size=10
        )
        self.bench_control = rospy.Publisher(
            '/vio_homing/bench_control', String, queue_size=10
        )
        self.extra_subs = [
            rospy.Subscriber(
                '/vio_homing/mavlink_transport_status', String, self.on_status
            ),
            rospy.Subscriber(
                '/vio_homing/mavlink_tx_event', String, self.on_tx
            ),
            rospy.Subscriber(
                '/vio_homing/mavlink_command_ack', String, self.on_ack
            ),
            rospy.Subscriber(
                '/vio_homing/px4_console_response', String, self.on_console
            )
        ]

    def record(self, kind, data):
        if not self.events_file.closed:
            self.events_file.write(
                json.dumps(
                    dict(
                        time=time.time(),
                        phase=self.phase,
                        kind=kind,
                        data=data
                    )
                ) + '\n'
            )

    def checkpoint(self, label):
        self.checks.append(label)
        self.record('PASS', label)
        self.events_file.flush()
        print('PASS:', label)
        sys.stdout.flush()

    def on_status(self, msg):
        with self.lock:
            self.status = json.loads(msg.data)
            self.record('transport_status', self.status)

    def on_tx(self, msg):
        with self.lock:
            self.record('mavlink_tx', json.loads(msg.data))

    def on_ack(self, msg):
        with self.lock:
            data = json.loads(msg.data)
            self.acks.append(data)
            self.record('px4_ack', data)

    def on_console(self, msg):
        with self.lock:
            data = json.loads(msg.data)
            self.responses[data['id']] = data

    def publish(self, event):
        with self.lock:
            if self.odom_enabled:
                msg = Odometry()
                msg.header.stamp = rospy.Time.now()
                if self.odom_stale:
                    msg.header.stamp -= rospy.Duration(5.0)
                msg.header.frame_id = 'local_frd'
                msg.child_frame_id = 'body_frd'
                p = msg.pose.pose.position
                p.x, p.y, p.z = self.current_position
                msg.pose.pose.orientation.w = 1.0
                self.odom_pub.publish(msg)
            if self.heartbeat:
                self.sock.sendto(
                    b'VIO_HOMING_HEARTBEAT hardware bench',
                    ('127.0.0.1', self.port)
                )

    def console(self, command):
        self.request_id += 1
        request = str(self.request_id)
        self.console_request.publish(
            String(data=json.dumps(dict(id=request, command=command)))
        )
        self.wait(lambda: request in self.responses, 12, 'console ' + command)
        response = self.responses.pop(request)
        if response.get('error') or not response.get('text'):
            raise AssertionError('PX4 console failed: %r' % response)
        text = response['text']
        self.console_file.write(
            '\nPHASE %s | COMMAND %s\n%s\n' % (self.phase, command, text)
        )
        self.console_file.flush()
        self.record('px4_console', dict(command=command, text=text))
        return text

    def action(self, action, **kwargs):
        kwargs['action'] = action
        before = len(self.acks)
        self.bench_control.publish(String(data=json.dumps(kwargs)))
        if action in ('offboard', 'restore'):
            self.wait(
                lambda: any(a['command'] == 176 for a in self.acks[before:]),
                5, action + ' ACK'
            )
            ack = [a for a in self.acks[before:] if a['command'] == 176][-1]
            if ack['result'] != 0:
                raise AssertionError(
                    'Disarmed mode request refused; do not bypass: %r' % ack
                )

    def status_disarmed(self, expected_nav=None):
        text = self.console('listener vehicle_status 1')
        if not re.search(r'arming_state:\s+1\b',
                         text) or not re.search(r'armed_time:\s+0\b', text):
            raise AssertionError('PX4 is not confirmed disarmed: ' + text)
        if expected_nav is not None and not re.search(
            r'nav_state:\s+%d\b' % expected_nav, text
        ):
            raise AssertionError('Unexpected PX4 navigation state: ' + text)
        return text

    def confirm_position(self, command, expected):
        text = self.console(command)
        match = re.search(r'position:\s*\[([^]]+)\]', text)
        if match is None:
            raise AssertionError('No uORB position: ' + text)
        actual = tuple(float(v.strip()) for v in match.group(1).split(','))
        if any(abs(a - b) > 1e-4 for a, b in zip(actual, expected)):
            raise AssertionError(
                'PX4 position mismatch: %r != %r' % (actual, expected)
            )
        self.proofs.append(
            dict(
                phase=self.phase,
                command=command,
                expected=expected,
                actual=actual,
                text=text
            )
        )
        return text

    def no_commands(self, seconds, state='NORMAL'):
        self.hold(0.7, state)  # Drain only already in-flight ROS/MAVLink data.
        start_sp, start_land = self.status['setpoint_tx'], self.status[
            'land_tx']
        end = time.time() + seconds
        while time.time() < end:
            self.hold(0.1, state)
            if (
                self.status['setpoint_tx'] != start_sp
                or self.status['land_tx'] != start_land
            ):
                raise AssertionError('Commands continued after cancellation')

    def enter_offboard(self):
        self.wait(
            lambda: self.status.get('setpoint_tx', 0) > 5, 5,
            'setpoints on UART'
        )
        self.hold(2.0, 'RETURN')
        self.action('offboard')
        self.status_disarmed(14)
        text = self.console('listener offboard_control_mode 1')
        if not (
            'position: True' in text and 'velocity: False' in text
            and 'acceleration: False' in text
        ):
            raise AssertionError(
                'PX4 did not recognize position-only offboard setpoints'
            )

    def follow_with_px4(self, snapshot):
        targets = []
        for p in snapshot:
            if not targets or targets[-1] != p:
                targets.append(p)
        # Initial route point(s) are already reached at the loss location.
        while targets and targets[0] == self.current_position:
            targets.pop(0)
        for target in targets:
            self.wait(
                lambda: self.goal is not None and xyz(self.goal.pose) ==
                target, 5, 'next ROS target ' + str(target)
            )
            self.confirm_position(
                'listener trajectory_setpoint 1',
                self.alignment.convert(target)
            )
            with self.lock:
                self.current_position = target
            self.hold(0.15)
        self.wait(lambda: self.state == 'WAIT_LINK', 5, 'last good endpoint')
        self.checkpoint(
            'ordered ROS targets confirmed in PX4 trajectory_setpoint'
        )

    def run(self):
        self.wait(
            lambda: self.link_state == 'WAITING' and self.state == 'NORMAL' and
            self.status.get('link_alive') and self.status.get('timesync') and
            self.console_request.get_num_connections(), 20,
            'hardware bench startup'
        )
        version = self.console('ver all')
        if '6ea3539157ca358c70a515878b77077af7d4611d' not in version:
            raise AssertionError('PX4 firmware differs from audited version')
        self.status_disarmed()

        self.phase = 'empty_path_missing_odometry'
        self.recover()
        empty = rospy.Publisher('/vio_homing/return_path', Path, queue_size=1)
        self.wait(
            lambda: empty.get_num_connections() > 0, 3, 'empty path publisher'
        )
        empty.publish(Path())
        self.heartbeat = False
        self.wait(
            lambda: self.state == 'LINK_LOST', 5, 'loss without odometry'
        )
        self.no_commands(1, 'LINK_LOST')
        if self.status['setpoint_tx'] or self.status['land_tx']:
            raise AssertionError('Empty route/missing odometry sent commands')
        empty.unregister()
        self.recover()
        self.checkpoint(
            'empty path and absent odometry produce no MAVLink commands'
        )
        self.odom_enabled = True

        self.phase = 'return_cancel_stale_and_mavlink_loss'
        snapshot, first, delay = self.lose_link()
        self.enter_offboard()
        self.confirm_position(
            'listener vehicle_visual_odometry 1', self.current_position
        )
        self.odom_stale = True
        self.hold(1.5, 'RETURN')
        frozen = self.status['setpoint_tx']
        self.hold(1.0, 'RETURN')
        if self.status['setpoint_tx'] != frozen:
            raise AssertionError(
                'Stale odometry did not stop MAVLink setpoints'
            )
        self.odom_stale = False
        self.wait(
            lambda: self.status['setpoint_tx'] > frozen, 3,
            'fresh odometry resumes stream'
        )
        self.checkpoint('stale odometry stops setpoint TX')
        self.action('disconnect', seconds=4.0)
        self.wait(
            lambda: self.status.get('link_alive') is False, 3,
            'serial connection closed'
        )
        frozen = self.status['setpoint_tx']
        self.hold(1.5, 'RETURN')
        if self.status['setpoint_tx'] != frozen:
            raise AssertionError(
                'Setpoints sent during closed MAVLink connection'
            )
        self.wait(
            lambda: self.status.get('link_alive') and self.status.
            get('timesync') and self.status['setpoint_tx'] > frozen, 15,
            'MAVLink reconnect'
        )
        self.checkpoint(
            'closed MAVLink connection stops TX and reconnects with TIMESYNC'
        )
        self.recover()
        self.no_commands(1.0)
        self.action('restore')
        self.checkpoint(
            'heartbeat recovery during RETURN stops MAVLink setpoints'
        )

        self.phase = 'wait_link_cancel_old_timer'
        snapshot, first, delay = self.lose_link()
        self.enter_offboard()
        self.follow_with_px4(snapshot)
        self.hold(0.5, 'WAIT_LINK')
        self.recover()
        self.action('restore')
        print('Waiting 61 real seconds to verify cancelled LAND timer...')
        sys.stdout.flush()
        self.no_commands(61.0)
        if self.status['land_tx'] != 0:
            raise AssertionError('Cancelled WAIT_LINK timer sent LAND')
        self.checkpoint(
            'WAIT_LINK recovery cancels LAND beyond the old 60-second deadline'
        )

        self.phase = 'full_return_wait_land'
        snapshot, first, delay = self.lose_link()
        self.enter_offboard()
        self.follow_with_px4(snapshot)
        wait_start = [t for t, s in self.transitions if s == 'WAIT_LINK'][-1]
        stats_start = copy.deepcopy(self.status)
        rate_start = time.time()
        self.hold(5.0, 'WAIT_LINK')
        elapsed = time.time() - rate_start
        imu_rate = (self.status['imu_rx'] - stats_start['imu_rx']) / elapsed
        odom_rate = (self.status['odom_tx'] - stats_start['odom_tx']) / elapsed
        if not (90 <= imu_rate <= 110 and 27 <= odom_rate <= 33):
            raise AssertionError(
                'IMU/ODOMETRY regression: %.2f / %.2f Hz' %
                (imu_rate, odom_rate)
            )
        self.checkpoint('IMU about 100 Hz and ODOMETRY about 30 Hz preserved')
        self.confirm_position(
            'listener vehicle_visual_odometry 1', self.current_position
        )
        print('Waiting for full 60-second WAIT_LINK and PX4 LAND ACK...')
        sys.stdout.flush()
        self.wait(
            lambda: self.state == 'LAND' and self.status.get('land_tx') == 1
            and self.status.get('land_ack') not in (None, 'PENDING'), 65,
            'LAND and real PX4 ACK'
        )
        land_ack = self.status['land_ack']
        real_ack = [a for a in self.acks if a['command'] == 21]
        if not real_ack:
            raise AssertionError('No actual COMMAND_ACK from Pixhawk')
        wait_duration = real_ack[-1]['ros_time'] - wait_start
        if wait_duration < 60.0:
            raise AssertionError(
                'LAND sent before 60 seconds: %.6f' % wait_duration
            )
        command_text = self.console('listener vehicle_command 1')
        ack_text = self.console('listener vehicle_command_ack 1')
        if not re.search(r'command:\s+21\b', command_text
                         ) or not re.search(r'command:\s+21\b', ack_text):
            raise AssertionError('PX4 uORB did not retain NAV_LAND and ACK')
        self.status_disarmed()
        self.checkpoint(
            'LAND received by PX4: vehicle_command, '
            'vehicle_command_ack and MAVLink ACK'
        )

        self.phase = 'repeated_land_and_manager_restart'
        repeated = rospy.Publisher(
            '/vio_homing/land_command', Bool, queue_size=10
        )
        self.wait(
            lambda: repeated.get_num_connections(), 3,
            'repeated LAND publisher'
        )
        for unused in range(10):
            repeated.publish(Bool(data=True))
            time.sleep(0.1)
        repeated.unregister()
        self.hold(1.0, 'LAND')
        if self.status['land_tx'] != 1:
            raise AssertionError('Repeated ROS LAND was resent to PX4')
        self.checkpoint('repeated LAND sends one command per episode')
        session = self.status['session']
        killed, failed = rosnode.kill_nodes(['/px4_bench_manager'])
        if failed:
            raise AssertionError('Could not restart manager: %r' % failed)
        self.wait(
            lambda: self.status.get('session') != session and self.state ==
            'NORMAL', 10, 'new manager session after restart'
        )
        self.no_commands(1.0)
        if self.status['land_tx'] != 1:
            raise AssertionError('Manager restart replayed LAND')
        self.recover()
        self.action('restore')
        self.status_disarmed(4)
        self.checkpoint(
            'manager restart stops old goals and does not replay LAND'
        )

        self.phase = 'complete'
        result = dict(
            passed=True,
            px4_git_hash='6ea3539157ca358c70a515878b77077af7d4611d',
            px4_version='1.16.0',
            wait_link_seconds=wait_duration,
            land_ack=land_ack,
            land_tx=self.status['land_tx'],
            imu_rate_hz=imu_rate,
            odometry_rate_hz=odom_rate,
            frame_alignment=dict(yaw_degrees=90, origin_ned=[10, -20, 2]),
            px4_position_proofs=self.proofs,
            checks=self.checks,
            artifacts=self.artifact_dir,
            final_status=self.status,
            physical_motion=False,
            camera=False,
            wifi=False,
            transitions=self.transitions
        )
        self.save_result(result)
        print(
            'PASS: PX4 hardware bench completed; artifacts:', self.artifact_dir
        )
        sys.stdout.flush()
        return result

    def save_result(self, result):
        with open(
            os.path.join(self.artifact_dir, 'result.json'), 'w'
        ) as handle:
            json.dump(result, handle, indent=2)
        with open('/tmp/vio_homing_px4_bench/latest.json', 'w') as handle:
            json.dump(result, handle, indent=2)

    def close(self):
        super(Px4Bench, self).close()
        with self.lock:
            self.events_file.close()
            self.console_file.close()


if __name__ == '__main__':
    rospy.init_node('px4_homing_bench_test')
    bench = Px4Bench()
    try:
        bench.run()
    except Exception as error:
        print('FAIL:', str(error), 'artifacts:', bench.artifact_dir)
        sys.stdout.flush()
        bench.save_result(
            dict(
                passed=False,
                error=str(error),
                artifacts=bench.artifact_dir,
                checks=bench.checks,
                status=bench.status
            )
        )
        try:
            bench.action('restore')
        except Exception:
            pass
        sys.exit(1)
    finally:
        bench.close()
