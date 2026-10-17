#!/usr/bin/env python
"""Offline tests of peer/heartbeat policy; no network or flight commands."""
import socket
import unittest

from vio_homing.qgc_link import QgcLink


class Message(object):
    type = 6

    def __init__(self, kind, system=255, component=190):
        self.kind, self.system, self.component = kind, system, component

    def get_type(self):
        return self.kind

    def get_srcSystem(self):
        return self.system

    def get_srcComponent(self):
        return self.component

    def get_msgbuf(self):
        return bytearray(b'original-wire-packet')


class Socket(object):
    def __init__(self):
        self.queue = []
        self.sent = []

    def recvfrom(self, size):
        if not self.queue:
            raise socket.error('empty')
        return self.queue.pop(0)

    def sendto(self, data, peer):
        self.sent.append((data, peer))


class Parser(object):
    def __init__(self, messages):
        self.messages = messages

    def parse_buffer(self, unused):
        return self.messages


class QgcTests(unittest.TestCase):
    def setUp(self):
        self.link = object.__new__(QgcLink)
        self.link.sock = Socket()
        self.link.peer = ('192.168.50.2', 14550)
        self.link.gcs_system = 255
        self.link.gcs_component = 190
        self.link.timeout = 4.0
        self.link.last_heartbeat = None
        self.link.rx_packets = self.link.tx_packets = 0
        self.heartbeats = []
        self.written = []
        self.messages = []
        self.link.factory = lambda: Parser(self.messages)
        self.link.on_heartbeat = lambda: self.heartbeats.append(True)

    def receive(self, messages, now=10, ip='192.168.50.2'):
        self.messages = messages
        self.link.sock.queue.append((b'packet', (ip, 14550)))
        self.link.receive(now, self.written.append)

    def test_heartbeat_and_wire_preservation(self):
        self.receive([Message('HEARTBEAT'), Message('PARAM_REQUEST_LIST')])
        self.assertEqual(len(self.heartbeats), 1)
        self.assertEqual(self.written, [b'original-wire-packet'] * 2)

    def test_commands_require_recent_gcs_heartbeat(self):
        self.receive([Message('PARAM_REQUEST_LIST')])
        self.assertEqual(self.written, [])
        self.receive([Message('HEARTBEAT')])
        self.written[:] = []
        self.receive([Message('PARAM_REQUEST_LIST')], now=14)
        self.assertEqual(self.written, [])
        self.receive([Message('PARAM_REQUEST_LIST')], now=9)
        self.assertEqual(self.written, [])

    def test_other_peer_identity_and_vehicle_do_not_refresh(self):
        self.receive([Message('HEARTBEAT')], ip='192.168.137.1')
        self.receive([Message('HEARTBEAT', system=1)])
        self.receive([Message('HEARTBEAT', component=1)])
        msg = Message('HEARTBEAT')
        msg.type = 2
        self.receive([msg])
        self.assertEqual(self.heartbeats, [])
        self.assertEqual(self.written, [])

    def test_reset_discards_old_socket_and_requires_heartbeat(self):
        self.receive([Message('HEARTBEAT')])
        old_socket = self.link.sock
        closed = []
        old_socket.close = lambda: closed.append(True)
        self.link._open_socket = lambda: setattr(self.link, 'sock', Socket())
        self.link.reset()
        self.assertEqual(closed, [True])
        self.assertIsNone(self.link.last_heartbeat)
        self.written[:] = []
        self.receive([Message('PARAM_REQUEST_LIST')])
        self.assertEqual(self.written, [])

    def test_telemetry_to_configured_peer(self):
        self.link.forward(Message('HEARTBEAT', system=1, component=1))
        self.assertEqual(self.link.sock.sent,
                         [(b'original-wire-packet', self.link.peer)])
        self.link.forward(Message('BAD_DATA'))
        self.assertEqual(len(self.link.sock.sent), 1)


if __name__ == '__main__':
    unittest.main()
