# -*- coding: utf-8 -*-
"""Single trusted GCS peer; preserve original MAVLink wire packets."""
import socket


class QgcLink(object):
    def __init__(self, bind_ip, bind_port, peer_ip, peer_port, parser_factory,
                 heartbeat_callback, gcs_system=255, gcs_component=190,
                 timeout=4.0):
        # Literal unicast IPv4 only; no learning peers from arbitrary traffic.
        socket.inet_pton(socket.AF_INET, peer_ip)
        socket.inet_pton(socket.AF_INET, bind_ip)
        if peer_ip in ('0.0.0.0', '255.255.255.255'):
            raise ValueError('A fixed GCS unicast address is required')
        self.peer = (peer_ip, int(peer_port))
        self.factory = parser_factory
        self.on_heartbeat = heartbeat_callback
        self.gcs_system = int(gcs_system)
        self.gcs_component = int(gcs_component)
        self.timeout = float(timeout)
        self.last_heartbeat = None
        self.rx_packets = 0
        self.tx_packets = 0
        self.bind = (bind_ip, int(bind_port))
        self._open_socket()

    def receive(self, now, write):
        # Bound each loop so a noisy peer cannot starve PX4 receive/TIMESYNC.
        for unused in range(32):
            try:
                data, address = self.sock.recvfrom(4096)
            except socket.error:
                break
            if address[0] != self.peer[0]:
                continue
            parser = self.factory()
            try:
                messages = parser.parse_buffer(data) or []
            except Exception:
                continue
            for msg in messages:
                if (msg.get_type() == 'BAD_DATA' or
                        msg.get_srcSystem() != self.gcs_system or
                        msg.get_srcComponent() != self.gcs_component):
                    continue
                if msg.get_type() == 'HEARTBEAT' and msg.type == 6:
                    self.last_heartbeat = now
                    self.on_heartbeat()
                if (self.last_heartbeat is None or
                        now < self.last_heartbeat or
                        now - self.last_heartbeat >= self.timeout):
                    continue
                # Raw forwarding retains sysid/compid/sequence/signature.
                write(bytes(msg.get_msgbuf()))
                self.rx_packets += 1

    def forward(self, msg):
        if msg.get_type() == 'BAD_DATA':
            return
        try:
            self.sock.sendto(bytes(msg.get_msgbuf()), self.peer)
            self.tx_packets += 1
        except socket.error:
            pass

    def reset(self):
        self.last_heartbeat = None
        # Recreate the socket to discard every queued datagram atomically.
        self.sock.close()
        self._open_socket()

    def _open_socket(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(self.bind)
        self.sock.setblocking(False)

    def close(self):
        self.sock.close()
