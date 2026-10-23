# -*- coding: utf-8 -*-
"""Read-only, bounded NSH probe; never executes user-provided commands."""
import re


class Px4Health(object):
    COMMAND = 'listener vehicle_local_position 1\n'

    def __init__(self):
        self.values = None
        self.received = None
        self.pending = False
        self.started = None
        self.last_rx = None
        self.last_query = -1e20
        self.text = ''
        self.reason = 'WAITING_FOR_PX4_HEALTH'

    def begin(self, now):
        self.pending = True
        self.started = self.last_query = now
        self.last_rx = None
        self.text = ''

    def feed(self, text, now):
        if self.pending:
            self.text += text
            self.last_rx = now
            if len(self.text) > 16000:
                self.text = ''
                self.reason = 'PX4_HEALTH_RESPONSE_TOO_LARGE'

    def finish(self, now, px4_to_ros, ros_now):
        self.pending = False
        try:
            if self.text.count('TOPIC: vehicle_local_position') != 1:
                raise ValueError('No unique local-position response')
            stamp = int(re.search(r'\btimestamp:\s*(\d+)', self.text).group(1))
            converted = px4_to_ros(stamp)
            if not 0 <= ros_now-converted <= 2. or converted < self.started-.1:
                raise ValueError('Stale PX4 health response')
            names = ('heading_good_for_control', 'xy_valid', 'z_valid', 'v_xy_valid', 'v_z_valid')
            values = {}
            for name in names:
                matches = re.findall(r'\b'+name+r':\s*(True|False)\b', self.text)
                if len(matches) != 1:
                    raise ValueError('Incomplete PX4 health response')
                values[name] = matches[0] == 'True'
            self.values, self.received = values, now
            self.reason = 'READY' if all(values.values()) else 'PX4_NAV_OR_HEADING_NOT_READY'
        except (ValueError, AttributeError, TypeError):
            self.values = None
            self.received = None
            self.reason = 'PX4_HEALTH_UNAVAILABLE'

    def fresh(self, now):
        return self.received is not None and 0 <= now-self.received <= 4.

    def ready(self, now, require_heading=True):
        names = ('xy_valid', 'z_valid', 'v_xy_valid', 'v_z_valid')
        return (self.fresh(now) and self.values is not None
                and all(self.values[name] for name in names)
                and (not require_heading or self.values['heading_good_for_control']))
