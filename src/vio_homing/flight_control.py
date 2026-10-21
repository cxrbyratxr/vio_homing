# -*- coding: utf-8 -*-
"""Flight-mode ownership and bounded commands. Never arms a vehicle."""
OFFBOARD = (6, 0)
HOLD = (4, 3)
LAND = (4, 6)


class FlightControl(object):
    def __init__(self):
        self.phase = 'IDLE'
        self.reason = 'WAITING_FOR_RETURN'
        self.started = None
        self.owned = False
        self.hold = None
        self.pending = None
        self.ack = None
        self.session = None

    def command(self, kind, now):
        self.pending = kind
        self.ack = None
        self.started = now
        return kind

    def acknowledge(self, command, result):
        if ((command == 176 and self.pending in ('OFFBOARD', 'HOLD'))
                or (command == 21 and self.pending == 'LAND')):
            self.ack = result

    def step(self, now, session, armed, airborne, alive, mode, requested,
             normal, land, healthy, position, target, failsafe=False):
        """Return (position to stream, command or None); inputs already gated."""
        if not alive:
            self.phase, self.reason = 'FAULT', 'MAVLINK_LOST_PX4_FAILSAFE'
            self.owned = False
            return None, None
        if not armed or not airborne:
            self.phase, self.reason = 'IDLE', 'DISARMED_OR_NOT_AIRBORNE'
            self.owned = False
            self.pending = None
            self.session = session
            return None, None
        if failsafe:
            self.phase, self.reason = 'FAULT', 'PX4_CRITICAL_STATE'
            self.owned = False
            return None, None
        if self.session is not None and session != self.session and self.owned:
            self.phase, self.reason = 'REQUEST_LAND', 'MANAGER_RESTART'
            self.session = session
            return None, self.command('LAND', now)
        self.session = session
        if self.phase == 'FAULT':
            if normal:
                self.phase, self.reason = 'IDLE', 'RECOVERED_WAITING_NEW_EPISODE'
            return None, None
        if self.phase == 'IDLE':
            if requested and healthy and target is not None and position is not None:
                self.phase, self.reason = 'PRESTREAM', 'OFFBOARD_WARMUP'
                self.started, self.hold = now, tuple(position)
            return self.hold if self.phase == 'PRESTREAM' else None, None
        if self.phase in ('PRESTREAM', 'REQUEST_OFFBOARD', 'ACTIVE'):
            # An operator/PX4 mode change after takeover relinquishes ownership.
            if self.phase == 'ACTIVE' and mode != OFFBOARD:
                self.phase, self.reason = 'FAULT', 'PX4_MODE_CHANGED'
                self.owned = False
                return None, None
            if normal or (not requested and not land):
                if self.phase == 'PRESTREAM':
                    self.phase, self.reason = 'IDLE', 'RETURN_CANCELLED_BEFORE_TAKEOVER'
                    return None, None
                self.hold = tuple(position) if position is not None else self.hold
                self.phase, self.reason = 'RELEASING', 'LINK_RECOVERED_HOLD_HANDOVER'
                return self.hold, self.command('HOLD', now)
            if not healthy:
                if self.phase == 'PRESTREAM':
                    self.phase, self.reason = 'FAULT', 'INVALID_NAV_BEFORE_TAKEOVER'
                    return None, None
                self.phase, self.reason = 'REQUEST_LAND', 'NAVIGATION_OR_INPUT_LOST'
                return None, self.command('LAND', now)
            if self.phase == 'PRESTREAM':
                if now-self.started >= 1.2:
                    self.phase = 'REQUEST_OFFBOARD'
                    return self.hold, self.command('OFFBOARD', now)
                return self.hold, None
            if self.phase == 'REQUEST_OFFBOARD':
                # ACK is diagnostic; actual mode from PX4 is authoritative.
                if mode == OFFBOARD:
                    self.phase, self.reason = 'ACTIVE', 'RETURN_CONTROL_ACTIVE'
                    self.owned = True
                    self.pending = None
                elif self.ack not in (None, 0, 5) or now-self.started > 3.:
                    self.phase, self.reason = 'REQUEST_LAND', 'OFFBOARD_REJECTED_OR_TIMEOUT'
                    return None, self.command('LAND', now)
                else:
                    return self.hold, None
            if self.phase == 'ACTIVE':
                if land:
                    self.phase, self.reason = 'REQUEST_LAND', 'WAIT_LINK_FINISHED'
                    return position, self.command('LAND', now)
                return target, None
        if self.phase == 'RELEASING':
            if mode != OFFBOARD:
                self.phase, self.reason = 'IDLE', 'CONTROL_RELEASED'
                self.owned = False
                self.pending = None
                return None, None
            if not healthy or self.ack not in (None, 0, 5) or now-self.started > 3.:
                self.phase, self.reason = 'FAULT', 'HOLD_HANDOVER_FAILED_PX4_FAILSAFE'
                self.owned = False
                return None, None
            return self.hold, None
        if self.phase == 'REQUEST_LAND':
            if mode == LAND:
                self.phase, self.reason = 'LANDING', 'PX4_LAND_CONFIRMED'
                self.pending = None
                self.owned = False
            elif self.ack not in (None, 0, 5) or now-self.started > 5.:
                self.phase, self.reason = 'FAULT', 'LAND_UNCONFIRMED_PX4_FAILSAFE'
                self.owned = False
                return None, None
            # Only maintain offboard proof until PX4 takes over landing.
            return position if mode == OFFBOARD and healthy else None, None
        return None, None
