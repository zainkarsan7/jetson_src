#!/usr/bin/env python3
"""Torch-height Nano Every bridge, protocol v2.

Positive steps retract toward terminal HOME.
Negative steps advance. PREP and SEARCH both advance.

Default measured calibration: 40 pulses/mm.

Services acknowledge acceptance; status/motion_event report completion.
No movement command is replayed across serial sessions.
All parameters are startup-only.
"""

import json
import math
import secrets
import threading
import time

import serial
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, SetBool
from hb_robot_interfaces.msg import TorchStatus
from hb_robot_interfaces.srv import Jog


def frame(*fields):
    body = ','.join(map(str, fields)).encode('ascii')
    check = 0
    for byte in body:
        check ^= byte
    return b'$' + body + f'*{check:02X}\n'.encode()


def decode(raw):
    body, check = raw.strip().rsplit(b'*', 1)
    if not body.startswith(b'$') or len(check) != 2:
        raise ValueError('framing')
    value = 0
    for byte in body[1:]:
        value ^= byte
    if value != int(check, 16):
        raise ValueError('checksum')
    return body[1:].decode('ascii').split(',')


class TorchHeightBridge(Node):
    def __init__(self):
        super().__init__('torch_height_bridge')

        defaults = {
            'port': '/dev/ttyACM1',
            'baud': 230400,
            'status_timeout_s': 0.75,
            'command_timeout_s': 2.0,
            'command_retry_s': 0.25,
            'heartbeat_period_s': 0.05,
            'link_timeout_ms': 500,
            'torch_status_timeout_s': 0.75,

            'calibration_mode': False,
            'steps_per_mm': 40.0,
            'initial_home_direction': 1,
            'positive_direction_high': True,

            'prep_distance_mm': 10.0,
            'search_distance_mm': 15.0,
            'home_max_travel_mm': 27.0,
            'search_duration_s': 4.0,
            'prep_speed_mm_s': 15.0,
            'home_speed_mm_s': 10.0,

            'jog_max_steps': 500,
            'jog_rate_steps_s': 400,
            'jog_window_steps': 1000,

            'arc_established_wait_ms': 300,
            'arc_establish_timeout_ms': 2000,

            'voltage_tracking_enabled': False,
            'voltage_source': 'analog_input_value',
            'voltage_analog_domain': 1,
            'voltage_target_v': 3.5,
            'voltage_lower_margin_v': 0.5,
            'voltage_upper_margin_v': 0.5,
            'voltage_min_valid_v': 0.1,
            'voltage_max_valid_v': 10.0,

            # Your supplied convention:
            # -1 advances when voltage is above the upper bound.
            'voltage_high_direction': -1,
            'voltage_correction_dwell_ms': 100,
            'tracking_speed_mm_s': 0.5,
            'tracking_max_offset_mm': 2.0,

            # CutProfileServer retains ownership of return HOME.
            'auto_home_after_cut': False,
        }

        descriptor = ParameterDescriptor(read_only=True)
        self.p = {
            name: self.declare_parameter(name, value, descriptor).value
            for name, value in defaults.items()
        }
        self.config = self.make_config()

        # Short cache operations only; no serial I/O while holding this.
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.wake = threading.Event()

        self.tracking_authorized = False
        self.session = 0
        self.configured = False
        self.pending = None
        self.urgent = None
        self.next_id = 0

        self.torch = None
        self.torch_at = 0.0
        self.arc_since = None
        self.correction_since = None
        self.correction_direction = 0

        self.h = None
        self.h_at = 0.0
        self.height_revision = 0
        self.published_height = None
        self.last_frame_at = 0.0

        self.stats = dict(
            bad_frames=0,
            config_sends=0,
            max_write_ms=0.0,
            max_heartbeat_gap_ms=0.0,
            last_ack_ms=0.0,
            reconnects=0,
            error='',
        )
        self.last_done = None
        self.last_diagnostic = 0.0

        self.pub = {}
        for name, message_type in (
            ('torch_height/status', String),
            ('torch_height/at_home', Bool),
            ('torch_height/arc_established', Bool),
            ('torch_height/voltage_in_band', Bool),
            ('torch_height/motion_event', String),
            ('torch_height/diagnostics', String),
            ('torch_height/serial_online', Bool),
            ('torch_height/status_full', String),
        ):
            self.pub[name] = self.create_publisher(message_type, name, 10)

        self.create_subscription(
            TorchStatus,
            '/cut_profile/torch_status_debug',
            
            self.on_torch,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            ),
        )

        # Allows STOP to preempt a service awaiting an ACK.
        self.services_group = ReentrantCallbackGroup()

        operations = {
            'home': ('HOME', ()),
            'prepare': ('PREP', ()),
            'arc_search': ('SEARCH', ()),
            'stop': ('STOP', ()),
            'reset_fault': ('RESET_FAULT', ()),
        }
        for name, (operation, args) in operations.items():
            self.create_service(
                Trigger,
                'torch_height/' + name,
                self.callback(operation, args),
                callback_group=self.services_group,
            )

        self.create_service(
            Jog,
            'torch_height/jog',
            self.jog,
            callback_group=self.services_group,
        )
        self.create_service(
            SetBool,
            '/torch_height/set_tracking_enabled',
            self.set_tracking,
            callback_group=self.services_group,
        )

        self.create_timer(0.02, self.publish_cached)

        if self.p['calibration_mode']:
            self.get_logger().warn(
                'CALIBRATION MODE: physically inhibit torch; cutting disabled'
            )

        self.worker = threading.Thread(
            target=self.io_loop,
            name='nano-serial',
            daemon=True,
        )
        self.worker.start()

    def make_config(self):
        p = self.p
        if p['auto_home_after_cut']:
            raise ValueError(
                'Set auto_home_after_cut=False; cut server owns HOME'
            )

        for name, value in p.items():
            if isinstance(value, float):
                if not math.isfinite(value) or value < 0:
                    raise ValueError(
                        f'{name}: require finite nonnegative value'
                    )

        for name in (
            'steps_per_mm',
            'status_timeout_s',
            'torch_status_timeout_s',
            'command_timeout_s',
            'command_retry_s',
        ):
            if p[name] <= 0:
                raise ValueError(f'{name} must be positive')

        if not 0.02 <= p['heartbeat_period_s'] <= 0.1:
            raise ValueError('heartbeat_period_s: 0.02..0.1')
        if not 100 <= p['link_timeout_ms'] <= 2000:
            raise ValueError('link_timeout_ms: 100..2000')
        if 1000 * p['heartbeat_period_s'] * 3 > p['link_timeout_ms']:
            raise ValueError('Link timeout too short')
        if not 0 < p['command_retry_s'] < p['command_timeout_s']:
            raise ValueError('Invalid command retry')
        if p['initial_home_direction'] != 1:
            raise ValueError('Invalid home direction')

        total_travel_mm = p['prep_distance_mm'] + p['search_distance_mm']

        if not (
            0 <= p['arc_established_wait_ms']
            < p['arc_establish_timeout_ms'] <= 30000
        ):
            raise ValueError('Arc dwell')
        if not 0 <= p['voltage_correction_dwell_ms'] <= 5000:
            raise ValueError('Voltage dwell')
        if p['voltage_source'] not in (
            'analog_input_value', 'arc_voltage_v'
        ):
            raise ValueError('Voltage source')

        self.low = p['voltage_target_v'] - p['voltage_lower_margin_v']
        self.high = p['voltage_target_v'] + p['voltage_upper_margin_v']

        if not (
            p['voltage_min_valid_v'] <= self.low
            < self.high <= p['voltage_max_valid_v']
        ):
            raise ValueError('Voltage band')

        if p['calibration_mode'] and p['voltage_tracking_enabled']:
            raise ValueError('Tracking disabled in calibration')
        if (
            p['voltage_tracking_enabled']
            and p['voltage_high_direction'] not in (-1, 1)
        ):
            raise ValueError('Set voltage polarity')
        if not 0.5 <= p['search_duration_s'] <= 30:
            raise ValueError('Search duration: 0.5..30s')
        if (
            not p['calibration_mode']
            and p['home_max_travel_mm'] < total_travel_mm
        ):
            raise ValueError(
                'Home travel budget must cover the negative cutting envelope'
            )

        scale = p['steps_per_mm']
        prep_steps = round(p['prep_distance_mm'] * scale)
        search_steps = round(p['search_distance_mm'] * scale)
        home_steps = round(p['home_max_travel_mm'] * scale)
        prep_rate = round(p['prep_speed_mm_s'] * scale)
        home_rate = round(p['home_speed_mm_s'] * scale)

        # Compensate for the 200ms linear acceleration ramp.
        search_rate = math.ceil(
            search_steps / (p['search_duration_s'] - 0.1)
        )

        tracking_rate = (
            round(p['tracking_speed_mm_s'] * scale)
            if p['voltage_tracking_enabled'] else 0
        )
        tracking_offset = (
            round(p['tracking_max_offset_mm'] * scale)
            if tracking_rate else 0
        )

        if not all(
            1 <= value <= 100000
            for value in (prep_steps, search_steps, home_steps)
        ):
            raise ValueError('Travel: 1..100000 steps')

        if not all(
            1 <= value <= 2000
            for value in (prep_rate, home_rate, search_rate)
        ):
            raise ValueError('Speed: 1..2000 steps/s')

        if p['voltage_tracking_enabled'] and not (
            1 <= tracking_rate <= 2000
            and 1 <= tracking_offset <= 100000
        ):
            raise ValueError('Tracking limits')

        if (
            not 1 <= p['jog_max_steps'] <= 500
            or not 1 <= p['jog_rate_steps_s'] <= 1000
        ):
            raise ValueError('Jog limits')
        if not p['jog_max_steps'] <= p['jog_window_steps'] <= 100000:
            raise ValueError('Jog window')

        return [
            int(p['calibration_mode']),
            prep_steps,
            search_steps,
            home_steps,
            prep_rate,
            home_rate,
            search_rate,
            round(p['search_duration_s'] * 1000),
            int(p['positive_direction_high']),
            1,
            p['arc_establish_timeout_ms'],
            tracking_rate,
            tracking_offset,
            p['link_timeout_ms'],
            p['jog_max_steps'],
            p['jog_rate_steps_s'],
            p['jog_window_steps'],
        ]

    def permission(self, now):
        t = self.torch

        if (
            t is None
            or now - self.torch_at > self.p['torch_status_timeout_s']
            or not t.io_connected
            or t.fault
        ):
            return 3

        if not t.command_on:
            return 4 if t.output_on or t.transfer_detected else 0

        if t.output_on:
            return 2 if t.transfer_detected else 1

        return 3 if t.transfer_detected else 5

    def voltage(self):
        t = self.torch
        if t is None:
            return None

        if self.p['voltage_source'] == 'analog_input_value':
            if (
                not t.analog_valid
                or t.analog_domain != self.p['voltage_analog_domain']
            ):
                return None
            value = t.analog_input_value
        else:
            if not t.arc_voltage_valid:
                return None
            value = t.arc_voltage_v

        if (
            math.isfinite(value)
            and self.p['voltage_min_valid_v']
            <= value <= self.p['voltage_max_valid_v']
        ):
            return value

        return None

    def control(self, now):
        # Caller holds self.lock.
        if self.p['calibration_mode']:
            return 0, 0, 0, 0

        mode = self.permission(now)
        qualified = (
            mode == 2
            and self.arc_since is not None
            and (now - self.arc_since) * 1000
            >= self.p['arc_established_wait_ms']
        )

        height_ok = (
            self.configured
            and self.h is not None
            and now - self.h_at < self.p['status_timeout_s']
            and self.h[3] == 0
            and self.h[11] == 1
            and self.h[0] in ('HOLD', 'TRACK')
        )

        if not qualified or not height_ok:
            self.tracking_authorized = False

        enabled = (
            self.tracking_authorized
            and self.p['voltage_tracking_enabled']
            and qualified
        )
        correction = 0

        if enabled:
            if self.voltage() is None:
                self.tracking_authorized = False
                return 3, 0, 0, 0

            if (
                self.correction_since is not None
                and (now - self.correction_since) * 1000
                >= self.p['voltage_correction_dwell_ms']
            ):
                correction = self.correction_direction

        return mode, int(qualified), correction, int(enabled)

    def set_tracking(self, request, response):
        now = time.monotonic()

        with self.lock:
            if request.data:
                fresh = (
                    self.configured
                    and self.h is not None
                    and now - self.h_at < self.p['status_timeout_s']
                )
                valid = (
                    not self.p['calibration_mode']
                    and self.p['voltage_tracking_enabled']
                    and fresh
                    and self.h[0] in ('HOLD', 'TRACK')
                    and self.h[3] == 0
                    and self.h[11] == 1
                    and self.h[4] <= self.p['link_timeout_ms']
                    and self.permission(now) == 2
                    and self.arc_since is not None
                    and (now - self.arc_since) * 1000
                    >= self.p['arc_established_wait_ms']
                    and self.voltage() is not None
                )

                if not valid:
                    response.success = False
                    response.message = 'Tracking prerequisites not satisfied'
                    return response

            self.tracking_authorized = bool(request.data)

        self.wake.set()
        response.success = True
        response.message = (
            'Tracking enabled' if request.data else 'Tracking disabled'
        )
        return response

    def on_torch(self, msg):
        now = time.monotonic()

        with self.lock:
            gap = now - self.torch_at > self.p['torch_status_timeout_s']
            self.torch = msg
            self.torch_at = now

            if gap or self.permission(now) != 2:
                self.tracking_authorized = False
                self.arc_since = None
                self.correction_since = None
                self.correction_direction = 0

            if self.permission(now) == 2:
                if self.arc_since is None:
                    self.arc_since = now

                value = self.voltage()
                correction = 0

                if (
                    value is not None
                    and self.p['voltage_tracking_enabled']
                ):
                    if value > self.high:
                        correction = self.p['voltage_high_direction']
                    elif value < self.low:
                        correction = -self.p['voltage_high_direction']

                if correction != self.correction_direction:
                    self.correction_direction = correction
                    self.correction_since = now if correction else None

        self.wake.set()

    def submit(self, operation, args=()):
        with self.lock:
            if not self.configured:
                return None, 'Serial session not configured'

            if self.pending is not None and operation != 'STOP':
                return None, 'Another command is awaiting ACK'

            self.next_id += 1
            command = dict(
                id=self.next_id,
                op=operation,
                args=args,
                event=threading.Event(),
                answer=None,
                first=0.0,
                last=0.0,
            )

            if operation == 'STOP':
                self.tracking_authorized = False

                if self.pending is not None:
                    self.complete(self.pending, False, 'Preempted by STOP')
                if self.urgent is not None:
                    self.complete(self.urgent, False, 'Superseded by STOP')

                self.pending = None
                self.urgent = command

            else:
                if self.urgent is not None:
                    return None, 'STOP is pending'
                self.pending = command

        self.wake.set()
        return command, ''

    @staticmethod
    def complete(command, ok, message):
        command['answer'] = (
            ok, f"id={command['id']} {message}"
        )
        command['event'].set()

    def request(self, operation, args=()):
        command, error = self.submit(operation, args)
        if command is None:
            return False, error

        if not command['event'].wait(self.p['command_timeout_s'] + 0.5):
            with self.lock:
                if self.pending is command:
                    self.pending = None
                if self.urgent is command:
                    self.urgent = None

            self.submit('STOP')
            return (
                False,
                'Worker deadline exceeded; STOP requested; inspect status',
            )

        return command['answer']

    def callback(self, operation, args):
        def call(request, response):
            response.success, response.message = self.request(operation, args)
            return response
        return call

    def jog(self, request, response):
        if (
            not self.p['calibration_mode']
            or not 1 <= abs(request.steps) <= self.p['jog_max_steps']
        ):
            response.success = False
            response.message = 'Calibration-only bounded jog'
            return response

        response.success, response.message = self.request(
            'JOG', (int(request.steps),)
        )
        return response

    def receive(self, fields, now):
        # Cache updates only. ROS publication runs in publish_cached().
        tag = fields[0]

        with self.lock:
            self.last_frame_at = now

            if (
                tag == 'ACK'
                and len(fields) == 5
                and int(fields[1]) == self.session
            ):
                key = int(fields[2])
                ok = fields[3] == '1'

                if key == 0:
                    self.configured = ok
                    if not ok:
                        self.stats['error'] = 'CONFIG: ' + fields[4]
                    if not ok and fields[4] == 'BUSY_STOP_FIRST':
                        self.takeover_stop = True

                for slot in ('urgent', 'pending'):
                    command = getattr(self, slot)
                    if command is not None and command['id'] == key:
                        self.stats['last_ack_ms'] = (
                            now - command['first']
                        ) * 1000
                        self.complete(command, ok, fields[4])
                        setattr(self, slot, None)
                return

            if tag == 'H' and len(fields) == 15:
                sid = int(fields[1])

                if sid == 0 and self.configured:
                    raise serial.SerialException('Nano rebooted')
                if sid != self.session:
                    return

                self.h = [fields[2]] + list(map(int, fields[3:]))
                self.h_at = now
                self.height_revision += 1
                return

    def write(self, port, wire):
        started = time.monotonic()

        if port.write(wire) != len(wire):
            raise serial.SerialException('Short write')

        elapsed = (time.monotonic() - started) * 1000
        with self.lock:
            self.stats['max_write_ms'] = max(
                self.stats['max_write_ms'], elapsed
            )

    def io_loop(self):
        # Sole owner of ALL reads/writes for the height serial port.
        while not self.stop_event.is_set():
            try:
                with serial.Serial(
                    self.p['port'],
                    self.p['baud'],
                    timeout=0,
                    write_timeout=0.1,
                    exclusive=True,
                ) as port:
                    with self.lock:
                        self.session = secrets.randbelow(2147483646) + 1
                        sid = self.session
                        self.next_id = 0
                        self.configured = False
                        self.h = None
                        self.h_at = 0
                        self.tracking_authorized = False
                        self.arc_since = None
                        self.correction_since = None
                        self.correction_direction = 0
                        self.stats['reconnects'] += 1

                    buffer = bytearray()
                    seen = False
                    config_at = 0.0
                    heartbeat_at = 0.0
                    self.takeover_stop = False

                    while not self.stop_event.is_set():
                        data = port.read(min(port.in_waiting, 2048))
                        buffer.extend(data)

                        while b'\n' in buffer:
                            raw, _, rest = buffer.partition(b'\n')
                            buffer = bytearray(rest)

                            try:
                                fields = decode(raw)
                                if fields[0] == 'H':
                                    seen = True
                                self.receive(fields, time.monotonic())
                            except (ValueError, UnicodeError):
                                with self.lock:
                                    self.stats['bad_frames'] += 1

                        if len(buffer) > 512:
                            buffer.clear()
                            with self.lock:
                                self.stats['bad_frames'] += 1

                        now = time.monotonic()
                        with self.lock:
                            ready = self.configured
                            control = self.control(now)

                        # Same session for CONFIG retries; do not reopen
                        # USB just because a CONFIG ACK was lost.
                        if (
                            seen and not ready
                            and now - config_at >= 0.5
                        ):
                            if self.takeover_stop:
                                self.write(port, frame('C', sid, 1, 'STOP'))
                                self.takeover_stop = False

                            self.write(
                                port,
                                frame('C', sid, 0, 'CONFIG', *self.config),
                            )
                            config_at = now
                            with self.lock:
                                self.stats['config_sends'] += 1

                        if (
                            ready
                            and now - heartbeat_at
                            >= self.p['heartbeat_period_s']
                        ):
                            self.write(port, frame('P', sid, *control))

                            with self.lock:
                                if heartbeat_at:
                                    self.stats['max_heartbeat_gap_ms'] = max(
                                        self.stats['max_heartbeat_gap_ms'],
                                        (now - heartbeat_at) * 1000,
                                    )
                            heartbeat_at = now

                        send = None
                        permission_wire = None
                        timed_out = None

                        with self.lock:
                            command = self.urgent or self.pending

                            if ready and command is not None:
                                command_now = time.monotonic()
                                command_control = self.control(command_now)

                                if not command['first']:
                                    command['first'] = command_now

                                # Preserve the ignition handoff:
                                # don't transmit SEARCH with OFF/invalid
                                # permission merely because the ROS
                                # service request arrived first.
                                waiting_for_ignition = (
                                    command['op'] == 'SEARCH'
                                    and command['last'] == 0.0
                                    and command_control[0] not in (1, 2)
                                )

                                timeout = self.p['command_timeout_s']
                                if waiting_for_ignition:
                                    timeout = min(timeout, 0.75)

                                expired = (
                                    command_now - command['first'] >= timeout
                                )

                                if expired:
                                    timed_out = command
                                    if waiting_for_ignition:
                                        message = (
                                            'SEARCH ignition handoff timed out: '
                                            f'permission={command_control[0]}; '
                                            'SEARCH was not transmitted'
                                        )
                                    else:
                                        message = (
                                            'ACK timeout; execution uncertain; '
                                            'inspect status'
                                        )

                                    self.complete(command, False, message)
                                    if self.urgent is command:
                                        self.urgent = None
                                    if self.pending is command:
                                        self.pending = None

                                elif (
                                    not waiting_for_ignition
                                    and command_now - command['last']
                                    >= self.p['command_retry_s']
                                ):
                                    command['last'] = command_now

                                    # Capture permission and command together.
                                    permission_wire = frame(
                                        'P', sid, *command_control
                                    )
                                    send = frame(
                                        'C',
                                        sid,
                                        command['id'],
                                        command['op'],
                                        *command['args'],
                                    )

                        if (
                            timed_out is not None
                            and timed_out['op'] != 'STOP'
                        ):
                            self.submit('STOP')

                        if send is not None:
                            # Same worker: P immediately precedes C.
                            self.write(port, permission_wire)
                            self.write(port, send)

                        self.wake.wait(0.003)
                        self.wake.clear()

                    # Best-effort STOP on orderly bridge shutdown.
                    # The Nano's local link watchdog remains independent.
                    with self.lock:
                        self.next_id += 1
                        stop_id = self.next_id

                    self.write(port, frame('C', sid, stop_id, 'STOP'))

            except (serial.SerialException, OSError) as error:
                with self.lock:
                    self.stats['error'] = str(error)

            finally:
                with self.lock:
                    self.configured = False
                    self.h_at = 0
                    self.h = None
                    self.tracking_authorized = False

                    for slot in ('pending', 'urgent'):
                        command = getattr(self, slot)
                        if command is not None:
                            self.complete(
                                command, False,
                                'Disconnected; completion unknown; re-home',
                            )
                        setattr(self, slot, None)

                self.stop_event.wait(0.5)

    def publish_cached(self):
        now = time.monotonic()

        with self.lock:
            height = list(self.h) if self.h is not None else None
            session = self.session
            configured = self.configured
            revision = self.height_revision

            fresh = (
                self.configured
                and height is not None
                and now - self.h_at < self.p['status_timeout_s']
            )
            mode, qualified, _, _ = self.control(now)
            voltage = self.voltage()

            stats = dict(
                self.stats,
                rx_age_s=(
                    now - self.last_frame_at
                    if self.last_frame_at else None
                ),
                configured=self.configured,
                calibration=self.p['calibration_mode'],
            )
            stats.update(
                torch_permission=mode,
                arc_qualified=bool(qualified),

                torch_age_ms=(
                    (now - self.torch_at) * 1000
                    if self.torch is not None else None
                ),

                arc_continuous_ms=(
                    (now - self.arc_since) * 1000
                    if self.arc_since is not None else 0.0
                ),

                transfer_detected=(
                    bool(self.torch.transfer_detected)
                    if self.torch is not None else None
                ),

                voltage_value=voltage,

                tracking_authorized=self.tracking_authorized,
            )

        # H cache:
        # 0 state, 1 position, 2 switch, 3 fault, 4 heartbeat age,
        # 5 active ID, 6 completed ID, 7 result,
        # 8/9 reserved zeros, 10 RX errors, 11 referenced,
        # 12 tracking authorized.
        control_fresh = (
            fresh and height[4] <= self.p['link_timeout_ms']
        )

        if control_fresh:
            full_status = ','.join(map(str, (
                height[0],
                height[1],
                height[2],
                height[3],
                height[11],
                session,
                height[5],
                height[6],
                height[7],
                int(self.p['calibration_mode']),
                int(configured),
                height[12],
            )))
        else:
            full_status = 'OFFLINE,0,0,2,0,0,0,0,0,1,0,0'

        # Do not repeatedly publish a cached healthy status as though
        # it were a newly received Nano snapshot.
        key = (session, revision)
        if not control_fresh or key != self.published_height:
            self.published_height = key
            self.pub['torch_height/status_full'].publish(
                String(data=full_status)
            )

        established = bool(
            control_fresh
            and qualified
            and height[0] in ('HOLD', 'TRACK')
            and height[3] == 0
        )

        self.pub['torch_height/serial_online'].publish(
            Bool(data=bool(fresh))
        )
        self.pub['torch_height/status'].publish(
            String(
                data=(
                    ','.join(map(str, height[:4]))
                    if fresh else 'OFFLINE,0,0,2'
                )
            )
        )
        self.pub['torch_height/at_home'].publish(
            Bool(data=bool(
                control_fresh
                and height[0] == 'HOME'
                and height[2]
                and height[11] == 1
                and height[1] == 0
                and not height[3]
            ))
        )
        self.pub['torch_height/arc_established'].publish(
            Bool(data=established)
        )
        self.pub['torch_height/voltage_in_band'].publish(
            Bool(data=bool(
                established
                and voltage is not None
                and self.low <= voltage <= self.high
            ))
        )

        if fresh:
            stats.update(
                nano_heartbeat_age_ms=height[4],
                active_id=height[5],
                nano_rx_errors=height[10],
            )

            done = (session, height[6], height[7])
            if height[6] and done != self.last_done:
                self.last_done = done
                event = dict(
                    session=done[0],
                    id=height[6],
                    result=(
                        'completed' if height[7] == 1 else 'aborted'
                    ),
                    position_steps=height[1],
                    fault=height[3],
                )
                self.pub['torch_height/motion_event'].publish(
                    String(data=json.dumps(event))
                )

        if now - self.last_diagnostic >= 1:
            self.last_diagnostic = now
            self.pub['torch_height/diagnostics'].publish(
                String(data=json.dumps(stats))
            )

    def destroy_node(self):
        self.stop_event.set()
        self.wake.set()
        self.worker.join(timeout=3)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TorchHeightBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()