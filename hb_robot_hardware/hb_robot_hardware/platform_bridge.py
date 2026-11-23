#!/usr/bin/env python3
"""Two MPU6050 streams and the camera-flap servo.

No TorchStatus subscription or height-control dependency.
Servo ACK/status report commanded angle, not measured position.
All parameters are startup-only.
"""

import json
import math
import secrets
import threading
import time
from collections import deque

import serial
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, UInt32, String
from std_srvs.srv import SetBool


def frame(*fields):
    payload = ','.join(map(str, fields)).encode('ascii')
    check = 0
    for byte in payload:
        check ^= byte
    return b'$' + payload + f'*{check:02X}\n'.encode('ascii')


def decode(raw):
    payload, check = raw.strip().rsplit(b'*', 1)
    if not payload.startswith(b'$') or len(check) != 2:
        raise ValueError('Bad framing')
    value = 0
    for byte in payload[1:]:
        value ^= byte
    if value != int(check, 16):
        raise ValueError('Bad checksum')
    return payload[1:].decode('ascii').split(',')


class PlatformBridge(Node):
    def __init__(self):
        super().__init__('platform_bridge')

        defaults = {
            'port': '/dev/ttyACM0',
            'baud': 230400,
            'frame_id': 'awp_mpu6050',
            'imu2_frame_id': 'awp_mpu6050_2',
            'accel_variance_m2_s4': 0.04,
            'imu2_accel_variance_m2_s4': 0.04,
            'max_sample_age_s': 0.25,
            'flap_open_deg': 45,
            'flap_closed_deg': 5,
            'command_timeout_s': 2.0,
            'command_retry_s': 0.25,
        }

        desc = ParameterDescriptor(read_only=True)
        self.p = {
            k: self.declare_parameter(k, v, desc).value
            for k, v in defaults.items()
        }

        for k in (
            'max_sample_age_s',
            'command_timeout_s',
            'command_retry_s',
        ):
            if not math.isfinite(self.p[k]) or self.p[k] <= 0:
                raise ValueError(f'{k} must be positive and finite')

        if self.p['command_retry_s'] >= self.p['command_timeout_s']:
            raise ValueError(
                'command_retry_s must be below command_timeout_s'
            )

        for k in ('flap_open_deg', 'flap_closed_deg'):
            if not 0 <= self.p[k] <= 180:
                raise ValueError(f'{k} must be in 0..180')

        if self.p['flap_open_deg'] == self.p['flap_closed_deg']:
            raise ValueError('Flap open and closed angles must differ')

        for k in ('accel_variance_m2_s4', 'imu2_accel_variance_m2_s4'):
            if not math.isfinite(self.p[k]) or self.p[k] < 0:
                raise ValueError(f'{k} must be finite and nonnegative')

        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.pending = None
        self.next_id = secrets.randbelow(1000000000) + 1

        self.status = None
        self.status_at = 0.0
        self.queues = {tag: deque(maxlen=8) for tag in ('M', 'M2')}
        self.last_rx = {'M': 0.0, 'M2': 0.0}
        self.previous_us = {'M': None, 'M2': None}

        self.stats = dict(
            bad_frames=0,
            imu_queue_drops=0,
            connections=0,
            error='',
        )
        self.last_diagnostic = 0.0
        self.channels = {}

        for tag, prefix, key, variance in (
            ('M', 'mpu6050', 'frame_id', 'accel_variance_m2_s4'),
            ('M2', 'mpu6050_2', 'imu2_frame_id', 'imu2_accel_variance_m2_s4'),
        ):
            self.channels[tag] = dict(
                frame=self.p[key],
                variance=self.p[variance],
                imu=self.create_publisher(
                    Imu, prefix + '/imu', qos_profile_sensor_data
                ),
                seq=self.create_publisher(UInt32, prefix + '/sequence', 10),
                rate=self.create_publisher(
                    Float32, prefix + '/sample_rate_hz', 10
                ),
                online=self.create_publisher(Bool, prefix + '/online', 10),
            )

        # Retains the existing primary-IMU online topic.
        self.online_pub = self.create_publisher(Bool, 'hardware/online', 10)
        self.serial_pub = self.create_publisher(
            Bool, 'hardware/serial_online', 10
        )
        self.flap_pub = self.create_publisher(Bool, 'camera_flap/closed', 10)
        self.diag_pub = self.create_publisher(String, 'hardware/diagnostics', 10)

        self.service_group = MutuallyExclusiveCallbackGroup()
        self.create_service(
            SetBool,
            'camera_flap/set_closed',
            self.set_flap,
            callback_group=self.service_group,
        )

        self.create_timer(0.02, self.publish_cached)
        self.worker = threading.Thread(target=self.io_loop, daemon=True)
        self.worker.start()

    @staticmethod
    def complete(command, ok, message):
        command['answer'] = (ok, message)
        command['event'].set()

    def set_flap(self, request, response):
        angle = (
            self.p['flap_closed_deg']
            if request.data else self.p['flap_open_deg']
        )

        with self.lock:
            if (
                self.status is None
                or time.monotonic() - self.status_at > 2.0
            ):
                response.success = False
                response.message = (
                    'Nano offline or awaiting servo/IMU firmware status'
                )
                return response

            if self.pending is not None:
                response.success = False
                response.message = 'Another flap command is pending'
                return response

            self.next_id = self.next_id % 2147483647 + 1
            command = dict(
                id=self.next_id,
                angle=angle,
                last=0.0,
                deadline=time.monotonic() + self.p['command_timeout_s'],
                event=threading.Event(),
                answer=None,
            )
            self.pending = command

        command['event'].wait(self.p['command_timeout_s'] + 0.2)

        with self.lock:
            if command['answer'] is None:
                if self.pending is command:
                    self.pending = None
                self.complete(
                    command, False,
                    'Flap ACK timeout; commanded position uncertain'
                )
            response.success, response.message = command['answer']

        return response

    def receive(self, fields, now):
        with self.lock:
            if fields[0] == 'ACK' and len(fields) == 3:
                key, angle = map(int, fields[1:])
                command = self.pending

                if (
                    command is not None
                    and key == command['id']
                    and angle == command['angle']
                ):
                    self.complete(
                        command, True,
                        f'id={key} FLAP_COMMAND_ACCEPTED angle={angle}'
                    )
                    self.pending = None

            elif fields[0] == 'S' and len(fields) == 8:
                status = tuple(map(int, fields[1:]))
                if status[0] != 1 or not 0 <= status[2] <= 180:
                    raise ValueError('Unsupported status')
                self.status = status
                self.status_at = now

            elif fields[0] in self.queues and len(fields) == 9:
                data = tuple(map(int, fields[1:]))

                if not all(0 <= v <= 0xffffffff for v in data[:2]):
                    raise ValueError('Bad sensor counters')
                if not all(-32768 <= v <= 32767 for v in data[2:]):
                    raise ValueError('Bad sensor sample')

                tag = fields[0]
                queue = self.queues[tag]
                if len(queue) == queue.maxlen:
                    self.stats['imu_queue_drops'] += 1

                queue.append((now, data))
                self.last_rx[tag] = now

    def io_loop(self):
        # This worker exclusively owns this serial port.
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
                        self.stats['connections'] += 1

                    buffer = bytearray()

                    while not self.stop_event.is_set():
                        buffer.extend(port.read(min(port.in_waiting, 4096)))

                        while b'\n' in buffer:
                            raw, _, tail = buffer.partition(b'\n')
                            buffer = bytearray(tail)
                            try:
                                self.receive(decode(raw), time.monotonic())
                            except (ValueError, UnicodeError):
                                with self.lock:
                                    self.stats['bad_frames'] += 1

                        if len(buffer) > 512:
                            buffer.clear()
                            with self.lock:
                                self.stats['bad_frames'] += 1

                        wire = None
                        now = time.monotonic()

                        with self.lock:
                            command = self.pending
                            if command is not None:
                                if now >= command['deadline']:
                                    self.complete(
                                        command, False,
                                        'Flap ACK timeout; commanded position uncertain'
                                    )
                                    self.pending = None
                                elif (
                                    now - command['last']
                                    >= self.p['command_retry_s']
                                ):
                                    command['last'] = now
                                    wire = frame(
                                        'FLAP', command['id'], command['angle']
                                    )

                        if wire is not None:
                            if port.write(wire) != len(wire):
                                raise serial.SerialException('Short serial write')

                        self.stop_event.wait(0.003)

            except (serial.SerialException, OSError) as error:
                with self.lock:
                    self.stats['error'] = str(error)

            finally:
                with self.lock:
                    self.status = None
                    self.status_at = 0.0
                    for tag in self.queues:
                        self.queues[tag].clear()
                        self.last_rx[tag] = 0.0

                    if self.pending is not None:
                        self.complete(
                            self.pending, False,
                            'Serial disconnected; flap position uncertain'
                        )
                        self.pending = None

                self.stop_event.wait(0.5)

    def publish_cached(self):
        now = time.monotonic()

        with self.lock:
            status = self.status
            serial_online = (
                status is not None and now - self.status_at < 2.0
            )
            items = {tag: list(q) for tag, q in self.queues.items()}
            for q in self.queues.values():
                q.clear()
            ages = {tag: now - at for tag, at in self.last_rx.items()}
            diagnostics = dict(self.stats, serial_online=serial_online)

        self.serial_pub.publish(Bool(data=serial_online))

        if serial_online:
            self.flap_pub.publish(
                Bool(data=status[2] == self.p['flap_closed_deg'])
            )
            diagnostics.update(
                uptime_ms=status[1],
                servo_angle=status[2],
                imu1_ready=status[3],
                imu2_ready=status[4],
                nano_imu_drops=status[5],
                nano_i2c_errors=status[6],
            )

        for tag, samples in items.items():
            channel = self.channels[tag]
            online = ages[tag] < self.p['max_sample_age_s']
            channel['online'].publish(Bool(data=online))

            if tag == 'M':
                self.online_pub.publish(Bool(data=online))

            if not online:
                self.previous_us[tag] = None

            for received, data in samples:
                if now - received > self.p['max_sample_age_s']:
                    continue

                sequence, device_us, *raw = data
                msg = Imu()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = channel['frame']
                msg.orientation_covariance[0] = -1.0

                for axis, a, g in zip('xyz', raw[:3], raw[3:]):
                    setattr(
                        msg.linear_acceleration,
                        axis,
                        a * 9.80665 / 16384.0,
                    )
                    setattr(
                        msg.angular_velocity,
                        axis,
                        math.radians(g / 131.0),
                    )

                for i in (0, 4, 8):
                    msg.linear_acceleration_covariance[i] = channel['variance']

                channel['imu'].publish(msg)
                channel['seq'].publish(UInt32(data=sequence))

                previous = self.previous_us[tag]
                if previous is not None:
                    delta = (device_us - previous) & 0xffffffff
                    if 100 < delta < 1000000:
                        channel['rate'].publish(Float32(data=1e6 / delta))

                self.previous_us[tag] = device_us

        if now - self.last_diagnostic >= 1.0:
            self.last_diagnostic = now
            self.diag_pub.publish(String(data=json.dumps(diagnostics)))

    def destroy_node(self):
        self.stop_event.set()
        self.worker.join(timeout=3.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PlatformBridge()
    executor = MultiThreadedExecutor(num_threads=2)
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