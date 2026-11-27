#!/usr/bin/env python3
"""Passive two-MPU tilt diagnostics. Never publishes TF or stability permission."""
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
from hb_robot_awp.msg import TiltSample
from attitude_filter import AttitudeFilter, rotate, rpy


class AttitudeNode(Node):
    def __init__(self):
        super().__init__('awp_attitude')
        gain = self.declare_parameter('correction_gain', 1.0).value
        calibration = self.declare_parameter('calibration_s', 2.0).value
        self.max_age = self.declare_parameter('max_sample_age_s', 0.3).value
        if not math.isfinite(self.max_age) or not 0 < self.max_age <= 1:
            raise ValueError('Invalid max_sample_age_s')
        self.channels = []
        for name, default in [('imu_1', '/mpu6050/imu'), ('imu_2', '/mpu6050_2/imu')]:
            topic = self.declare_parameter(name+'_topic', default).value
            xyzw = self.declare_parameter(name+'_base_from_sensor_xyzw', [0., 0., 0., 1.]).value
            q = np.asarray([xyzw[3], *xyzw[:3]], dtype=float) if len(xyzw) == 4 else np.zeros(4)
            if not np.isfinite(q).all() or abs(np.linalg.norm(q)-1) > 1e-3:
                raise ValueError('Mount quaternion must be finite and unit length')
            channel = dict(name=name, filter=AttitudeFilter(gain, calibration), mount=q,
                           rx=None, source_age=0., stamp=None, count=0)
            channel['pub'] = self.create_publisher(TiltSample, '~/'+name, qos_profile_sensor_data)
            channel['sub'] = self.create_subscription(Imu, topic, lambda m, c=channel: self.receive(c, m), qos_profile_sensor_data)
            self.channels.append(channel)
        self.create_service(Trigger, '~/zero', self.zero)
        self.create_timer(0.05, self.publish)

    def receive(self, c, m):
        stamp = m.header.stamp.sec + 1e-9*m.header.stamp.nanosec
        age = self.get_clock().now().nanoseconds*1e-9 - stamp
        f = c['filter']
        if not math.isfinite(age) or not -0.005 <= age <= self.max_age:
            f.valid = False; f.status = 'Source timestamp stale or future'; return
        a = m.linear_acceleration; g = m.angular_velocity
        c['rx'], c['source_age'], c['stamp'] = time.monotonic(), max(0., age), m.header.stamp
        c['count'] += 1
        f.update(stamp, rotate(c['mount'], [a.x, a.y, a.z]), rotate(c['mount'], [g.x, g.y, g.z]))

    def fresh(self, c):
        return c['rx'] is not None and c['source_age']+time.monotonic()-c['rx'] <= self.max_age

    def zero(self, request, response):
        response.success = all(c['filter'].valid and self.fresh(c) for c in self.channels)
        if response.success:
            for c in self.channels: c['filter'].zero()
        response.message = 'Both relative tilt references set' if response.success else 'Both channels must be valid and fresh'
        return response

    def publish(self):
        for c in self.channels:
            f = c['filter']; m = TiltSample()
            if c['stamp'] is not None: m.header.stamp = c['stamp']  # preserve actual sample age
            m.header.frame_id = 'robot_base_attitude_measurement'
            m.calibration_generation = f.calibration_generation
            m.sensor = c['name']; m.samples = c['count']; m.valid = f.valid and self.fresh(c)
            m.calibrated = f.calibrated; m.gravity_used = f.gravity_used
            m.source_age_ms = 1000*(c['source_age']+time.monotonic()-c['rx']) if c['rx'] is not None else float('inf')
            m.dt_s = f.dt; m.status = f.status if self.fresh(c) else 'Raw sample missing/stale'
            m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = map(float, f.q)
            for target, values in [(m.rpy_rad, rpy(f.q)), (m.delta_rpy_rad, f.delta()), (m.gyro_bias_rad_s, f.bias)]:
                target.x, target.y, target.z = map(float, values)
            c['pub'].publish(m)


def main():
    rclpy.init(); node = AttitudeNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
