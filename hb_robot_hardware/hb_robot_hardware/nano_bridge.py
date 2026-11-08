"""ROS 2 Humble serial bridge. Protocol documented in README.md."""
import math
import threading
import time
import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, UInt32
from std_srvs.srv import SetBool


def xor_checksum(payload):
    value = 0
    for byte in payload.encode('ascii'):
        value ^= byte
    return f'{value:02X}'


class NanoBridge(Node):
    def __init__(self):
        super().__init__('nano_bridge')
        self.port = self.declare_parameter('port', '/dev/serial/by-id/CHANGE_ME').value
        self.baud = int(self.declare_parameter('baud', 230400).value)
        self.frame = self.declare_parameter('frame_id', 'awp_mpu6050').value
        self.max_age = float(self.declare_parameter('max_sample_age_s', 0.25).value)
        self.timeout = float(self.declare_parameter('command_timeout_s', 1.0).value)
        self.open_deg = int(self.declare_parameter('flap_open_deg', 0).value)
        self.closed_deg = int(self.declare_parameter('flap_closed_deg', 90).value)
        self.accel_var = float(self.declare_parameter('accel_variance_m2_s4', 0.04).value)
        self.pub = self.create_publisher(Imu, 'mpu6050/imu', qos_profile_sensor_data)
        self.rate_pub = self.create_publisher(Float32, 'mpu6050/sample_rate_hz', 10)
        self.seq_pub = self.create_publisher(UInt32, 'mpu6050/sequence', 10)
        self.online_pub = self.create_publisher(Bool, 'hardware/online', 10)
        self.flap_pub = self.create_publisher(Bool, 'camera_flap/closed', 10)
        self.ser = None
        self.lock = threading.Lock()
        self.ack_cv = threading.Condition()
        self.acks = {}
        self.command_id = 0
        self.last_rx = 0.0
        self.last_seq = None
        self.last_device_us = None
        self.rate_ema = None
        self.running = True
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()
        self.create_service(SetBool, 'camera_flap/set_closed', self.set_flap)
        self.create_timer(0.5, self.publish_health)

    def publish_health(self):
        msg = Bool()
        msg.data = (time.monotonic() - self.last_rx) < self.max_age
        self.online_pub.publish(msg)

    def send(self, fields):
        payload = ','.join(map(str, fields))
        wire = f'${payload}*{xor_checksum(payload)}\n'.encode('ascii')
        with self.lock:
            if self.ser is None:
                return False
            try:
                self.ser.write(wire)
                return True
            except (serial.SerialException, OSError):
                return False

    def set_flap(self, request, response):
        # This service confirms firmware acknowledgement, NOT physical flap position.
        angle = self.closed_deg if request.data else self.open_deg
        if not (0 <= angle <= 180):
            response.success = False
            response.message = 'Invalid configured servo angle'
            return response
        with self.ack_cv:
            self.command_id += 1
            cid = self.command_id
        if not self.send(['FLAP', cid, angle]):
            response.success = False
            response.message = 'Serial offline'
            return response
        deadline = time.monotonic() + self.timeout
        with self.ack_cv:
            while cid not in self.acks:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self.ack_cv.wait(timeout=remaining)
            accepted = self.acks.pop(cid, None)
        response.success = accepted == angle
        response.message = ('Command acknowledged (position not verified)' if response.success
                            else 'No matching firmware acknowledgement')
        return response

    def handle_line(self, raw):
        if not raw.startswith('$') or '*' not in raw:
            return
        payload, check = raw[1:].rsplit('*', 1)
        if xor_checksum(payload) != check.upper():
            return
        f = payload.split(',')
        if f[0] == 'ACK' and len(f) == 3:
            try:
                cid, angle = int(f[1]), int(f[2])
            except ValueError:
                return
            with self.ack_cv:
                self.acks[cid] = angle
                self.ack_cv.notify_all()
            closed = Bool()
            closed.data = (angle == self.closed_deg)
            self.flap_pub.publish(closed)
            return
        if f[0] == 'ERROR' and len(f) == 2:
            self.get_logger().error(f'Nano reports {f[1]}')
            return
        if f[0] != 'M' or len(f) != 9:
            return
        try:
            seq, us, x, y, z, gx, gy, gz = map(int, f[1:])
        except ValueError:
            return
        if not (0 <= seq <= 0xffffffff and 0 <= us <= 0xffffffff):
            return
        if any(v < -32768 or v > 32767 for v in (x, y, z, gx, gy, gz)):
            return
        if self.last_seq is not None and ((seq - self.last_seq) & 0xFFFFFFFF) != 1:
            self.get_logger().warn('MPU6050 sample sequence gap', throttle_duration_sec=5.0)
        self.last_seq = seq
        self.last_rx = time.monotonic()
        # Host receipt time is approximate; device micros is used for rate measurement.
        if self.last_device_us is not None:
            dt_us = (us - self.last_device_us) & 0xFFFFFFFF
            if 100 < dt_us < 1_000_000:
                hz = 1_000_000.0 / dt_us
                self.rate_ema = hz if self.rate_ema is None else 0.95 * self.rate_ema + 0.05 * hz
        self.last_device_us = us
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.orientation_covariance[0] = -1.0
        # With no measured gyro covariance, ROS Imu uses the default zero array.
        gyro_scale = math.radians(1.0 / 131.0)
        msg.angular_velocity.x = gx * gyro_scale
        msg.angular_velocity.y = gy * gyro_scale
        msg.angular_velocity.z = gz * gyro_scale
        scale = 9.80665 / 16384.0
        msg.linear_acceleration.x = x * scale
        msg.linear_acceleration.y = y * scale
        msg.linear_acceleration.z = z * scale
        msg.linear_acceleration_covariance = [self.accel_var, 0.0, 0.0,
                                               0.0, self.accel_var, 0.0,
                                               0.0, 0.0, self.accel_var]
        self.pub.publish(msg)
        q = UInt32()
        q.data = seq
        self.seq_pub.publish(q)
        if self.rate_ema is not None:
            r = Float32()
            r.data = float(self.rate_ema)
            self.rate_pub.publish(r)

    def read_loop(self):
        while self.running and rclpy.ok():
            try:
                with serial.Serial(self.port, self.baud, timeout=0.15, write_timeout=0.5) as ser:
                    with self.lock:
                        self.ser = ser
                    self.get_logger().info(f'Connected to {self.port}')
                    # Nano Every can reset when serial opens; allow bootloader time.
                    time.sleep(2.0)
                    while self.running and rclpy.ok():
                        line = ser.readline()
                        if line and len(line) <= 160:
                            try:
                                self.handle_line(line.decode('ascii').strip())
                            except UnicodeDecodeError:
                                pass
            except (serial.SerialException, OSError) as exc:
                self.get_logger().warn(f'Serial unavailable: {exc}', throttle_duration_sec=5.0)
            finally:
                with self.lock:
                    self.ser = None
                self.last_rx = 0.0
            time.sleep(1.0)

    def destroy_node(self):
        self.running = False
        with self.lock:
            if self.ser:
                self.ser.close()
                self.ser = None
        self.reader.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NanoBridge()
    # MultiThreadedExecutor allows service to await acknowledgement from reader thread.
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
