"""Exercise driver recovery and ROS output using a test-only simulated SDK device."""
import os
import signal
import struct
import subprocess
import sys
import tempfile
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from azure_kinect_ros2_driver_codex.msg import RGBDFrame
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


def run_case(executable, fake_sdk, mode):
    env = dict(os.environ, LD_PRELOAD=fake_sdk,
               K4A_TEST_FAULT='corrupt' if mode == 'no_color_demand' else mode)
    node = rclpy.create_node('pipeline_regression_' + mode)
    depths, colors, diagnostics, compressed, infos = [], [], [], [], []
    subscriptions = [
        node.create_subscription(Image, '/k4a/depth/image_raw', depths.append, qos_profile_sensor_data),
        node.create_subscription(DiagnosticArray, '/k4a_ros2_node_codex/diagnostics', diagnostics.append, 10),
    ]
    pairs = []
    if mode in ('rgbd', 'corrupt', 'stale'):
        subscriptions.append(node.create_subscription(
            RGBDFrame, '/k4a/rgbd', pairs.append, qos_profile_sensor_data))
    if mode == 'jpeg':
        subscriptions.append(node.create_subscription(
            CompressedImage, '/rgb/image_raw/compressed', colors.append, qos_profile_sensor_data))
    elif mode not in ('no_color_demand', 'native_only', 'info_only', 'rgbd'):
        subscriptions.append(node.create_subscription(
            Image, '/k4a/rgb/image_raw', colors.append, qos_profile_sensor_data))
    if mode in ('native_only', 'native_both'):
        subscriptions.append(node.create_subscription(
            CompressedImage, '/k4a/rgb/image_raw/compressed', compressed.append, qos_profile_sensor_data))
    if mode == 'info_only':
        subscriptions.append(node.create_subscription(
            CameraInfo, '/k4a/rgb/camera_info', infos.append, qos_profile_sensor_data))
    with tempfile.TemporaryFile(mode='w+') as output:
        command = [
            executable, '--ros-args', '-p', 'capture_timeout_ms:=200',
            '-p', 'recovery_backoff_ms:=20', '-p', 'recovery_max_attempts:=2',
        ]
        if mode == 'jpeg':
            command += ['-p', 'color_format:=jpeg']
        if mode == 'plugin_transport':
            command += ['-p', 'native_mjpeg_transport:=false']
        process = subprocess.Popen(command, env=env, stdout=output, stderr=subprocess.STDOUT)
        try:
            # Allow dynamic loading and DDS discovery before counting observation time.
            startup_deadline = time.monotonic() + 15
            while not diagnostics and time.monotonic() < startup_deadline and process.poll() is None:
                rclpy.spin_once(node, timeout_sec=0.05)
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline and process.poll() is None:
                rclpy.spin_once(node, timeout_sec=0.05)
            if mode == 'startup_failure':
                assert process.poll() == 1, 'Startup failure did not exit cleanly'
            else:
                assert process.poll() is None, 'Node died during fault injection'
                assert diagnostics, f'No health diagnostics; discovered nodes: {node.get_node_names()}'
                status = diagnostics[-1].status[0]
                counts = {entry.key: int(entry.value) for entry in status.values}
                if mode in ('always_fail', 'timeout'):
                    level = int.from_bytes(status.level, 'little') if isinstance(status.level, bytes) else status.level
                    assert level == 2, status.message
                    assert counts['stream_restarts'] == 2, counts
                    assert not depths and not colors
                elif mode == 'stale':
                    assert not depths and not colors and not pairs, 'Stale sensor data escaped age gate'
                    assert counts['stale_output_drops'] > 0
                elif mode == 'corrupt':
                    assert depths and not colors and not pairs, 'Corrupt color should not stop depth or publish old RGB'
                    assert counts['color_decode_errors'] > 0
                    assert counts['stream_restarts'] == 0
                elif mode == 'rgbd':
                    assert len(pairs) > 1, 'No consecutive RGBD observations'
                    for pair in pairs:
                        assert pair.header == pair.rgb.header == pair.camera_info.header
                        assert pair.depth.header.frame_id == pair.header.frame_id
                        assert pair.rgb.encoding == 'bgra8' and pair.depth.encoding == '32FC1'
                        for image in (pair.rgb, pair.depth):
                            assert image.width == pair.camera_info.width == 16
                            assert image.height == pair.camera_info.height == 8
                            assert len(image.data) == image.step * image.height
                        assert struct.unpack('<f', bytes(pair.depth.data[:4]))[0] == 1.0
                        def ns(stamp):
                            return stamp.sec * 10**9 + stamp.nanosec
                        assert ns(pair.rgb.header.stamp) - ns(pair.depth.header.stamp) == 500000
                elif mode == 'recover':
                    assert depths and colors, 'No image output after recovery'
                    assert counts['stream_restarts'] == 1
                    assert colors[-1].encoding == 'bgra8'
                elif mode == 'no_color_demand':
                    assert depths and not colors
                    assert counts['color_decode_errors'] == 0, 'Unused color was decoded'
                elif mode == 'jpeg':
                    assert depths and colors
                    assert bytes(colors[-1].data[:2]) == b'\xff\xd8'
                elif mode == 'native_only':
                    assert compressed and not colors
                    assert counts['direct_color_decodes'] == 0
                    assert counts['native_jpeg_frames'] > 0
                elif mode == 'native_both':
                    assert compressed and colors
                    assert counts['direct_color_decodes'] > 0
                    stamp = lambda msg: msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
                    assert set(map(stamp, compressed)) & set(map(stamp, colors)), 'RGB/JPEG timestamps differ'
                elif mode == 'info_only':
                    assert infos and not colors
                    assert counts['direct_color_decodes'] == 0
                elif mode == 'plugin_transport':
                    assert colors and counts['direct_color_decodes'] > 0
                if depths:
                    assert depths[-1].encoding == '32FC1'
                    assert struct.unpack('<f', bytes(depths[-1].data[:4]))[0] == 1.0
                    stamps = [msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec for msg in depths]
                    assert all(b > a for a, b in zip(stamps, stamps[1:])), 'Non-monotonic depth timestamps'
        except BaseException:
            output.seek(0)
            print(output.read())
            raise
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
                    raise AssertionError('Driver shutdown hung')
            node.destroy_node()
        assert process.returncode == (1 if mode == 'startup_failure' else 0), process.returncode
    print('PASS:', mode, flush=True)


if __name__ == '__main__':
    # Avoid discovering or communicating with production robot nodes.
    os.environ['ROS_DOMAIN_ID'] = '197'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    rclpy.init()
    try:
        for fault in ('recover', 'corrupt', 'always_fail', 'timeout', 'stale',
                      'no_color_demand', 'jpeg', 'startup_failure',
                      'native_only', 'native_both', 'info_only', 'plugin_transport', 'rgbd'):
            run_case(sys.argv[1], sys.argv[2], fault)
    finally:
        rclpy.shutdown()
