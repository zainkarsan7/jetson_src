"""ROS 2 adapter. Robot TF is queried at acquisition time, never at Time(0)."""
from collections import deque
import json
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException

from .calibration import Sample, check_capture, solve
from .geometry import from_quaternion, quaternion, transform
from .storage import complete_result, export_result, load_session, save_session
from .target import BoardSpec, Target, camera_model


def tf_matrix(message):
    t, q = message.transform.translation, message.transform.rotation
    return from_quaternion([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])


class CalibrationNode(Node):
    def __init__(self, **kwargs):
        super().__init__('handeye', **kwargs)
        defaults = {
            'mode': 'eye_in_hand', 'base_frame': 'base_link', 'effector_frame': 'tool0',
            'camera_frame': 'camera_color_optical_frame', 'mount_frame': '',
            'image_topic': '/camera/color/image_raw', 'camera_info_topic': '/camera/color/camera_info',
            'image_is_rectified': False, 'output_directory': '~/.ros/handeye/session',
            'solver': 'park', 'min_samples': 5, 'max_sample_age_s': 1.0,
            'settle_time_s': 0.5, 'stability_translation_m': 0.002,
            'stability_rotation_deg': 0.5, 'max_reprojection_error_px': 2.0,
            'max_translation_rms_m': 0.01, 'max_rotation_rms_deg': 2.0,
            'detector_rate_hz': 10.0, 'board.kind': 'charuco',
            'board.dictionary': 'DICT_5X5_250', 'board.columns': 5, 'board.rows': 7,
            'board.square_length_m': 0.04, 'board.marker_length_m': 0.03,
            'board.marker_separation_m': 0.01,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value, ParameterDescriptor(read_only=True))
        self.cfg = {k: self.get_parameter(k).value for k in defaults}
        if self.cfg['mode'] not in ('eye_in_hand', 'eye_to_hand'):
            raise ValueError('Invalid calibration mode')
        for key in ('base_frame', 'effector_frame', 'camera_frame'):
            if not self.cfg[key] or self.cfg[key].startswith('/'):
                raise ValueError(f'{key} must be a nonempty TF name without a leading slash')
        if len({self.cfg[k] for k in ('base_frame', 'effector_frame', 'camera_frame')}) != 3:
            raise ValueError('Base, effector, and optical frames must be distinct')
        for key in ('max_sample_age_s', 'settle_time_s', 'stability_translation_m',
                    'stability_rotation_deg', 'max_reprojection_error_px', 'detector_rate_hz',
                    'max_translation_rms_m', 'max_rotation_rms_deg'):
            if not np.isfinite(self.cfg[key]) or self.cfg[key] <= 0:
                raise ValueError(f'{key} must be positive')
        if not 5 <= self.cfg['min_samples'] <= 500:
            raise ValueError('min_samples must be 5..500')
        from .calibration import METHODS
        if self.cfg['solver'] not in METHODS:
            raise ValueError('Unknown solver')
        if not self.cfg['output_directory'].strip():
            raise ValueError('output_directory must be set')
        self.spec = BoardSpec(**{k[6:]: v for k, v in self.cfg.items() if k.startswith('board.')})
        self.target = Target(self.spec)
        self.session_context = {k: self.cfg[k] for k in ('mode', 'base_frame', 'effector_frame', 'camera_frame',
                                               'image_topic', 'camera_info_topic', 'image_is_rectified')}
        self.session_context['board'] = self.spec.to_dict()
        self.session_context['detector_opencv_version'] = cv2.__version__
        self.session_context['camera_model'] = None
        self.directory = Path(self.cfg['output_directory']).expanduser()
        self.samples, self.result = [], None
        self.history = deque(maxlen=1000)
        self.pending = None
        self.info = None
        self.last_image_ns = -1
        self.last_processed = -float('inf')
        self.message = 'Waiting for matching Image, CameraInfo and robot TF'
        self.bridge = CvBridge()
        self.tf = Buffer(cache_time=Duration(seconds=30.0))
        self.listener = TransformListener(self.tf, self)
        self.image_sub = self.create_subscription(Image, self.cfg['image_topic'], self.on_image, qos_profile_sensor_data)
        self.info_sub = self.create_subscription(CameraInfo, self.cfg['camera_info_topic'], self.on_info, qos_profile_sensor_data)
        self.debug_pub = self.create_publisher(Image, '~/target_detection', qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(PoseStamped, '~/target_pose', qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.calibration_services = [self.create_service(Trigger, '~/'+name, self.service_handler(name))
                         for name in ('capture', 'undo', 'clear', 'solve', 'save', 'load')]
        self.tf_timer = self.create_timer(0.02, self.pair_pending)
        self.status_timer = self.create_timer(0.25, self.publish_status)
        self.get_logger().info(f'{self.cfg["mode"]}: {self.cfg["base_frame"]} -> {self.cfg["effector_frame"]}; '
                               f'optical frame {self.cfg["camera_frame"]}; session {self.directory}')
        if (self.directory/'session.yaml').exists():
            self.get_logger().warning('Existing session found. Use Load before Capture to resume it; '
                                      'Capture will otherwise replace the saved session.')

    def on_info(self, msg):
        self.info = msg

    def invalidate_observation(self, message):
        self.history.clear()
        self.pending = None
        self.message = str(message)

    def on_image(self, msg):
        if time.monotonic() - self.last_processed < 1.0/self.cfg['detector_rate_hz']:
            return
        self.last_processed = time.monotonic()
        debug = None
        try:
            if self.info is None:
                raise ValueError('Waiting for CameraInfo')
            if msg.header.frame_id != self.cfg['camera_frame'] or self.info.header.frame_id != msg.header.frame_id:
                raise ValueError('Image and CameraInfo frame_id must match configured optical camera_frame')
            stamp = Time.from_msg(msg.header.stamp)
            if stamp.nanoseconds <= 0:
                raise ValueError('Image has zero timestamp')
            if stamp.nanoseconds <= self.last_image_ns:
                self.invalidate_observation('Image clock reset or out-of-order image; waiting for fresh frames')
                self.last_image_ns = stamp.nanoseconds
                return
            self.last_image_ns = stamp.nanoseconds
            age = (self.get_clock().now().nanoseconds-stamp.nanoseconds)/1e9
            if age < -0.05 or age > self.cfg['max_sample_age_s']:
                raise ValueError('Image timestamp is stale/future; check clocks and use_sim_time')
            debug = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            k, d, correction = camera_model(self.info, msg.width, msg.height, self.cfg['image_is_rectified'])
            model = {'k': k.tolist(), 'd': d.tolist(), 'optical_from_rectified': correction.tolist(),
                     'width': msg.width, 'height': msg.height}
            if self.samples and model != self.session_context['camera_model']:
                raise ValueError('Camera intrinsics changed: clear samples or restore original camera settings')
            self.session_context['camera_model'] = model
            pose, error, debug = self.target.detect(debug, k, d, self.cfg['max_reprojection_error_px'])
            pose = transform(correction) @ pose
            # Retain the oldest unpaired observation until TF catches up or it expires.
            if self.pending is None:
                self.pending = (stamp, pose, error)
            target_msg = PoseStamped()
            target_msg.header = msg.header
            target_msg.pose.position.x, target_msg.pose.position.y, target_msg.pose.position.z = map(float, pose[:3, 3])
            q = quaternion(pose)
            target_msg.pose.orientation.x, target_msg.pose.orientation.y, target_msg.pose.orientation.z, target_msg.pose.orientation.w = q
            self.pose_pub.publish(target_msg)
        except (ValueError, cv2.error, RuntimeError, CvBridgeError) as exc:
            self.invalidate_observation(exc)
            if debug is not None:
                cv2.putText(debug, 'NO VALID SAMPLE: '+str(exc)[:70], (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        if debug is not None:
            image = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            image.header = msg.header
            self.debug_pub.publish(image)

    def pair_pending(self):
        if self.pending is None:
            return
        stamp, pose, error = self.pending
        if (self.get_clock().now().nanoseconds-stamp.nanoseconds)/1e9 > self.cfg['max_sample_age_s']:
            self.invalidate_observation('TF did not arrive at image timestamp before observation expired')
            return
        try:
            tf = self.tf.lookup_transform(self.cfg['base_frame'], self.cfg['effector_frame'], stamp)
        except TransformException as exc:
            self.message = 'Waiting for robot TF at image timestamp: '+str(exc)
            return  # Never block executor callbacks that deliver TF.
        self.history.append(Sample(tf_matrix(tf), pose, stamp.nanoseconds, error))
        self.pending = None
        self.message = f'Target paired with robot TF; reprojection RMS {error:.3f}px'

    def capture_candidate(self):
        return check_capture(self.history, self.samples, self.get_clock().now().nanoseconds,
                             self.cfg['max_sample_age_s'], self.cfg['settle_time_s'],
                             self.cfg['stability_translation_m'], self.cfg['stability_rotation_deg'])

    def change_samples(self, samples):
        if len(samples) > 500:
            raise ValueError('Maximum 500 samples')
        # Commit to disk before changing in-memory state; failed writes preserve old state.
        save_session(self.directory/'session.yaml', self.session_context, samples)
        self.samples, self.result = samples, None

    def service_handler(self, name):
        def handle(request, response):
            del request
            try:
                if name == 'capture':
                    self.change_samples(self.samples + [self.capture_candidate()])
                    message = f'Captured sample {len(self.samples)}'
                elif name == 'undo':
                    if not self.samples:
                        raise ValueError('No samples to remove')
                    self.change_samples(self.samples[:-1])
                    message = f'Removed last sample; {len(self.samples)} remain'
                elif name == 'clear':
                    self.change_samples([])
                    self.history.clear()
                    message = 'Session cleared'
                elif name == 'load':
                    if self.session_context['camera_model'] is None:
                        raise ValueError('Wait for a valid camera image before loading; intrinsics must match')
                    _, samples = load_session(self.directory/'session.yaml', self.session_context)
                    self.samples, self.result = samples, None
                    self.history.clear()
                    message = f'Loaded {len(samples)} samples'
                elif name == 'solve':
                    result = complete_result(solve(self.samples, self.cfg['mode'], self.cfg['solver'],
                                                   self.cfg['min_samples']), self.session_context.copy())
                    if self.cfg['mount_frame']:
                        # This must be a known rigid intra-camera transform, NOT the guessed robot mount.
                        c_from_m = tf_matrix(self.tf.lookup_transform(
                            self.cfg['camera_frame'], self.cfg['mount_frame'], Time()))
                        result['mount_frame'] = self.cfg['mount_frame']
                        result['parent_from_mount'] = (np.asarray(result['parent_from_camera']) @ c_from_m).tolist()
                    res = result['residuals']
                    result['quality_passed'] = (res['translation_rms_m'] <= self.cfg['max_translation_rms_m'] and
                                                res['rotation_rms_deg'] <= self.cfg['max_rotation_rms_deg'])
                    self.result = result
                    message = (f'Closure RMS: {res["translation_rms_m"]*1000:.2f} mm, '
                               f'{res["rotation_rms_deg"]:.3f} deg. '
                               + ('Quality thresholds passed.' if result['quality_passed'] else
                                  'Quality thresholds FAILED; inspect samples before export.'))
                elif name == 'save':
                    if self.result is None:
                        raise ValueError('Solve current samples first')
                    if not self.result['quality_passed']:
                        raise ValueError('Residual thresholds failed: calibration export blocked')
                    export_result(self.directory, self.result)
                    message = 'Saved calibration YAML and preview launch to '+str(self.directory)
                response.success, response.message = True, message
                self.message = message
            except (ValueError, OSError, TypeError, KeyError, cv2.error, TransformException) as exc:
                response.success, response.message = False, str(exc)
                self.message = str(exc)
            self.publish_status()
            return response
        return handle

    def publish_status(self):
        ready, reason = True, ''
        try:
            self.capture_candidate()
        except ValueError as exc:
            ready, reason = False, str(exc)
        payload = {'context': self.session_context, 'sample_count': len(self.samples), 'capture_ready': ready,
                   'min_samples': self.cfg['min_samples'],
                   'capture_reason': reason, 'message': self.message, 'result': self.result,
                   'output_directory': str(self.directory),
                   'samples': [{'index': i, 'stamp_ns': s.stamp_ns, 'reprojection_error_px': s.reprojection_error_px}
                               for i, s in enumerate(self.samples)]}
        self.status_pub.publish(String(data=json.dumps(payload, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CalibrationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
