"""Run under Humble/colcon; explicitly skipped on hosts without ROS 2."""
import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import TransformStamped
from rclpy.parameter import Parameter
from rclpy.time import Time
from std_srvs.srv import Trigger
from sensor_msgs.msg import CameraInfo

from handeye_calibration_ros2.node import CalibrationNode
from test_calibration import dataset
from test_target import render_observation
from handeye_calibration_ros2.target import BoardSpec


@pytest.fixture
def node(tmp_path):
    rclpy.init()
    n = CalibrationNode(parameter_overrides=[Parameter('output_directory', value=str(tmp_path))])
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_timestamped_tf_pairing_uses_image_time_not_latest(node):
    now = node.get_clock().now().nanoseconds
    for offset, x in [(-200000000, 0.1), (-100000000, 0.2), (0, 0.3)]:
        msg = TransformStamped()
        msg.header.frame_id = node.cfg['base_frame']
        msg.child_frame_id = node.cfg['effector_frame']
        msg.header.stamp = Time(nanoseconds=now+offset).to_msg()
        msg.transform.rotation.w = 1.0
        msg.transform.translation.x = x
        node.tf.set_transform(msg, 'test')
    stamp = Time(nanoseconds=now-150000000)
    node.pending = (stamp, np.eye(4), 0.2)
    node.pair_pending()
    assert node.pending is None
    assert node.history[-1].base_from_effector[0, 3] == pytest.approx(0.15)
    assert node.history[-1].stamp_ns == stamp.nanoseconds


def test_trigger_solve_export_and_invalidation(node):
    samples, _, _ = dataset()
    node.change_samples(samples)
    solved = node.service_handler('solve')(Trigger.Request(), Trigger.Response())
    assert solved.success, solved.message
    saved = node.service_handler('save')(Trigger.Request(), Trigger.Response())
    assert saved.success, saved.message
    assert (node.directory/'calibration.yaml').is_file()
    node.service_handler('undo')(Trigger.Request(), Trigger.Response())
    assert node.result is None
    assert not node.service_handler('save')(Trigger.Request(), Trigger.Response()).success


def test_real_ros_messages_image_to_paired_sample(node):
    _, image, k, _ = render_observation(BoardSpec())
    info = CameraInfo()
    info.header.frame_id = node.cfg['camera_frame']
    info.width, info.height = image.shape[1], image.shape[0]
    info.k = list(map(float, k.reshape(-1)))
    info.d = [0.] * 5
    info.distortion_model = 'plumb_bob'
    node.on_info(info)
    msg = node.bridge.cv2_to_imgmsg(image, encoding='bgr8')
    msg.header.frame_id = node.cfg['camera_frame']
    msg.header.stamp = node.get_clock().now().to_msg()
    tf = TransformStamped()
    tf.header.frame_id = node.cfg['base_frame']
    tf.child_frame_id = node.cfg['effector_frame']
    tf.header.stamp = msg.header.stamp
    tf.transform.rotation.w = 1.0
    node.tf.set_transform(tf, 'test')
    node.on_image(msg)
    assert node.pending is not None, node.message
    node.pair_pending()
    assert len(node.history) == 1
    # A missing target must invalidate capture history, not silently reuse it.
    node.last_processed = -float('inf')
    msg = node.bridge.cv2_to_imgmsg(np.full_like(image, 255), encoding='bgr8')
    msg.header.frame_id = node.cfg['camera_frame']
    msg.header.stamp = node.get_clock().now().to_msg()
    node.on_image(msg)
    assert not node.history


def test_service_through_ros_graph(node):
    import time
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    client_node = Node('handeye_test_client')
    client = client_node.create_client(Trigger, '/handeye/solve')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(client_node)
    try:
        assert client.wait_for_service(timeout_sec=5.0)
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic()+5
        while not future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert future.done()
        assert not future.result().success
        assert 'at least' in future.result().message
    finally:
        executor.remove_node(node)
        executor.remove_node(client_node)
        executor.shutdown()
        client_node.destroy_node()
