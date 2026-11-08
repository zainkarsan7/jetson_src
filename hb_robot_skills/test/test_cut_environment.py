"""ROS integration test with timestamped clouds, TF and in-memory MoveGroup services."""
import runpy
import sys
import threading
import unittest
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, TransformStamped
from hb_robot_interfaces.msg import TaskState
from hb_robot_interfaces.srv import CaptureCutEnvironment
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
from shape_msgs.msg import Mesh, MeshTriangle
from tf2_ros import StaticTransformBroadcaster

scripts = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(scripts))
CaptureNode = runpy.run_path(str(scripts/'capture_cut_environment'))['CaptureNode']


class EnvironmentTest(unittest.TestCase):
    def test_capture_replace_empty_and_failure(self):
        rclpy.init()
        capture = CaptureNode()
        peer = Node('environment_fixture')
        executor = MultiThreadedExecutor(num_threads=5)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        state_pub = peer.create_publisher(TaskState, '/task_state', qos)
        cloud_pub = peer.create_publisher(PointCloud2, '/hb_robot_lidar/cloud', 5)
        state = TaskState(geometry_version=1, pose_version=2, scene_version=3,
                          imu_stable=True, supervision_valid=True, scene_valid=True, workpiece_pose_valid=False)
        work = CollisionObject(id='wk_workpiece_collision')
        work.header.frame_id = 'world'
        # Triangular extrusion. The diagonal leaves an open region inside its bounding box.
        mesh = Mesh(vertices=[Point(x=x, y=y, z=z) for z in (0.2, 0.5)
                              for x, y in ((0., 0.), (0.5, 0.), (0., 0.5))],
                    triangles=[MeshTriangle(vertex_indices=t) for t in ([0, 1, 2], [3, 4, 5])])
        work.pose.position.x = 0.3
        mesh_pose = Pose()
        mesh_pose.position.y = 0.2
        work.meshes, work.mesh_poses = [mesh], [mesh_pose]
        objects = {work.id: work}
        applied = []

        def get_scene(req, res):
            res.scene.world.collision_objects = list(objects.values())
            return res

        def apply_scene(req, res):
            applied.append(req.scene)
            for o in req.scene.world.collision_objects:
                if o.operation == o.REMOVE:
                    objects.pop(o.id)
                else:
                    objects[o.id] = o
            res.success = True
            return res

        peer.create_service(GetPlanningScene, '/get_planning_scene', get_scene)
        peer.create_service(ApplyPlanningScene, '/apply_planning_scene', apply_scene)
        broadcaster = StaticTransformBroadcaster(peer)
        frames = set(capture.arm + ['camera_base', 'ur10e_bracket_link', 'ur10e_torch_link', 'lidar'])
        transforms = []
        for link in frames:
            t = TransformStamped()
            t.header.frame_id, t.child_frame_id = 'world', link
            t.header.stamp = peer.get_clock().now().to_msg()
            t.transform.translation.x = 0. if link == 'lidar' else -0.6
            transforms.append(t)
        broadcaster.sendTransform(transforms)
        # Jacket shell, profile interior, cut-surface noise, outside range.
        fixed = [(-0.6, 0.14, 0.), (0.40, 0.30, 0.35), (0.40, 0.19, 0.35), (2.2, 0., 0.)]
        obstacle = [(0.72, 0.62, 0.35)]
        def publish_state():
            state.header.stamp = peer.get_clock().now().to_msg()
            state_pub.publish(state)
        peer.create_timer(0.05, publish_state)
        def publish():
            xyz = np.array(fixed + obstacle, dtype='<f4')
            cloud = PointCloud2(height=1, width=len(xyz), point_step=12, row_step=12*len(xyz))
            cloud.header.frame_id, cloud.header.stamp = 'lidar', peer.get_clock().now().to_msg()
            cloud.fields = [PointField(name=k, offset=i*4, datatype=PointField.FLOAT32, count=1)
                            for i, k in enumerate('xyz')]
            cloud.data = xyz.tobytes()
            cloud_pub.publish(cloud)
        timer = peer.create_timer(1/12, publish)
        client = peer.create_client(CaptureCutEnvironment, '/capture_cut_environment')
        executor.add_node(peer)
        executor.add_node(capture)
        spinning = threading.Thread(target=executor.spin)
        spinning.start()
        def call():
            future = client.call_async(CaptureCutEnvironment.Request(
                geometry_version=1, pose_version=2, scene_version=3))
            ready = threading.Event()
            future.add_done_callback(lambda _: ready.set())
            self.assertTrue(ready.wait(12))
            return future.result()
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=3))
            for _ in range(40):
                if capture.state is not None:
                    break
                threading.Event().wait(0.05)
            first = call()
            self.assertTrue(first.success, first.message)
            self.assertEqual(first.occupied_voxels, 1)
            self.assertAlmostEqual(objects[first.environment_id].primitive_poses[0].position.x,
                                   (np.floor(np.float32(0.72)/0.04)+0.5)*0.04)
            timer.cancel()
            failed = call()
            self.assertFalse(failed.success)
            self.assertIn('Insufficient timestamp coverage', failed.message)
            self.assertIn(first.environment_id, objects)
            self.assertEqual(len(applied), 1)
            obstacle.clear()
            timer.reset()
            threading.Event().wait(0.2)
            empty = call()
            self.assertTrue(empty.success, empty.message)
            self.assertEqual(empty.occupied_voxels, 0)
            self.assertEqual(set(objects), {'wk_workpiece_collision'})
        finally:
            executor.shutdown()
            spinning.join()
            capture.destroy_node()
            peer.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
