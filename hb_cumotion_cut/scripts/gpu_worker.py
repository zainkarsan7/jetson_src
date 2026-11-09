#!/usr/bin/env python3
"""Run beside the existing cuMotion goal-set planner in the Isaac ROS container."""
import math
import threading

import rclpy
from action_msgs.msg import GoalStatus
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
from hb_cumotion_cut.srv import PlanTransit, SolveProcessPath
from isaac_ros_cumotion.update_kinematics import get_robot_config
from isaac_ros_cumotion_interfaces.action import MotionPlan
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


class Worker(Node):
    def __init__(self):
        super().__init__('cut_gpu_worker')
        robot = self.declare_parameter('robot', '').value
        urdf = self.declare_parameter('urdf_path', '').value
        self.tool = self.declare_parameter('tool_frame', 'ur10e_torch_link').value
        config = get_robot_config(robot_file=robot, urdf_file_path=urdf,
                                  logger=self.get_logger())['robot_cfg']
        self.device = TensorDeviceType()
        self.ik = IKSolver(IKSolverConfig.load_from_robot_config(
            config, tensor_args=self.device, ee_link_name=self.tool, num_seeds=8,
            use_cuda_graph=False, position_threshold=0.0005, rotation_threshold=0.01))
        self.frame = self.ik.kinematics.base_link
        self.transit = ActionClient(self, MotionPlan, '/cumotion/motion_plan',
                                    callback_group=MutuallyExclusiveCallbackGroup())
        self.create_service(PlanTransit, '/cut_gpu/transit', self.plan_transit)
        self.create_service(SolveProcessPath, '/cut_gpu/process', self.process)

    def process(self, req, res):
        try:
            if req.frame != self.frame or req.tool != self.tool or not req.poses:
                raise ValueError('Unexpected frame, tool, or empty path')
            names = list(self.ik.joint_names)
            if len(req.seed.name) != len(req.seed.position) or set(req.seed.name) != set(names):
                raise ValueError('Process seed joints differ from GPU model')
            q = [req.seed.position[req.seed.name.index(n)] for n in names]
            if not all(map(math.isfinite, q)):
                raise ValueError('Nonfinite seed')
            res.path.joint_names = names
            # Bound temporary GPU memory; seed each chunk from its predecessor.
            for offset in range(0, len(req.poses), 32):
                chunk = req.poses[offset:offset + 32]
                targets = [[p.position.x, p.position.y, p.position.z, p.orientation.w,
                            p.orientation.x, p.orientation.y, p.orientation.z] for p in chunk]
                seeds = self.device.to_device(q).repeat(len(chunk), 1)
                solved = self.ik.solve_batch(Pose.from_batch_list(targets, self.device),
                                             retract_config=seeds, seed_config=seeds.unsqueeze(1),
                                             return_seeds=4)
                solutions = solved.solution.detach().cpu().tolist()
                flags = solved.success.reshape(len(chunk), -1).cpu().tolist()
                for choices, valid in zip(solutions, flags):
                    feasible = [v for v, ok in zip(choices, valid) if ok and all(map(math.isfinite, v))]
                    if not feasible:
                        res.message = 'No IK solution for Cartesian waypoint'
                        return res
                    q = min(feasible, key=lambda v: sum((a-b)**2 for a, b in zip(v, q)))
                    res.path.points.append(JointTrajectoryPoint(positions=q))
            res.success = True
        except Exception as error:
            res.backend_error = True
            res.message = str(error)
            self.get_logger().error(res.message)
        return res

    @staticmethod
    def wait(future, timeout):
        ready = threading.Event()
        future.add_done_callback(lambda _: ready.set())
        if not ready.wait(timeout):
            raise TimeoutError('cuMotion action timed out')
        return future.result()

    def plan_transit(self, req, res):
        try:
            if req.frame != self.frame or any(obj.header.frame_id != self.frame
                                               for obj in req.world.collision_objects):
                raise ValueError('Scene objects must use the GPU base frame')
            if not self.transit.wait_for_server(timeout_sec=2.0):
                raise RuntimeError('/cumotion/motion_plan is unavailable')
            goal = MotionPlan.Goal()
            goal.use_current_state = False
            goal.plan_cspace = True
            goal.use_planning_scene = True
            goal.start_state = req.start
            goal.goal_state = req.target
            goal.world = req.world
            goal.time_dilation_factor = 0.1
            handle = self.wait(self.transit.send_goal_async(goal), 5)
            if not handle.accepted:
                raise RuntimeError('cuMotion rejected transit')
            try:
                reply = self.wait(handle.get_result_async(), 45)
            except TimeoutError:
                handle.cancel_goal_async()
                raise
            if (reply.status != GoalStatus.STATUS_SUCCEEDED or not reply.result.success
                    or len(reply.result.planned_trajectory) != 1):
                res.message = 'cuMotion found no transit: ' + str(reply.result.message)
                return res
            res.trajectory = reply.result.planned_trajectory[0]
            res.success = True
        except Exception as error:
            res.backend_error = True
            res.message = str(error)
            self.get_logger().error(res.message)
        return res


def main():
    rclpy.init()
    worker = Worker()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(worker)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        worker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
