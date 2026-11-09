# GPU cut profile planner

`hb_cumotion_cut` supplies the same `/cut_profile` action, `/profile_inspection/estimate` subscription, `/display_planned_path`, approval service, stored-plan execution and recovery interfaces as the CPU server. An accepted planning result contains a server-local plan ID. The executable trajectories are stored by that server and published for display. Run one cut server per ROS domain. Request stored-plan execution from the same running instance that planned it.

## Build and run

On the ROS Humble host, build against the matching revision of the workspace, including the installed `hb_robot_skills` headers and motion library.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-up-to hb_cumotion_cut --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

In the existing Isaac ROS container, mount the standalone package under the container workspace's source tree. Source the container's Isaac ROS environment, then run the following from its workspace root. The worker-only option builds the two service types and installs the Python worker using the cuRobo installation bundled with cuMotion. The host and container must discover each other through ROS 2 and use matching service definitions.

```bash
colcon build --packages-select hb_cumotion_cut --cmake-args -DCONTAINER_WORKER_ONLY=ON
source install/setup.bash
ros2 action info /cumotion/motion_plan
ros2 run hb_cumotion_cut gpu_worker.py --ros-args \
  --params-file "$(ros2 pkg prefix --share hb_isaac_bringup)/config/cumotion.yaml"
```

Start the existing nvblox and cuMotion **goal-set** planner as usual. The action info command should show one action server. The worker reads the same `robot`, `urdf_path`, and `tool_frame` parameters as that planner. If `hb_isaac_bringup` is only mounted as source, pass the mounted `config/cumotion.yaml` path directly and ensure its URDF and XRDF paths resolve inside the container. The target API is the Isaac ROS Humble goal-set planner and cuRobo 0.7.x batch IK. The default GPU world and tool frames in this repo are `world` and `ur10e_torch_link`.

On the host, use this launch **in place of** `hb_robot_bringup skills.launch.py`. It starts the companion skills and selects the cut server. Pass `start_other_skills:=false` if those companion nodes are already running separately.

```bash
ros2 service type /cut_gpu/process
ros2 service type /cut_gpu/transit
ros2 launch hb_cumotion_cut cut_profile.launch.py cut_server_package:=hb_cumotion_cut
```

For the CPU run, stop the launch and start it again with `cut_server_package:=hb_robot_skills`. The launch uses the same graph weights, sample budgets and home-return settings for both runs. The worker and goal-set planner can stay running between runs if GPU memory permits.

## Planning and validation

* The existing host geometry planner converts the estimated profile and requested tolerances into cut segments. The host retains graph search, cost evaluation, phase assembly, time parameterization and execution supervision. These stages still contribute to end-to-end CPU time.
* For each sampled roll and seed, the worker solves approach, cut and retract Cartesian waypoints in batches on the GPU. It returns named joint positions, with several IK branches considered per waypoint. The host checks FK accuracy, orientation constraints, joint bounds, continuity and collisions against the detached planning scene before a candidate enters the graph. GPU IK uses robot self-collision geometry. The host checks the workpiece and other scene geometry for every accepted process waypoint.
* Process trajectories use the accepted IK samples directly. Each phase is timed using MoveIt joint limits and scaled to cap its sampled TCP speed at the requested phase speed. The resulting feed profile and duration can differ from Pilz LIN. Compare TCP speed through the cut as well as solver latency.
* Transit edges use the explicit start and target configurations and the snapshot's collision objects. The container calls the goal-set planner in joint-space mode. Its nvblox ESDF behavior follows the running cuMotion configuration. The host checks returned timestamps, joint names, finite values, waypoint bounds and collisions, and both joint endpoints. The shared graph validates phase joins and the Cartesian process geometry. A missing GPU service or transport error fails the request.
* The recovery and action implementation is carried within this package because the baseline server is installed as an executable. When updating the CPU server's action or execution behavior, synchronize this package before comparing revisions.

## Benchmark

Use a fixed estimate, robot start state and collision scene. Keep the task heartbeat live. Record the actual geometry, pose and scene versions from `/task_state`. Create a JSON goal using those values, for example:

```json
{"cut_id":"compare-01","execute":false,"standoff":0.01,"pos_tol":0.01,"ang_tol":0.3,
 "geometry_version":1,"pose_version":1,"scene_version":1}
```

Run the benchmark once per backend with the same goal file. Each line records end-to-end wall time, acceptance, result, plan ID and segment names. The backend label identifies the launch you selected. Record the first run separately to expose warm-up cost. Report successful latency and the number of failed attempts. A rejected or aborted request is not a fast plan.

```bash
ros2 run hb_cumotion_cut benchmark.py goal.json --backend cpu --runs 6 > cpu.jsonl
# Switch the launch to the GPU cut server.
ros2 run hb_cumotion_cut benchmark.py goal.json --backend gpu --runs 6 > gpu.jsonl
```

Capture Jetson CPU, GPU and memory readings with `tegrastats --interval 200` during the same trials. Inspect the individual trajectories on `/display_planned_path` for cut feed, path deviation and transit travel. Compare like-for-like scenes and report whether the live nvblox ESDF supplied additional obstacles to the GPU planner.

## Tests

```bash
colcon test --packages-select hb_cumotion_cut --return-code-on-test-failure
colcon test-result --test-result-base build/hb_cumotion_cut
```

The host integration tests use a ROS service with analytically known prismatic IK to check process reconstruction and rejection of an incorrect GPU response. Running a plan-only goal through the real container is the end-to-end hardware check.

[NVIDIA goal-set planner](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion/blob/release-3.2/isaac_ros_cumotion/isaac_ros_cumotion/cumotion_goal_set_planner.py) · [cuRobo IK solver](https://github.com/NVlabs/curobo/blob/v0.7.8/src/curobo/wrap/reacher/ik_solver.py)
