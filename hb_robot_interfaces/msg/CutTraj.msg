# CutTraj.msg

# planned rob traj with process events with moveit as the traj message basis

moveit_msgs/RobotTrajectory trajectory
uint32 torch_on_index
uint32 cut_start_index
uint32 cut_end_index
uint32 torch_off_index