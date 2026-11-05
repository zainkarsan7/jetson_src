# An accepted post-cut observation. Record this topic for later ray-based inference.
std_msgs/Header header
string cut_id
uint64 geometry_version
uint64 pose_version
uint64 execution_version
string feature_name
uint32 viewpoint_index
geometry_msgs/Pose cut_pose
geometry_msgs/TransformStamped camera_pose
sensor_msgs/Image rgb
sensor_msgs/Image depth
sensor_msgs/CameraInfo camera_info
# Current implementation uses TF only; no hidden ICP correction.
bool registration_refined
