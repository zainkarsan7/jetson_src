# IK warm starts only. Never authorize motion or reuse a cut collision scene.
std_msgs/Header header
string cut_id
string profile_name
string planning_group
uint64 geometry_version
uint64 pose_version
# Execution/scene versions deliberately do not gate reuse: every inspection
# resolves fresh camera IK and validates against its own scene and supervision.
# Parallel arrays, one feature name per approach/retract joint state.
string[] features
sensor_msgs/JointState[] states
