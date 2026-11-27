# Diagnostic estimate only. Never published as platform motion authorization or TF.
std_msgs/Header header
string sensor
bool valid
bool gravity_used
bool calibrated
uint64 calibration_generation
uint64 samples
float64 source_age_ms
float64 dt_s
geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 rpy_rad
geometry_msgs/Vector3 delta_rpy_rad
geometry_msgs/Vector3 gyro_bias_rad_s
string status
