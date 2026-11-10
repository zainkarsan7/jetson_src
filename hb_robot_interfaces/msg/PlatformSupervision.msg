
std_msgs/Header header

hb_robot_interfaces/PlatformImuMetrics unilidar
hb_robot_interfaces/PlatformImuMetrics mpu6050

float64 combined_confidence
float64 combined_coverage_s

bool both_channels_valid
bool both_channels_stable

bool supervision_valid
bool imu_stable

bool hard_motion_detected

uint8 UNKNOWN=0
uint8 STABLE=1
uint8 UNCERTAIN=2
uint8 UNTRUSTED=3

uint8 state
string reason
