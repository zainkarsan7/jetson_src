
std_msgs/Header header

string sensor_name

bool valid
bool stable

uint32 sample_count
float64 sample_age

uint64 reception_gaps

geometry_msgs/Vector3 gyro_mean_rad_s
geometry_msgs/Vector3 gyro_stddev_rad_s

geometry_msgs/Vector3 accel_mean_m_s2
geometry_msgs/Vector3 accel_stddev_m_s2

float64 gyro_residual_rms_rad_s
float64 accel_residual_rms_m_s2

float64 gyro_peak_rad_s
float64 accel_deviation_peak_m_s2

float64 gyro_latest_rad_s
float64 accel_deviation_latest_m_s2

float64 accel_mean_norm_m_s2

float64 stable_confidence
float64 confidence_coverage_s

bool gyro_rms_pass
bool accel_rms_pass
bool mean_gyro_pass
