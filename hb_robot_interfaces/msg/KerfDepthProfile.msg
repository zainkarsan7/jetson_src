# One bounded batch per RGB-D frame, scoped to exactly one cutting feature.
std_msgs/Header header
string cut_id
string feature_id
uint64 geometry_version
uint64 pose_version
uint64 execution_version
# Coordinates use the SAME trimmed feature interval as the recut/evidence evaluator.
geometry_msgs/Point start
geometry_msgs/Point end
geometry_msgs/Vector3 inward_normal
float64 axial_half_band_m
float64 uncertainty_m
# Parallel arrays. s describes the nominal front-face ray intersection.
# Depth is signed distance along inward_normal from the nominal feature surface.
# NaN measured_depth_m = missing/invalid return, NEVER open space.
float32[] s
float32[] measured_depth_m
float32[] material_entry_m
float32[] material_exit_m
float32[] axial_m
uint8 UNKNOWN=0
uint8 OPEN=1
uint8 MATERIAL=2
uint8[] classification
# Broader plotting band samples are diagnostic only; only strict slit rays qualify.
bool[] within_kerf
# Current authoritative bin verdicts (the existing three-ray evaluator).
float32[] bin_s
uint8[] bin_classification
