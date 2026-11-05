std_msgs/Header header
string cut_id
uint64 geometry_version
uint64 pose_version
uint64 execution_version
bool registration_valid
# Enumerates ALL required original features, including unobserved ones.
string[] required_segments
KerfSegmentEvidence[] segments
float64 observed_fraction
float64 open_fraction
float64 unknown_fraction
