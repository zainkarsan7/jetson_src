# ProfileEstimate.msg
# estimated in the planning/world frame, profile should be one of the families in profile library
# pose is profile plane , z along the extrusion direction
# should already be i planning world frame
std_msgs/Header header
string profile_name
geometry_msgs/Pose pose
#not sure i need this now
float32 section_thickness

### fit quality/ confidence
float32 rms_distance
float32 inlier_fraction
float32 score

### observation statistic
uint32 observation_points_count
