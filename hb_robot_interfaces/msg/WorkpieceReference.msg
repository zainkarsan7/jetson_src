# Atomic reference: cloud and pose come from the same successful inspection.
ProfileEstimate estimate
sensor_msgs/PointCloud2 cloud
# Revalidation keeps geometry identity; full inspection creates a new reference.
bool relocalized
uint64 expected_pose_version
