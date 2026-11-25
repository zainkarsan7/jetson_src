std_msgs/Header header
string task_id
# RUNNING, CONTAINING, PAUSED, NEEDS_INSPECTION, FAULT_UNCONTAINED,
# CANCELED_SAFE, COMPLETED. Separation is reported independently.
string outcome
string reason
string active_node
string plan_id
string batch_id
string interrupted_plan_id
string interrupted_feature_id
bool contained
bool separation_verified
string[] feature_ids
string[] feature_states
string recovery_feature
# Parallel to feature_ids; evidence epochs are historical after another cut.
uint64[] feature_execution_epochs
uint64[] feature_evidence_epochs
uint8[] feature_kerf_status
string[] feature_original_plan_ids
string[] feature_recut_plan_ids