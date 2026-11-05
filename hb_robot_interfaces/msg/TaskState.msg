# TaskState.msg

string task_id
string scene_id 
uint8 phase

uint8 IDLE =0
uint8 INSPECTING =1
uint8 PLANNING=2
uint8 POSITIONING=3
uint8 CUTTING=4
uint8 VERIFYING=5
uint8 COMPLETE=6
uint8 FAILED=7

string active_skill

bool scene_valid
bool path_valid
bool cut_complete
uint8 retry_count
string status_message

# Recovery validity is independent of task phase. Default values deny execution.
std_msgs/Header header
uint64 geometry_version
uint64 pose_version
uint64 scene_version
uint64 execution_version
bool workpiece_pose_valid
bool imu_stable
bool process_fault_active
bool motion_fault_active
bool supervision_valid
string active_cut_id
string active_plan_id
uint8 INVALID_OBSERVATION=0
uint8 INCONCLUSIVE=1
uint8 VERIFIED=2
uint8 RESIDUAL_MATERIAL=3
uint8 kerf_status
CutInterval[] residual_intervals
