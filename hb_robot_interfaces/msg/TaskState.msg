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
