# ROS stamp for bag correlation; all durations below use a monotonic clock.
std_msgs/Header header
string role
uint64 timer_ticks
uint64 callbacks
uint64 rejected
uint64 middleware_arrivals
float64 callback_gap_ms
float64 max_callback_gap_ms
# Message header age at callback entry: includes transport, scheduling AND clock offset.
# This is NOT an isolated DDS latency measurement.
float64 source_age_at_entry_ms
float64 state_lock_wait_ms
float64 max_state_lock_wait_ms
float64 timer_lateness_ms
float64 max_timer_lateness_ms
float64 callback_duration_ms
float64 max_callback_duration_ms
float64 publish_duration_ms
float64 max_publish_duration_ms
