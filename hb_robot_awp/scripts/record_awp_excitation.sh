#!/usr/bin/env bash
set -euo pipefail
OUT="${1:-awp_excitation_$(date +%Y%m%d_%H%M%S)}"
read -r -a EXTRA <<< "${EXTRA_TOPICS:-}"
exec ros2 bag record -s mcap -o "$OUT" \
 /awp_excitation_server/events /cut_pose_experiment/events \
 /awp_attitude/imu_1 /awp_attitude/imu_2 \
 /mpu6050/imu /mpu6050_2/imu /joint_states /tf /tf_static \
 /task_state /platform_supervision /motion_fault \
 /task_state_server/timing /inspect_cut_server/timing /rosout \
 /display_planned_path "${EXTRA[@]}"
