#!/bin/bash
# V2: 단순 후진 복귀 (목표 bearing + v/ω)
# 사용법: ./launch_return_v2.sh [경로파일]

if [ -z "${BASH_VERSION:-}" ]; then
    exec /bin/bash "$0" "$@"
fi

source ~/.bashrc
cd ~/roboro_nav

PATH_FILE=${1:-"$(pwd)/paths/latest_path.csv"}
LINEAR_VEL=${LINEAR_VEL:-0.5}
GOAL_TOL=${GOAL_TOL:-0.08}

echo "🔙 V2 복귀 시작  v=-${LINEAR_VEL}  tol=${GOAL_TOL}"
echo "   $PATH_FILE"

python3 path_follower_slam.py \
    --ros-args \
    -p path_file:="$PATH_FILE" \
    -p odom_topic:=/liorf/mapping/odometry \
    -p cmd_topic:=/cmd_vel \
    -p follow_mode:=p2p_line \
    -p drive_dir:=reverse \
    -p save_actual_path:=false \
    -p linear_vel:="$LINEAR_VEL" \
    -p max_angular_vel:=0.8 \
    -p goal_tolerance:="$GOAL_TOL" \
    -p angular_kp:=2.0 \
    -p slow_dist:=1.5 \
    -p min_vel:=0.05 \
    -p crawl_dist:=0.35 \
    -p crawl_vel:=0.08 \
    -p control_rate:=50.0
