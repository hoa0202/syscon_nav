#!/bin/bash
# SLAM 기반 경로 추종 (Repeat 모드)
# 사용법: ./launch_follow.sh [경로파일]

source ~/.bashrc
cd /root/nav2_v2/roboro_nav

PATH_FILE=${1:-"/root/nav2_v2/roboro_nav/paths/latest_path.csv"}

echo "🚀 경로 추종 시작"
echo "   경로 파일: $PATH_FILE"
echo ""

python3 path_follower_slam.py \
    --ros-args \
    -p path_file:="$PATH_FILE" \
    -p odom_topic:=/liorf/mapping/odometry \
    -p cmd_topic:=/cmd_vel_main \
    -p linear_vel:=0.15 \
    -p max_angular_vel:=0.4 \
    -p lookahead_dist:=0.3 \
    -p goal_tolerance:=0.15 \
    -p angular_kp:=1.5
