#!/bin/bash
# SLAM 기반 경로 기록 (Teach 모드)
# 사용법: ./launch_teach.sh

# sh로 실행해도 bash로 재실행
if [ -z "${BASH_VERSION:-}" ]; then
    exec /bin/bash "$0" "$@"
fi

source ~/.bashrc
cd ~/roboro_nav

echo "🔴 경로 기록 시작 - 로봇을 움직이세요!"
echo "   Ctrl+C로 종료하면 경로가 저장됩니다"
echo "   (좁은 공간용: 5cm마다 기록)"
echo ""

python3 teach_recorder_slam.py \
    --ros-args \
    -p odom_topic:=/liorf/mapping/odometry \
    -p record_mode:=dense \
    -p sample_dist:=0.05 \
    -p sample_angle:=0.05
