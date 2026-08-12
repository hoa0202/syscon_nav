#!/bin/bash
# V2: 출발점 + 도착점만 티칭 (P2P 직선용)
# 사용법: ./launch_teach_v2.sh
#  1) 출발 위치에서 실행
#  2) 리모컨으로 목적지까지 이동
#  3) Ctrl+C → 출발/도착 2점 저장

# sh로 실행해도 bash로 재실행
if [ -z "${BASH_VERSION:-}" ]; then
    exec /bin/bash "$0" "$@"
fi

source ~/.bashrc
cd ~/roboro_nav

echo "🔴 V2 출발/도착 티칭 시작"
echo "   출발지에서 실행 → 목적지로 이동 → Ctrl+C 저장"
echo "   (시작점 + 종료 시점 위치만 저장)"
echo ""

python3 teach_recorder_slam.py \
    --ros-args \
    -p odom_topic:=/liorf/mapping/odometry \
    -p record_mode:=endpoints
