#!/bin/bash

# --- ROS 2 환경 설정 ---
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger

echo "=== 터틀봇3 하드웨어 & SLAM & 저화질 카메라 시작 ==="

# 1. 로봇 기본 구동
echo "[1/2] Bringup 실행 중..."
ros2 launch turtlebot3_bringup robot.launch.py > /dev/null 2>&1 &
PID_ROS=$!
sleep 5

# 2. 카메라 실행 (해상도 320x240 / 화질 100)
# - image_size: 해상도 설정
# - image_raw.compressed.jpeg_quality: 압축 화질 (1~100). 낮을수록 용량 작음.
echo "[2/2] 카메라(저화질 모드) 실행 중..."
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p image_size:="[320,240]" \
  -p image_raw.compressed.jpeg_quality:=100 > /dev/null 2>&1 & # jpeg_quality 15 로 하면 영상 화질이 너무 낮음
PID_CAM=$!

echo "---------------------------------------"
echo "시스템 실행 완료 (PID: $PID_ROS, $PID_SLAM, $PID_CAM)"
echo "   - 주의: 화질을 낮췄으므로 책에 가까이 가야 숫자가 보입니다."
echo "---------------------------------------"
echo "종료하려면 Ctrl+C를 누르세요"

cleanup() {
    echo "=== 종료합니다 ==="
    kill $PID_ROS $PID_SLAM $PID_CAM
    exit
}

trap cleanup SIGINT SIGTERM

wait