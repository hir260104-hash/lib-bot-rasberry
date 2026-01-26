#!/bin/bash

# --- ROS 2 환경 설정 ---
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger

echo "=== 🐢 터틀봇3 기본 하드웨어 & SLAM 구동 시작 ==="

# 1. 로봇 기본 구동 (모터, 라이다, Core)
echo "🚀 [1/3] Bringup 실행 중..."
ros2 launch turtlebot3_bringup robot.launch.py > /dev/null 2>&1 &
PID_ROS=$!
sleep 5 # 하드웨어 초기화 대기

# 2. SLAM (Cartographer) 실행
echo "🗺️ [2/3] SLAM (지도 생성) 실행 중..."
ros2 launch turtlebot3_cartographer cartographer.launch.py > /dev/null 2>&1 &
PID_SLAM=$!
sleep 3

# 3. 카메라 드라이버 실행 (v4l2_camera)
# 해상도를 640x480으로 낮춰서 실행 (전송 속도 최적화)
echo "📷 [3/3] 카메라 노드 실행 중..."
ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[640,480]" > /dev/null 2>&1 &
PID_CAM=$!

echo "---------------------------------------"
echo "✅ 모든 시스템이 백그라운드에서 실행 중입니다."
echo "   - ROS Bringup (PID: $PID_ROS)"
echo "   - SLAM (PID: $PID_SLAM)"
echo "   - Camera (PID: $PID_CAM)"
echo "---------------------------------------"
echo "🛑 종료하려면 Ctrl+C를 누르세요"

# --- 종료 처리 (Trap) ---
cleanup() {
    echo ""
    echo "=== 🛑 기본 시스템을 종료합니다 ==="
    kill $PID_ROS $PID_SLAM $PID_CAM
    exit
}

trap cleanup SIGINT SIGTERM

# 무한 대기 (스크립트가 꺼지지 않게)
wait