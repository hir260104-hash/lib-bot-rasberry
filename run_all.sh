#!/bin/bash

# --- 0. 설정 (IP 주소 받기) ---
# 사용법: ./run_all.sh [노트북IP]
# 예시: ./run_all.sh 192.168.0.5
# 입력이 없으면 기본값(192.168.0.3)을 사용합니다.
TARGET_IP=${1:-"192.168.0.3"}

echo "🎯 타겟 노트북 IP: $TARGET_IP 로 설정되었습니다."

# --- 1. ROS 2 환경 설정 ---
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger

# --- 2. 다운로드 받은 폴더로 이동 ---
cd ~/lib-bot-rasberry

echo "=== 🤖 터틀봇 소켓 서버 3종 세트 시작 ==="

# --- 3. 프로그램 실행 (백그라운드 &) ---

# (1) 제어 서버 실행
python3 movebot_twist.py &
PID1=$!
echo "✅ 실행됨: MoveBot Twist (PID: $PID1)"
sleep 1

# (2) 영상 송신 서버 실행 (파라미터 전달 부분 수정됨!)
python3 pi_ros_bridge.py --ros-args -p laptop_ip:=$TARGET_IP &
PID3=$!
echo "✅ 실행됨: Video Bridge (PID: $PID3) -> IP: $TARGET_IP"

echo "---------------------------------------"
echo "🛑 종료하려면 Ctrl+C를 누르세요"
echo "---------------------------------------"

# --- 4. 종료 처리 (Trap) ---
cleanup() {
    echo ""
    echo "=== 🛑 모든 프로그램을 종료합니다 ==="
    # PID_ODOM도 종료 목록에 추가했습니다.
    kill $PID1 $PID2 $PID3 $PID_ODOM
    exit
}

trap cleanup SIGINT SIGTERM

wait