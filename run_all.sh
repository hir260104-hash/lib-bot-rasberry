#!/bin/bash

# --- 1. ROS 2 환경 설정 (필요시 humble 등으로 변경) ---
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger

# --- 2. 다운로드 받은 폴더로 이동 ---
# 깃허브에서 받은 폴더명이 lib-bot-rasberry 라고 가정
cd ~/lib-bot-rasberry

echo "=== 🤖 터틀봇 소켓 서버 3종 세트 시작 ==="

# --- 3. 프로그램 실행 (백그라운드 &) ---

# (1) 제어 서버 실행
python3 movebot_twist.py &
PID1=$!
echo "✅ 실행됨: MoveBot Twist (PID: $PID1)"
sleep 1 # 1초 간격으로 순차 실행 (충돌 방지)

# (2) 지도 중계 서버 실행
python3 pi_map_server.py &
PID2=$!
echo "✅ 실행됨: Map Server (PID: $PID2)"
sleep 1

# [추가] (3) 위치(Odom) 서버 실행
python3 pi_odom_server.py &
PID_ODOM=$!
echo "✅ 실행됨: Odom Server (PID: $PID_ODOM)"
sleep 1

# (3) 영상 송신 서버 실행
python3 pi_ros_bridge.py &
PID3=$!
echo "✅ 실행됨: Video Bridge (PID: $PID3)"

echo "---------------------------------------"
echo "🛑 종료하려면 Ctrl+C를 누르세요"
echo "---------------------------------------"

# --- 4. 종료 처리 (Trap) ---
# Ctrl+C(SIGINT)가 들어오면 실행된 프로그램들을 모두 죽임
cleanup() {
    echo ""
    echo "=== 🛑 모든 프로그램을 종료합니다 ==="
    kill $PID1 $PID2 $PID3
    exit
}

trap cleanup SIGINT SIGTERM

# 스크립트가 바로 꺼지지 않고 계속 대기하게 함
wait