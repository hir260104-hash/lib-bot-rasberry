import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
import socket
import threading

# [설정값] 로봇의 물리적 한계 및 부드러움 정도
LIN_VEL_STEP = 0.02  # 가속도 (0.05초당 증가량)
ANG_VEL_STEP = 0.05  # 회전 가속도
MAX_LIN_VEL = 0.22  # 최대 선속도 제한
MAX_ANG_VEL = 2.84  # 최대 각속도 제한


class MoveBot(Node):
    def __init__(self):
        super().__init__('movebot_node')
        # TwistStamped 메시지 사용 (Turtlebot3 기본 설정에 따라 Twist로 변경 가능)
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # 현재 실제 속도와 목표 속도 초기화
        self.current_lin = 0.0
        self.current_ang = 0.0
        self.target_lin = 0.0
        self.target_ang = 0.0

        # [중요] 사용자가 QDial을 돌리기 전 기본 주행 속도 설정
        self.user_set_lin = 0.10
        self.user_set_ang = 0.5

        # 0.05초마다 실제 속도를 갱신하는 타이머
        self.timer = self.create_timer(0.05, self.publish_twist)

        # TCP 소켓 서버 설정
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind(('0.0.0.0', 9999))
        self.server_sock.listen(1)

        # 소켓 수신을 별도 스레드에서 실행
        threading.Thread(target=self.start_listening, daemon=True).start()
        self.get_logger().info("=== [로봇 제어 서버] 구동 시작 (Port: 9999) ===")

    def constrain(self, val, min_val, max_val):
        """값의 범위를 제한하는 함수"""
        return max(min_val, min(val, max_val))

    def publish_twist(self):
        """가속도를 계산하여 목표 속도까지 부드럽게 도달하고 메시지 발행"""
        # 선속도 가속/감속 제어
        if self.target_lin > self.current_lin:
            self.current_lin = min(self.current_lin + LIN_VEL_STEP, self.target_lin)
        elif self.target_lin < self.current_lin:
            self.current_lin = max(self.current_lin - LIN_VEL_STEP, self.target_lin)

        # 각속도 가속/감속 제어
        if self.target_ang > self.current_ang:
            self.current_ang = min(self.current_ang + ANG_VEL_STEP, self.target_ang)
        elif self.target_ang < self.current_ang:
            self.current_ang = max(self.current_ang - ANG_VEL_STEP, self.target_ang)

        # 메시지 생성 및 발행
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = self.current_lin
        msg.twist.angular.z = self.current_ang
        self.publisher.publish(msg)

    def start_listening(self):
        """GUI로부터 명령어를 수신하는 루프"""
        while rclpy.ok():
            try:
                conn, addr = self.server_sock.accept()
                self.get_logger().info(f"GUI 연결됨: {addr}")
                with conn:
                    while rclpy.ok():
                        raw_data = conn.recv(1024)
                        if not raw_data:
                            break

                        data = raw_data.decode().strip()
                        # 여러 명령이 한꺼번에 올 경우 개행문자(\n)로 분리
                        for cmd in data.split('\n'):
                            cmd = cmd.strip()
                            if not cmd:
                                continue

                            # 1. 속도 설정 명령 (QDial)
                            if cmd.startswith('v:'):
                                val = float(cmd.split(':')[1])
                                self.user_set_lin = self.constrain(val, 0.0, MAX_LIN_VEL)
                            elif cmd.startswith('r:'):
                                val = float(cmd.split(':')[1])
                                self.user_set_ang = self.constrain(val, 0.0, MAX_ANG_VEL)

                            # 2. 방향 제어 명령 (Buttons/Keys)
                            elif cmd == 'w':  # 전진
                                self.target_lin = self.user_set_lin
                                self.target_ang = 0.0
                            elif cmd == 'x':  # 후진
                                self.target_lin = -self.user_set_lin
                                self.target_ang = 0.0
                            elif cmd == 'a':  # 좌회전
                                self.target_ang = self.user_set_ang
                                self.target_lin = 0.0
                            elif cmd == 'd':  # 우회전
                                self.target_ang = -self.user_set_ang
                                self.target_lin = 0.0
                            elif cmd == 's':  # 정지
                                self.target_lin = 0.0
                                self.target_ang = 0.0
                self.get_logger().info("GUI 연결 종료. 대기 중...")
            except Exception as e:
                self.get_logger().error(f"통신 에러 발생: {e}")


def main():
    rclpy.init()
    try:
        bot = MoveBot()
        rclpy.spin(bot)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
