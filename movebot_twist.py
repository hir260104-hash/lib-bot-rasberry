import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
import socket
import threading

# 설정값 조정
LIN_VEL_STEP = 0.02  # 가속도
ANG_VEL_STEP = 0.05  # 회전 가속도
MAX_LIN_VEL = 0.22   # 최대 전진 속도
MAX_ANG_VEL = 1.5    # 최대 회전 속도

class MoveBot(Node):
    def __init__(self):
        super().__init__('movebot_node')
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        # 현재 실제 속도와 목표 속도를 분리
        # target: 가고 싶은 속도
        # current: 현재 로봇의 실제 속도
        self.current_lin = 0.0
        self.current_ang = 0.0
        self.target_lin = 0.0
        self.target_ang = 0.0
        self.timer = self.create_timer(0.05, self.publish_twist) # 0.05초마다 갱신

        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind(('0.0.0.0', 9999))
        self.server_sock.listen(1)
        threading.Thread(target=self.start_listening, daemon=True).start()
        self.get_logger().info("=== [부드러운 주행 모드] 활성화 ===")

    def constrain(self, val, min_val, max_val):
        return max(min_val, min(val, max_val))

    def publish_twist(self):
        """가속도를 계산하여 부드럽게 실제 속도를 목표 속도에 맞춤"""
        # 선속도 가속 제어
        if self.target_lin > self.current_lin:
            self.current_lin = min(self.current_lin + LIN_VEL_STEP, self.target_lin)
        elif self.target_lin < self.current_lin:
            self.current_lin = max(self.current_lin - LIN_VEL_STEP, self.target_lin)

        # 각속도(회전) 가속 제어
        if self.target_ang > self.current_ang:
            self.current_ang = min(self.current_ang + ANG_VEL_STEP, self.target_ang)
        elif self.target_ang < self.current_ang:
            self.current_ang = max(self.current_ang - ANG_VEL_STEP, self.target_ang)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = self.current_lin
        msg.twist.angular.z = self.current_ang
        self.publisher.publish(msg)

    def start_listening(self):
        while rclpy.ok():
            try:
                conn, _ = self.server_sock.accept()
                while rclpy.ok():
                    data = conn.recv(1024).decode().strip()
                    if not data: break
                    # 목표 속도만 설정 (실제 가속은 타이머에서 처리)
                    if data == 'w': self.target_lin = 0.10; self.target_ang = 0.0
                    elif data == 'x': self.target_lin = -0.10; self.target_ang = 0.0
                    elif data == 'a': self.target_ang = 0.5; self.target_lin = 0.0
                    elif data == 'd': self.target_ang = -0.5; self.target_lin = 0.0
                    elif data == 's': self.target_lin = 0.0; self.target_ang = 0.0
                conn.close()
            except: break

def main():
    rclpy.init()
    bot = MoveBot()
    rclpy.spin(bot)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
