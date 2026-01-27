import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import socket
import threading
import json
import math


# 3차원 방향(쿼터니언)을 우리가 아는 각도(360도)로 바꾸는 공식
def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * z + x * y)
    t1 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t0, t1)
    return yaw


class OdomServer(Node):
    def __init__(self):
        super().__init__('odom_server')

        # 1. 로봇 위치(/odom) 데이터 구독
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            10)

        # 현재 위치 저장 변수 (초기값 0)
        self.current_pos = {'x': 0.0, 'y': 0.0, 'theta': 0.0}

        # 2. 소켓 서버 설정 (포트 9997)
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind(('0.0.0.0', 9997))  # 누구나 접속 가능
        self.server_sock.listen(1)

        # 별도 스레드에서 통신 대기 (ROS 동작 방해 안 하려고)
        threading.Thread(target=self.start_listening, daemon=True).start()
        self.get_logger().info("=== 위치(Odom) 서버 대기 중 (Port: 9997) ===")

    def listener_callback(self, msg):
        # 로봇에게서 위치 정보가 오면 변수에 저장
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation

        # 방향 계산 (라디안 -> 각도)
        theta_rad = quaternion_to_euler(o.x, o.y, o.z, o.w)
        theta_deg = math.degrees(theta_rad)  # 보기 편하게 도(degree)로 변환

        self.current_pos = {
            'x': round(p.x, 2),  # 미터 단위 (소수점 2자리)
            'y': round(p.y, 2),
            'theta': round(theta_deg, 1)  # 각도
        }

    def start_listening(self):
        while rclpy.ok():
            try:
                conn, addr = self.server_sock.accept()
                with conn:
                    while True:
                        data = conn.recv(1024).decode().strip()
                        if not data: break

                        # 윈도우가 "POS" 라고 물어보면 -> 좌표를 줌
                        if data == "POS":
                            msg = json.dumps(self.current_pos)
                            conn.sendall(msg.encode())
            except Exception:
                pass


def main():
    rclpy.init()
    node = OdomServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()