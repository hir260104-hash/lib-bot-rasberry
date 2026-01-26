import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import socket
import struct
import sys
import time  # [추가] 시간 체크용


class RosToSocket(Node):
    def __init__(self):
        super().__init__('ros_to_socket')

        self.declare_parameter('laptop_ip', '192.168.0.3')  # 노트북 IP 확인 필요
        self.declare_parameter('port', 8485)

        self.laptop_ip = self.get_parameter('laptop_ip').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value

        self.sock = None
        self.last_connect_attempt = 0  # 마지막 연결 시도 시간
        self.frame_count = 0
        self.skip_rate = 3

        # 시작하자마자 연결 시도 (실패해도 죽지 않음)
        self.connect_to_server()

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.listener_callback,
            10)
        self.get_logger().info(f"영상 중계 노드 시작 (윈도우 연결 대기 중...)")

    def connect_to_server(self):
        """서버 연결 시도 (실패 시 False 반환)"""
        try:
            if self.sock: self.sock.close()

            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.sock.settimeout(2.0)  # 2초 동안만 기다림
            self.sock.connect((self.laptop_ip, self.port))
            self.sock.settimeout(None)

            self.get_logger().info(f"=== ✅ 윈도우({self.laptop_ip}) 연결 성공! ===")
            return True
        except Exception as e:
            # 실패하면 조용히 넘어감 (로그는 너무 자주 찍히면 지저분하므로 생략 가능)
            self.sock = None
            return False

    def listener_callback(self, msg):
        # 1. 연결이 안 되어 있다면? -> 재연결 시도
        if self.sock is None:
            current_time = time.time()
            # 1초에 한 번만 재연결 시도 (CPU 과부하 방지)
            if current_time - self.last_connect_attempt > 1.0:
                self.last_connect_attempt = current_time
                if not self.connect_to_server():
                    print(f"[{current_time:.0f}] 윈도우 연결 찾는 중...", end='\r')
            return  # 연결 없으니 이번 프레임은 버림

        # 2. 프레임 건너뛰기 (부하 조절)
        self.frame_count += 1
        if self.frame_count % self.skip_rate != 0:
            return

        # 3. 데이터 전송
        try:
            data = bytes(msg.data)
            self.sock.sendall(struct.pack(">L", len(data)) + data)
        except (BrokenPipeError, ConnectionResetError):
            self.get_logger().error("❌ 윈도우 연결 끊김. 재연결 대기...")
            self.sock.close()
            self.sock = None  # 다시 None으로 만들어서 1번 로직이 돌게 함
        except Exception as e:
            self.get_logger().warn(f"전송 에러: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = RosToSocket()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.sock: node.sock.close()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()