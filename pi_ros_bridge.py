import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import socket
import struct
import sys  # 종료를 위해 추가


class RosToSocket(Node):
    def __init__(self):
        super().__init__('ros_to_socket')

        self.declare_parameter('laptop_ip', '192.168.0.3')
        self.declare_parameter('port', 8485)

        self.laptop_ip = self.get_parameter('laptop_ip').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value

        self.sock = None
        self.connect_to_server()

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.listener_callback,
            10)
        self.get_logger().info("Compressed 토픽 수신 및 중계 시작...")

    def connect_to_server(self):
        """서버 연결 시도 함수"""
        try:
            if self.sock:
                self.sock.close()

            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.sock.settimeout(5.0)
            self.sock.connect((self.laptop_ip, self.port))
            self.get_logger().info(f"=== 노트북 서버({self.laptop_ip}) 연결 성공! ===")
            self.sock.settimeout(None)
            return True
        except Exception as e:
            self.get_logger().error(f"연결 실패: {e}")
            self.sock = None
            return False

    def listener_callback(self, msg):
        if self.sock is None:
            return  # 연결 없으면 전송 시도 안 함

        try:
            data = bytes(msg.data)
            # 데이터 전송 시도
            self.sock.sendall(struct.pack(">L", len(data)) + data)

        except (BrokenPipeError, ConnectionResetError) as e:
            # [핵심 수정] 연결이 끊긴 경우
            self.get_logger().error(f"서버와 연결이 끊어졌습니다: {e}")
            self.sock.close()
            self.sock = None

            # 선택 1: 프로그램 완전 종료 (추천)
            # rclpy.shutdown()이 호출되도록 유도
            sys.exit(0)

            # 선택 2: 재연결 시도 (원하면 주석 해제하여 사용)
            # self.get_logger().info("재연결 시도 중...")
            # self.connect_to_server()

        except Exception as e:
            self.get_logger().warn(f"기타 전송 에러: {e}")


def main(args=None):
    rclpy.init(args=args)
    ros_to_socket = RosToSocket()

    try:
        rclpy.spin(ros_to_socket)
    except SystemExit:
        # sys.exit() 호출 시 깔끔하게 종료
        ros_to_socket.get_logger().info("프로그램을 종료합니다.")
    except KeyboardInterrupt:
        pass
    finally:
        if ros_to_socket.sock:
            ros_to_socket.sock.close()
        ros_to_socket.destroy_node()
        # rclpy가 이미 종료되었는지 확인 후 shutdown
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()