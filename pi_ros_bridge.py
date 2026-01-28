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

        # Ros2 토픽 구독 - Subscription (/image_raw/compressed)
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.listener_callback,
            10)
        self.get_logger().info(f"영상 중계 노드 시작 (윈도우 연결 대기 중...)")

    # 자동 재연결 시스템
    def connect_to_server(self):
        """서버 연결 시도 (실패 시 False 반환)"""
        try:
            # 재연결 시도가 반복될 때, 이전의 죽은 연결이 남아있어 메모리를 잡아먹거나 충돌하는 것 방지(자원정리)
            if self.sock: self.sock.close()

            # 인터넷(IP)을 통해 TCP 방식으로 통신
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

            # 원래 TCP 통신은 데이터를 모아서 한번에 보내려는 성질이 있지만, 데이터가 작더라도 모으지 말고 즉시 발송하라고 설정한 TCP_NODELAY
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

            # 타임아웃 설정 - 연결 시도 중 프로그램 멈춤 현상 방지
            self.sock.settimeout(2.0)  # 노트북이 꺼져있거나 네트워크 끊겼을 때 Freezing 막기. 2초만 시도 후 다음 로직.
            self.sock.connect((self.laptop_ip, self.port))

            # 연결 성공 후 타임아웃 풀기
            self.sock.settimeout(None)

            self.get_logger().info(f"=== ✅ 윈도우({self.laptop_ip}) 연결 성공! ===")
            return True
        except Exception as e:
            # 연결에 실패해도 프로그램이 Error를 띄우고 꺼지지 않고, 단순히 False 반환 -> listener_callback에서 1초 뒤 다시 시작 가능
            self.sock = None
            return False


    # callback 함수
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

        # 2. 프레임 건너뛰기 (부하 조절) - 모든 프레임을 다 보내면 라즈베리 과부하 (3장에 1장만 보내기)
        self.frame_count += 1
        if self.frame_count % self.skip_rate != 0:
            return

        # 3. 데이터 전송
        try:
            data = bytes(msg.data)

            # 패킷 헤더 처리 - 사진의 크기를 알려주는 헤더 (video_thread.py 에서 정확히 해석 가능)
            # >L: 빅 엔디안(4바이트)
            self.sock.sendall(struct.pack(">L", len(data)) + data)
        except (BrokenPipeError, ConnectionResetError):
            self.get_logger().error("❌ 윈도우 연결 끊김. 재연결 대기...")
            self.sock.close()
            self.sock = None  # 다시 None으로 만들어서 1번 로직이 돌게 함
        except Exception as e:
            self.get_logger().warn(f"전송 에러: {e}")


# 프로그램의 생명주기 관리
def main(args=None):
    rclpy.init(args=args) # ros2 통신 시작
    node = RosToSocket()
    try:
        rclpy.spin(node) # 노드가 계속 돎 (카메라 영상)
    except KeyboardInterrupt:
        pass
    finally:
        if node.sock: node.sock.close() # 소켓 끊기
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()