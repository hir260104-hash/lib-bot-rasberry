import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage # Image 대신 CompressedImage 사용
import socket
import struct

class RosToSocket(Node):
    def __init__(self):
        super().__init__('ros_to_socket')
        
        # 1. 파라미터 선언
        self.declare_parameter('laptop_ip', '192.168.0.3')
        self.declare_parameter('port', 8485)

        self.laptop_ip = self.get_parameter('laptop_ip').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        
        # 2. 소켓 연결 (TCP_NODELAY로 지연 최소화)
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.sock.settimeout(5.0) 
            self.sock.connect((self.laptop_ip, self.port))
            self.get_logger().info(f"=== 노트북 서버({self.laptop_ip}) 연결 성공! ===")
            self.sock.settimeout(None)
        except Exception as e:
            self.get_logger().error(f"연결 실패: {e}")
            exit()

        # 3. 압축된 이미지 토픽 구독
        # 카메라 노드에서 이미 압축된 데이터를 보내므로 파이 CPU는 놀게 됩니다.
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.listener_callback,
            10)
        self.get_logger().info("Compressed 토픽 수신 및 단순 중계 시작...")

    def listener_callback(self, msg):
        try:
            # msg.data 자체가 이미 .jpg 바이트 데이터입니다.
            # pickle을 쓰지 않고 원본 바이트를 그대로 보냅니다 (속도 최적화).
            data = bytes(msg.data)
            
            # [헤더: 데이터 크기] + [바디: 실제 데이터]
            self.sock.sendall(struct.pack(">L", len(data)) + data)
            
        except Exception as e:
            self.get_logger().warn(f"전송 에러: {e}")

def main(args=None):
    rclpy.init(args=args)
    ros_to_socket = RosToSocket()
    try:
        rclpy.spin(ros_to_socket)
    except KeyboardInterrupt:
        pass
    finally:
        if ros_to_socket.sock:
            ros_to_socket.sock.close()
        ros_to_socket.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
