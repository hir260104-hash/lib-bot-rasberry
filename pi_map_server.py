import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import socket
import threading
import numpy as np
import cv2
import struct


class MapServer(Node):
    def __init__(self):
        super().__init__('map_to_image_server')

        # 1. 지도 데이터 구독 (/map)
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)

        self.current_map_img = None  # 전송할 지도 이미지

        # 2. 통신 서버 설정 (포트 9996)
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind(('0.0.0.0', 9996))
        self.server_sock.listen(1)

        threading.Thread(target=self.start_listening, daemon=True).start()
        self.get_logger().info("=== 지도(SLAM) 서버 대기 중 (Port: 9996) ===")

    def map_callback(self, msg):
        """ ROS 지도 데이터(-1, 0, 100)를 -> OpenCV 이미지로 변환 """
        width = msg.info.width
        height = msg.info.height

        # [수정] 지도가 아직 생성되지 않았거나 크기가 0이면 무시 (에러 방지)
        if width <= 0 or height <= 0:
            return

            # 데이터 개수와 크기가 맞는지 확인
        if len(msg.data) != width * height:
            return

        try:
            data = np.array(msg.data).reshape((height, width))

            # 데이터 변환 로직
            # -1 (미탐험) -> 127 (회색)
            # 0 (빈 공간) -> 255 (흰색)
            # 100 (벽) -> 0 (검은색)

            img = np.full((height, width), 127, dtype=np.uint8)  # 전체 회색
            img[data == 0] = 255  # 갈 수 있는 곳은 흰색
            img[data == 100] = 0  # 벽은 검은색

            # 보기 좋게 상하 반전 (ROS 좌표계 -> 이미지 좌표계 보정)
            img = cv2.flip(img, 0)

            # 압축해서 저장 (전송 준비)
            _, encoded_img = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
            self.current_map_img = encoded_img.tobytes()

        except Exception as e:
            self.get_logger().warn(f"지도 변환 중 에러: {e}")

    def start_listening(self):
        while rclpy.ok():
            try:
                conn, addr = self.server_sock.accept()
                with conn:
                    while True:
                        req = conn.recv(1024).decode().strip()
                        if not req: break

                        if req == "MAP":
                            if self.current_map_img is not None:
                                # [데이터 길이] + [이미지 데이터] 전송
                                size = len(self.current_map_img)
                                conn.sendall(struct.pack(">L", size) + self.current_map_img)
                            else:
                                # 지도가 아직 없으면 길이 0 전송
                                conn.sendall(struct.pack(">L", 0))
            except Exception as e:
                pass  # 연결 끊김 등은 무시하고 대기


def main():
    rclpy.init()
    node = MapServer()
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