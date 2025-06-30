import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import pyrealsense2 as rs

class ColorPathTracker(Node):
    def __init__(self):
        super().__init__('color_path_tracker')

        # RealSense 파이프라인 설정
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        try:
            self.pipeline.start(self.config)
            self.get_logger().info("RealSense 파이프라인 시작됨.")
        except Exception as e:
            self.get_logger().error(f"RealSense 파이프라인 시작 실패: {e}")
            rclpy.shutdown()
            return

        # HSV 트랙바 설정
        cv2.namedWindow("HSV")
        for name, val, maxval in [("H_low", 80, 179), ("H_high", 100, 179),
                                  ("S_low", 100, 255), ("S_high", 255, 255),
                                  ("V_low", 100, 255), ("V_high", 255, 255)]:
            cv2.createTrackbar(name, "HSV", val, maxval, lambda x: None)

        self.timer = self.create_timer(0.03, self.process_frame)  # 약 30 FPS

    def process_frame(self):
        try:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                self.get_logger().warning("컬러 프레임을 가져오지 못했습니다.")
                return

            frame = np.asanyarray(color_frame.get_data())

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower = np.array([cv2.getTrackbarPos("H_low", "HSV"),
                              cv2.getTrackbarPos("S_low", "HSV"),
                              cv2.getTrackbarPos("V_low", "HSV")])
            upper = np.array([cv2.getTrackbarPos("H_high", "HSV"),
                              cv2.getTrackbarPos("S_high", "HSV"),
                              cv2.getTrackbarPos("V_high", "HSV")])

            mask = cv2.inRange(hsv, lower, upper)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            overlay = frame.copy()
            cv2.drawContours(overlay, contours, -1, (0, 255, 0), thickness=cv2.FILLED)
            combined = cv2.addWeighted(frame, 0.7, overlay, 0.3, 0)

            cv2.imshow("Original", frame)
            cv2.imshow("Mask", mask)
            cv2.imshow("Filled Region", combined)

            if cv2.waitKey(1) & 0xFF == 27:  # ESC 키로 종료
                self.cleanup()

        except Exception as e:
            self.get_logger().error(f"프레임 처리 중 오류 발생: {e}")
            self.cleanup()

    def cleanup(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ColorPathTracker()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
