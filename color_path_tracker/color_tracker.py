import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import pyrealsense2 as rs
from geometry_msgs.msg import Twist
import math

class ColorPathTracker(Node):
    def __init__(self):
        super().__init__('color_path_tracker')

        # RealSense 설정
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        try:
            self.pipeline.start(self.config)
            self.get_logger().info("RealSense pipeline started.")
        except Exception as e:
            self.get_logger().error(f"Failed to start RealSense pipeline: {e}")
            rclpy.shutdown()
            return

        # HSV 트랙바 설정
        cv2.namedWindow("HSV")
        for name, val, maxval in [("H_low", 80, 179), ("H_high", 100, 179),
                                  ("S_low", 100, 255), ("S_high", 255, 255),
                                  ("V_low", 100, 255), ("V_high", 255, 255)]:
            cv2.createTrackbar(name, "HSV", val, maxval, lambda x: None)

        # Twist 퍼블리셔
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 타이머
        self.timer = self.create_timer(0.03, self.process_frame)

    def process_frame(self):
        try:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                self.get_logger().warning("No color frame received.")
                return

            frame = np.asanyarray(color_frame.get_data())
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # HSV 범위 지정
            lower = np.array([cv2.getTrackbarPos("H_low", "HSV"),
                              cv2.getTrackbarPos("S_low", "HSV"),
                              cv2.getTrackbarPos("V_low", "HSV")])
            upper = np.array([cv2.getTrackbarPos("H_high", "HSV"),
                              cv2.getTrackbarPos("S_high", "HSV"),
                              cv2.getTrackbarPos("V_high", "HSV")])

            mask = cv2.inRange(hsv, lower, upper)

            height, width = mask.shape
            roi = mask[height // 2:, :]  # 하단 절반만

            contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            direction = "STOP"
            angle_deg = 0.0

            if contours:
                largest = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"]) + height // 2  # ROI 보정

                    frame_center = width // 2
                    threshold = 30

                    if cx < frame_center - threshold:
                        self.publish_cmd(0.1, 0.3)
                        direction = "LEFT"
                    elif cx > frame_center + threshold:
                        self.publish_cmd(0.1, -0.3)
                        direction = "RIGHT"
                    else:
                        self.publish_cmd(0.2, 0.0)
                        direction = "FORWARD"

                    # 각도 계산
                    dx = cx - frame_center
                    dy = 50  # 중심선 길이와 동일
                    angle_rad = math.atan2(dx, dy)
                    angle_deg = np.degrees(angle_rad)

            else:
                self.publish_cmd(0.0, 0.0)

            # 경로 강조
            highlight = np.zeros_like(frame)
            highlight[height//2:, :][roi > 0] = (0, 255, 0)
            combined = cv2.addWeighted(frame, 0.7, highlight, 0.3, 0)

            # 선 시각화
            cv2.line(combined, (width//2, height), (width//2, height - 50), (0, 0, 255), 2)
            if contours and M["m00"] != 0:
                cv2.line(combined, (cx, height), (cx, height - 50), (0, 255, 0), 2)

            # 텍스트 표시
            cv2.putText(combined, f"Direction: {direction}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(combined, f"Angle: {angle_deg:.1f} deg", (10, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

            # 출력
            cv2.imshow("Mask", mask)
            cv2.imshow("Path View", combined)

            if cv2.waitKey(1) & 0xFF == 27:
                self.cleanup()

        except Exception as e:
            self.get_logger().error(f"Error during frame processing: {e}")
            self.cleanup()

    def publish_cmd(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

    def cleanup(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()
        self.publish_cmd(0.0, 0.0)
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ColorPathTracker()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
