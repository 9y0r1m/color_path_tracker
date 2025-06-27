import rclpy
from rclpy.node import Node
import cv2
import numpy as np

class ColorPathTracker(Node):
    def __init__(self):
        super().__init__('color_path_tracker')

        self.cap = cv2.VideoCapture(0)
        cv2.namedWindow("HSV")
        for name, val, maxval in [("H_low",80,179),("H_high",100,179),
                                  ("S_low",100,255),("S_high",255,255),
                                  ("V_low",100,255),("V_high",255,255)]:
            cv2.createTrackbar(name, "HSV", val, maxval, lambda x: None)

        self.timer = self.create_timer(0.03, self.process_frame)  # 약 30FPS

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array([cv2.getTrackbarPos("H_low","HSV"),
                          cv2.getTrackbarPos("S_low","HSV"),
                          cv2.getTrackbarPos("V_low","HSV")])
        upper = np.array([cv2.getTrackbarPos("H_high","HSV"),
                          cv2.getTrackbarPos("S_high","HSV"),
                          cv2.getTrackbarPos("V_high","HSV")])

        mask = cv2.inRange(hsv, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        overlay = frame.copy()
        cv2.drawContours(overlay, contours, -1, (0,255,0), thickness=cv2.FILLED)
        combined = cv2.addWeighted(frame, 0.7, overlay, 0.3, 0)

        cv2.imshow("Original", frame)
        cv2.imshow("Mask", mask)
        cv2.imshow("Filled Region", combined)

        if cv2.waitKey(1) & 0xFF == 27:
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ColorPathTracker()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
