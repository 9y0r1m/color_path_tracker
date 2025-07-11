# color_path_tracker

Intel RealSense와 OpenCV, ROS 2를 활용해 HSV 색상 기반 경로를 추적하고, Twist 메시지를 퍼블리시하는 자율주행 패키지입니다.

## 실행 방법

ros2 run color_path_tracker color_tracker

실행 시 HSV 트랙바 창이 열리고, 실시간으로 색상 범위를 조정할 수 있습니다.

## 환경 및 의존성

- ROS 2 Humble
- Python 3
- pyrealsense2
- opencv-python
- rclpy
- geometry_msgs

## 주요 기능

- HSV 트랙바 기반 색상 범위 설정
- 가장 큰 색상 영역의 중심 추적
- 중심 위치에 따른 직진, 좌회전, 우회전 제어
- /cmd_vel 토픽으로 Twist 메시지 퍼블리시
