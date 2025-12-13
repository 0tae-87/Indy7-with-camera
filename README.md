# 🤖 Indy7-with-Camera (ROS 2 Jazzy)

이 프로젝트는 Neuromeka Robotics의 `indy-ros2` 저장소를 기반으로 Indy7 로봇에 카메라 시뮬레이션 및 기능을 통합한 맞춤형 ROS 2 패키지입니다.

## 📌 프로젝트 기원 (Original Source)

본 프로젝트의 모든 핵심 내용은 다음 저장소에서 포크(Fork)되었습니다.

- **Original Repository:** [https://github.com/neuromeka-robotics/indy-ros2](https://github.com/neuromeka-robotics/indy-ros2)

## ✨ 주요 변경 및 통합 사항

| 카테고리 | 상세 내용 |
| :--- | :--- |
| **버그 수정 (Bug Fix)** | `indy_macro.xacro` 파일 내의 `xacro` 변수 오타('perfix' $\rightarrow$ 'prefix')를 수정하여 launch 파일 실행 오류를 해결했습니다. |
| **시뮬레이션** | Gazebo 시뮬레이션 환경에 가상 카메라를 통합했습니다. |
| **로봇 모델** | Indy7 로봇 모델의 **`link6`** (로봇 손목)에 가상 카메라를 부착하도록 로봇 디스크립션 파일(`xacro`)을 수정했습니다. |
| **토픽 통신** | Gazebo 시뮬레이션에서 생성되는 카메라의 이미지 토픽을 ROS 2 시스템으로 구독(Subscribing image topic)할 수 있도록 브릿지(Bridge)를 설정했습니다. |
