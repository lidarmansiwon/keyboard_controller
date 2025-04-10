# 🚤 ROS2 Keyboard Control Node

키보드 입력으로 선박 PWM 스러스터를 제어하는 ROS2 패키지입니다.  
Real-time keyboard control for ship thrusters via ROS2.

## 📦 주요 기능 — Features

- 실시간 키보드 제어  
  Real-time keyboard control
- 가속 및 감속 적용  
  Acceleration & deceleration handling
- 복합 키 입력 (W+A, S+D 등) 지원  
  Multi-key input support (e.g., W+A, S+D)
- ROS2 토픽 `/orin1/control` 퍼블리시  
  Publishes to ROS2 topic `/orin1/control`
- 입력 없을 시 자동 감속  
  Auto-deceleration when no input

## 🛠️ 설치 및 실행 — Installation & Run

### 1. 패키지 빌드  
Build the package

```bash
cd ~/ros2_ws
colcon build --packages-select [패키지명]
source install/setup.bash
```

### 2. 노드 실행
Run the node

```bash
ros2 run [패키지명] keyboard_control_node
```

### ⌨️ 키 입력 — Key Commands
키 — Key	동작 — Action
W	전진 — Forward
S	후진 — Backward
A	좌회전 — Turn left
D	우회전 — Turn right

## 🧩 토픽 상세 — Topic Details
/orin1/control

메시지 타입 — Message type: ControlType

주요 필드 — Main fields:

thruster_pwm_port (좌현 스러스터 — Port thruster PWM)

thruster_pwm_stbd (우현 스러스터 — Starboard thruster PWM)

## 🗂️ 코드 설명 — Code Overview
keyboard_control_node.py 가 메인 코드입니다.
The main code is in keyboard_control_node.py.

주요 구성 — Main Components
PWM 초기값 및 범위 — PWM Initialization & Range

초기값: 1500 (중립 — Neutral)

최소값: 1250

최대값: 1650

가속/감속 설정 — Acceleration/Deceleration Settings

가속량: 30

감속량: 40

키보드 입력 스레드 — Keyboard Listener Thread

비동기 키 입력 감지

복합 키(W+A, S+D 등) 지원

PWM 업데이트 및 퍼블리시 — PWM Update & Publish

/orin1/control 토픽에 주기적으로 PWM 값을 퍼블리시

자동 감속 — Auto-deceleration

입력 없을 시 속도 단계적으로 감소

## 📄 참고 — Notes
PWM 범위: 1250 ~ 1650
PWM range: 1250 ~ 1650

기본 PWM: 1500 (중립 — Neutral)
Default PWM: 1500 (neutral)

감가속 값:

가속 — Acceleration: 30

감속 — Deceleration: 40
