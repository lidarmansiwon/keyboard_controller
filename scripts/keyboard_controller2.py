#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import threading
import select
import time
from mk3_msgs.msg import GuidanceType, NavigationType, ControlType
from rclpy.qos import QoSProfile, qos_profile_sensor_data

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')

        # PWM 초기값
        self.left_pwm = 1500
        self.right_pwm = 1500

        # PWM 범위
        self.pwm_min = 1250
        self.pwm_max = 1650

        # 감가속 설정
        self.acceleration = 30  # 가속시 변화량
        self.deceleration = 40  # 감속시 변화량

        # 현재 속도 상태 (PWM 변화량)
        self.left_speed = 0
        self.right_speed = 0

        # Publisher       
        self.control_publisher      = self.create_publisher(ControlType, '/orin1/control', qos_profile_sensor_data)

        # 키보드 입력 스레드
        self.last_input_time = time.time()
        self.current_command = None
        thread = threading.Thread(target=self.keyboard_listener)
        thread.daemon = True
        thread.start()

        # 주기적으로 PWM 업데이트 및 퍼블리시
        self.timer = self.create_timer(0.05, self.update_pwm)

    def update_pwm(self):
        # 감속 처리
        if time.time() - self.last_input_time > 0.1:
            self.apply_deceleration()

        # PWM에 속도 반영
        self.right_pwm = self.clamp_pwm(1500 + self.left_speed)
        self.left_pwm = self.clamp_pwm(1500 + self.right_speed)

        # publish
        control_data                   = ControlType()
        control_data.delta_psi         = round(0.0,2)
        control_data.delta_u           = round(0.0,2)
        control_data.thruster_pwm_port = round(float(self.left_pwm),2)
        control_data.thruster_pwm_stbd = round(float(self.right_pwm),2)
    
        self.control_publisher.publish(control_data)

        self.get_logger().info(f'Left PWM: {self.left_pwm} | Right PWM: {self.right_pwm}')

    def apply_deceleration(self):
        # 좌측 감속
        if self.left_speed > 0:
            self.left_speed = max(0, self.left_speed - self.deceleration)
        elif self.left_speed < 0:
            self.left_speed = min(0, self.left_speed + self.deceleration)

        # 우측 감속
        if self.right_speed > 0:
            self.right_speed = max(0, self.right_speed - self.deceleration)
        elif self.right_speed < 0:
            self.right_speed = min(0, self.right_speed + self.deceleration)

    def clamp_pwm(self, value):
        return max(self.pwm_min, min(self.pwm_max, value))

    def keyboard_listener(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setcbreak(fd)
            while rclpy.ok():
                if self.is_input_available():
                    key = sys.stdin.read(1)
                    self.last_input_time = time.time()
                    self.process_key(key)
                else:
                    rclpy.spin_once(self, timeout_sec=0.1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def is_input_available(self):
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    def process_key(self, key):
        if key in ['w', '\x1b[A']:  # 전진
            self.left_speed += self.acceleration
            self.right_speed += self.acceleration

        elif key in ['s', '\x1b[B']:  # 후진
            self.left_speed -= self.acceleration
            self.right_speed -= self.acceleration

        elif key in ['a', '\x1b[D']:  # 좌회전
            self.left_speed -= self.acceleration
            self.right_speed += self.acceleration

        elif key in ['d', '\x1b[C']:  # 우회전
            self.left_speed += self.acceleration
            self.right_speed -= self.acceleration

        elif key == ' ':  # 정지
            self.left_speed = 0
            self.right_speed = 0
            self.left_pwm = 1500
            self.right_pwm = 1500
            self.get_logger().info('Stop: Reset PWM and speed to 0.')

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt detected. Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
