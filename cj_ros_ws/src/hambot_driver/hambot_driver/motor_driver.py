#!/usr/bin/env python3
"""
motor_driver.py — Build HAT motor control + encoder odometry for HamBot.

Supports 2WD and 4WD drivetrains (selectable via 'drivetrain' parameter).

Subscribes /cmd_vel (Twist) → diff-drive kinematics → Build HAT motors.
Publishes /odom (Odometry) and /joint_states (JointState) from encoders.

"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from buildhat import Motor

DRIVE_2WD = '2WD'
DRIVE_4WD = '4WD'


class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')

        # ── Parameters ──
        self.declare_parameter('drivetrain', DRIVE_4WD)
        self.declare_parameter('wheel_separation', 0.199)   # m
        self.declare_parameter('wheel_radius', 0.045)       # m
        self.declare_parameter('max_rpm', 100)
        self.declare_parameter('odom_rate', 20.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.drivetrain = self.get_parameter('drivetrain').value
        self.wheel_sep = self.get_parameter('wheel_separation').value
        self.wheel_rad = self.get_parameter('wheel_radius').value
        self.max_rpm = self.get_parameter('max_rpm').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        odom_rate = self.get_parameter('odom_rate').value

        if self.drivetrain not in (DRIVE_2WD, DRIVE_4WD):
            raise ValueError(f"Invalid drivetrain '{self.drivetrain}'. Must be '{DRIVE_2WD}' or '{DRIVE_4WD}'.")

        # ── Build HAT motors ──
        if self.drivetrain == DRIVE_2WD:
            self.left_motor = Motor('B')
            self.left_motor.set_speed_unit_rpm(rpm=True)
            self.right_motor = Motor('A')
            self.right_motor.set_speed_unit_rpm(rpm=True)
            self._last_left_pos = self.left_motor.get_position()
            self._last_right_pos = self.right_motor.get_position()
        else:
            self.front_left_motor = Motor('C')
            self.front_left_motor.set_speed_unit_rpm(rpm=True)
            self.rear_left_motor = Motor('D')
            self.rear_left_motor.set_speed_unit_rpm(rpm=True)
            self.front_right_motor = Motor('B')
            self.front_right_motor.set_speed_unit_rpm(rpm=True)
            self.rear_right_motor = Motor('A')
            self.rear_right_motor.set_speed_unit_rpm(rpm=True)
            self._last_fl_pos = self.front_left_motor.get_position()
            self._last_rl_pos = self.rear_left_motor.get_position()
            self._last_fr_pos = self.front_right_motor.get_position()
            self._last_rr_pos = self.rear_right_motor.get_position()

        # Cumulative encoder radians
        self._left_rad = 0.0
        self._right_rad = 0.0

        # Odometry state
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0

        # ── ROS wiring ──
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # ── Timer: encoder polling + odom publishing ──
        period = 1.0 / max(odom_rate, 1.0)
        self._timer = self.create_timer(period, self.odom_tick)

        self.get_logger().info(
            f'MotorDriver started. Drivetrain={self.drivetrain}. '
            f'Wheel sep={self.wheel_sep:.3f}m radius={self.wheel_rad:.3f}m'
        )

    # CMD_VEL → MOTOR SPEEDS
    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        left_rad_s = (v - w * self.wheel_sep / 2.0) / self.wheel_rad
        right_rad_s = (v + w * self.wheel_sep / 2.0) / self.wheel_rad

        left_rpm = left_rad_s * 60.0 / (2.0 * math.pi)
        right_rpm = right_rad_s * 60.0 / (2.0 * math.pi)

        left_rpm = max(-self.max_rpm, min(self.max_rpm, left_rpm))
        right_rpm = max(-self.max_rpm, min(self.max_rpm, right_rpm))

        if self.drivetrain == DRIVE_2WD:
            self.left_motor.start(speed=-left_rpm)
            self.right_motor.start(speed=right_rpm)
        else:
            self.front_left_motor.start(speed=-left_rpm)
            self.rear_left_motor.start(speed=-left_rpm)
            self.front_right_motor.start(speed=right_rpm)
            self.rear_right_motor.start(speed=right_rpm)

    # ENCODER ODOMETRY
    @staticmethod
    def _read_delta(motor, last_pos, invert=False):
        """Return (current_raw_pos, delta_radians) with wrap-around."""
        cur = motor.get_position()
        delta_deg = cur - last_pos
        if delta_deg > 180:
            delta_deg -= 360
        elif delta_deg < -180:
            delta_deg += 360
        delta_rad = math.radians(delta_deg)
        if invert:
            delta_rad = -delta_rad
        return cur, delta_rad

    def odom_tick(self):
        now = self.get_clock().now()

        if self.drivetrain == DRIVE_2WD:
            self._last_left_pos, dl = self._read_delta(
                self.left_motor, self._last_left_pos, invert=True)
            self._last_right_pos, dr = self._read_delta(
                self.right_motor, self._last_right_pos, invert=False)
        else:
            self._last_fl_pos, dl_fl = self._read_delta(
                self.front_left_motor, self._last_fl_pos, invert=True)
            self._last_rl_pos, dl_rl = self._read_delta(
                self.rear_left_motor, self._last_rl_pos, invert=True)
            self._last_fr_pos, dr_fr = self._read_delta(
                self.front_right_motor, self._last_fr_pos, invert=False)
            self._last_rr_pos, dr_rr = self._read_delta(
                self.rear_right_motor, self._last_rr_pos, invert=False)
            dl = (dl_fl + dl_rl) / 2.0
            dr = (dr_fr + dr_rr) / 2.0

        self._left_rad += dl
        self._right_rad += dr

        left_disp = dl * self.wheel_rad
        right_disp = dr * self.wheel_rad

        dist = (left_disp + right_disp) / 2.0
        dtheta = (right_disp - left_disp) / self.wheel_sep

        self._theta += dtheta
        self._x += dist * math.cos(self._theta)
        self._y += dist * math.sin(self._theta)

        # ── Publish /odom ──
        q = self._yaw_to_quaternion(self._theta)
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        self._odom_pub.publish(odom)

        # ── Publish /joint_states ──
        js = JointState()
        js.header.stamp = now.to_msg()
        if self.drivetrain == DRIVE_2WD:
            js.name = ['left_wheel_joint', 'right_wheel_joint']
            js.position = [self._left_rad, self._right_rad]
        else:
            js.name = [
                'front_left_wheel_joint', 'rear_left_wheel_joint',
                'front_right_wheel_joint', 'rear_right_wheel_joint'
            ]
            # Publish individual wheel radians (not averaged)
            js.position = [dl_fl, dl_rl, dr_fr, dr_rr]

        self._joint_pub.publish(js)

    @staticmethod
    def _yaw_to_quaternion(yaw):
        half = yaw / 2.0
        return (0.0, 0.0, math.sin(half), math.cos(half))

    def stop_motors(self):
        if self.drivetrain == DRIVE_2WD:
            self.left_motor.stop()
            self.right_motor.stop()
        else:
            self.front_left_motor.stop()
            self.rear_left_motor.stop()
            self.front_right_motor.stop()
            self.rear_right_motor.stop()
        self.get_logger().info('Motors stopped.')

    def destroy_node(self):
        self.stop_motors()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()