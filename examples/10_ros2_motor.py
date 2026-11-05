#!/usr/bin/env python3
"""One HLS servo ROS 2 node. Speed commands and joint velocity are rad/s.

Run after sourcing ROS 2 and installing this library:
  python3 examples/10_ros2_motor.py --ros-args -p servo_id:=1 -p baudrate:=38400

speed: std_msgs/Float64, desired speed in rad/s
joint_states: sensor_msgs/JointState, position in rad, velocity in rad/s
motor_voltage: std_msgs/Float64, servo supply voltage in volts
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from feetech_hls_rpi import HLSMotor, HLSMotorError


class MotorNode(Node):
    def __init__(self):
        super().__init__('feetech_motor')
        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baudrate', 38400)
        self.declare_parameter('servo_id', 1)
        self.declare_parameter('joint_name', 'motor_joint')
        self.declare_parameter('acceleration', 5)
        self.declare_parameter('current_limit_raw', 200)

        self.joint_name = self.get_parameter('joint_name').value
        self.acceleration = self.get_parameter('acceleration').value
        self.current_limit_raw = self.get_parameter('current_limit_raw').value
        self.motor = HLSMotor(
            port=self.get_parameter('port').value,
            baudrate=self.get_parameter('baudrate').value,
            servo_id=self.get_parameter('servo_id').value,
            stop_on_close=False,
        )
        self.command_attempted = False
        try:
            self.motor.connect(ping=True)
        except Exception:
            self.motor.close()
            raise

        self.joint_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.voltage_publisher = self.create_publisher(Float64, 'motor_voltage', 10)
        self.speed_subscriber = self.create_subscription(Float64, 'speed', self.speed_callback, 10)
        self.encoder_timer = self.create_timer(0.01, self.encoder_callback)
        self.status_timer = self.create_timer(1.0, self.status_callback)
        self.get_logger().info('Motor connected; speed commands are in rad/s')

    def speed_callback(self, message):
        if not math.isfinite(message.data):
            self.get_logger().error('Ignoring non-finite speed command')
            return
        try:
            # Track attempts too: a communication error may occur after the motor
            # has already accepted a command.
            self.command_attempted = True
            self.motor.set_wheel_rpm(
                message.data * 60.0 / (2.0 * math.pi),
                acceleration=self.acceleration,
                torque=self.current_limit_raw,
            )
        except (HLSMotorError, OSError, ValueError) as exc:
            self.get_logger().error(f'Speed command failed: {exc}')

    def encoder_callback(self):
        try:
            sample = self.motor.read_encoder(include_speed=True, direction_policy='measured')
            if not sample.valid:
                self.get_logger().warning(f'Invalid encoder sample: {sample.glitch_reason}')
                return
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = [self.joint_name]
            # The unwrapped position starts at zero when the node starts.
            message.position = [sample.continuous_revolutions * 2.0 * math.pi]
            message.velocity = [sample.speed_rpm * 2.0 * math.pi / 60.0]
            self.joint_publisher.publish(message)
        except (HLSMotorError, OSError, ValueError) as exc:
            self.get_logger().error(f'Encoder read failed: {exc}')

    def status_callback(self):
        try:
            status = self.motor.read_status()
            voltage = Float64()
            voltage.data = float(status.voltage_v)
            self.voltage_publisher.publish(voltage)
            self.get_logger().info(
                f'Voltage={status.voltage_v:.1f} V, current={status.current_a:.3f} A, '
                f'temperature={status.temperature_c} C, speed={status.speed_rpm:.1f} rpm, '
                f'load={status.load_fraction:.3f}, moving={status.moving}, '
                f'status=0x{status.status_byte:02x}'
            )
        except (HLSMotorError, OSError, ValueError) as exc:
            self.get_logger().error(f'Motor status read failed: {exc}')

    def close_motor(self):
        if self.command_attempted:
            try:
                self.motor.stop(retries=3)
            except (HLSMotorError, OSError, ValueError) as exc:
                self.get_logger().error(f'Could not stop motor: {exc}')
        self.motor.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MotorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close_motor()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
