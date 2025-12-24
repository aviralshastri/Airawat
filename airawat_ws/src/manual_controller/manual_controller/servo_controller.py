#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray, Float32MultiArray


class DriveController(Node):
    def __init__(self):
        super().__init__('drive_controller')
        
        # Declare parameters
        self.declare_parameter('topic_sequence', ['back_left', 'back_right', 'front_left', 'front_right'])
        self.declare_parameter('throttle_range', [-1.0, 1.0])
        self.declare_parameter('topic_name', 'motor_control')
        self.declare_parameter('pwm_range', [1000, 2000])
        self.declare_parameter('channels_sequence', ['pitch', 'throttle', 'yaw', 'roll', 'var', 'mode'])
        self.declare_parameter('rc_topic', 'rc_input')
        
        # Get parameters
        self.motor_sequence = self.get_parameter('topic_sequence').value
        self.throttle_range = self.get_parameter('throttle_range').value
        self.topic_name = self.get_parameter('topic_name').value
        self.pwm_range = self.get_parameter('pwm_range').value
        self.channels_sequence = self.get_parameter('channels_sequence').value
        rc_topic = self.get_parameter('rc_topic').value
        
        # Find indices for roll and pitch in channels_sequence
        try:
            self.roll_idx = self.channels_sequence.index('roll')
            self.pitch_idx = self.channels_sequence.index('pitch')
        except ValueError as e:
            self.get_logger().error(f'roll or pitch not found in channels_sequence: {e}')
            raise
        
        # PWM neutral point
        self.pwm_neutral = (self.pwm_range[0] + self.pwm_range[1]) / 2.0
        
        # Publisher for motor control
        self.motor_pub = self.create_publisher(Float32MultiArray, self.topic_name, 10)
        
        # Subscriber for RC input
        self.rc_sub = self.create_subscription(
            UInt16MultiArray,
            rc_topic,
            self.rc_callback,
            10
        )
        
        self.get_logger().info(f'Drive controller initialized')
        self.get_logger().info(f'Subscribing to RC: {rc_topic}')
        self.get_logger().info(f'Publishing to motors: {self.topic_name}')
        self.get_logger().info(f'Motor sequence: {self.motor_sequence}')
        self.get_logger().info(f'Roll index: {self.roll_idx}, Pitch index: {self.pitch_idx}')
    
    def pwm_to_normalized(self, pwm_value):
        """
        Convert PWM value to normalized range [-1.0, 1.0]
        PWM range: [min, max] -> [-1.0, 1.0]
        Neutral (center) -> 0.0
        """
        # Clamp PWM to valid range
        pwm_value = max(self.pwm_range[0], min(self.pwm_range[1], pwm_value))
        
        # Normalize to [-1.0, 1.0] with neutral at center
        normalized = (pwm_value - self.pwm_neutral) / (self.pwm_range[1] - self.pwm_neutral)
        
        return normalized
    
    def clamp_throttle(self, value):
        """Clamp throttle to specified range"""
        return max(self.throttle_range[0], min(self.throttle_range[1], value))
    
    def calculate_differential_drive(self, pitch, roll):
        """
        Calculate differential drive motor speeds
        pitch: forward/backward input [-1.0, 1.0]
        roll: left/right input [-1.0, 1.0]
        
        Returns: [back_left, back_right, front_left, front_right]
        """
        # Differential drive calculation
        # pitch: positive = forward, negative = backward
        # roll: positive = turn right, negative = turn left
        
        left_speed = pitch - roll
        right_speed = pitch + roll
        
        # Clamp to throttle range
        left_speed = self.clamp_throttle(left_speed)
        right_speed = self.clamp_throttle(right_speed)
        
        # Map to motor sequence: [back_left, back_right, front_left, front_right]
        motor_speeds = []
        for motor_name in self.motor_sequence:
            if 'left' in motor_name:
                motor_speeds.append(left_speed)
            elif 'right' in motor_name:
                motor_speeds.append(right_speed)
            else:
                motor_speeds.append(0.0)
        
        return motor_speeds
    
    def rc_callback(self, msg):
        """Process RC input and publish motor commands"""
        # Check if we have enough channels
        if len(msg.data) <= max(self.roll_idx, self.pitch_idx):
            self.get_logger().warn(
                f'Not enough channels in RC data. Expected at least {max(self.roll_idx, self.pitch_idx) + 1}, got {len(msg.data)}'
            )
            return
        
        # Extract roll and pitch PWM values
        roll_pwm = msg.data[self.roll_idx]
        pitch_pwm = msg.data[self.pitch_idx]
        
        # Convert PWM to normalized values [-1.0, 1.0]
        roll = self.pwm_to_normalized(roll_pwm)
        pitch = self.pwm_to_normalized(pitch_pwm)
        
        # Calculate differential drive motor speeds
        motor_speeds = self.calculate_differential_drive(pitch, roll)
        
        # Publish motor commands
        motor_msg = Float32MultiArray()
        motor_msg.data = motor_speeds
        self.motor_pub.publish(motor_msg)
        
        # Debug logging (uncomment if needed)
        # self.get_logger().info(f'RC: pitch={pitch_pwm}, roll={roll_pwm} | Motors: {[f"{s:.2f}" for s in motor_speeds]}')


def main(args=None):
    rclpy.init(args=args)
    node = DriveController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()