#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from adafruit_servokit import ServoKit
import sys


class DriveInterface(Node):
    def __init__(self):
        super().__init__('drive_interface')
        
        # Declare parameters
        self.declare_parameter('front_left', 0)
        self.declare_parameter('front_right', 1)
        self.declare_parameter('back_left', 2)
        self.declare_parameter('back_right', 3)
        self.declare_parameter('servo_min_pwm', 1000)
        self.declare_parameter('servo_max_pwm', 2000)
        self.declare_parameter('topic_sequence', ['left_back', 'right_back', 'left_front', 'right_front'])
        self.declare_parameter('topic_name', 'motor_control')
        
        # Get parameters
        self.motor_indices = {
            'front_left': self.get_parameter('front_left').value,
            'front_right': self.get_parameter('front_right').value,
            'back_left': self.get_parameter('back_left').value,
            'back_right': self.get_parameter('back_right').value
        }
        
        self.min_pwm = self.get_parameter('servo_min_pwm').value
        self.max_pwm = self.get_parameter('servo_max_pwm').value
        self.topic_sequence = self.get_parameter('topic_sequence').value
        topic_name = self.get_parameter('topic_name').value
        
        # Initialize ServoKit (PCA9685 - 16 channels)
        try:
            self.kit = ServoKit(channels=16)
            
            # Configure each motor as standard servo with PWM range
            for motor_name, channel in self.motor_indices.items():
                self.kit.servo[channel].set_pulse_width_range(self.min_pwm, self.max_pwm)
                # Initialize to neutral position (90 degrees)
                self.kit.servo[channel].angle = 90
            
            self.get_logger().info('ServoKit initialized successfully')
            self.get_logger().info(f'Motor mapping: {self.motor_indices}')
            self.get_logger().info(f'Topic sequence: {self.topic_sequence}')
            self.get_logger().info(f'PWM range: {self.min_pwm} to {self.max_pwm}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize ServoKit: {e}')
            sys.exit(1)
        
        # Create mapping from topic sequence to motor channels
        self.sequence_to_channel = self._create_sequence_mapping()
        
        # Subscribe to motor control topic
        self.subscription = self.create_subscription(
            Float32MultiArray,
            topic_name,
            self.motor_callback,
            10
        )
        
        self.get_logger().info(f'Subscribed to {topic_name}')
        self.get_logger().info(f'Channel mapping: {self.sequence_to_channel}')
    
    def _create_sequence_mapping(self):
        """Create mapping from topic sequence index to motor channel"""
        mapping = {}
        
        # Map topic sequence names to actual motor channel indices
        name_to_channel = {
            'back_left': self.motor_indices['back_left'],
            'back_right': self.motor_indices['back_right'],
            'front_left': self.motor_indices['front_left'],
            'front_right': self.motor_indices['front_right']
        }
        
        for idx, motor_name in enumerate(self.topic_sequence):
            if motor_name in name_to_channel:
                mapping[idx] = name_to_channel[motor_name]
            else:
                self.get_logger().warn(f'Unknown motor name in sequence: {motor_name}')
        
        return mapping
    
    def map_to_servo_angle(self, value):
        """
        Maps a float value from range [-1, 1] to an integer in range [0, 180].
        
        Args:
            value: Float value between -1 and 1
            
        Returns:
            Integer value between 0 and 180
        """
        # Clamp the input to ensure it's within [-1, 1]
        clamped = max(-1.0, min(1.0, value))
        
        # Map from [-1, 1] to [0, 180]
        # Formula: ((input - input_min) / (input_max - input_min)) * (output_max - output_min) + output_min
        mapped = ((clamped - (-1)) / (1 - (-1))) * (180 - 0) + 0
        
        return int(mapped)
    
    def motor_callback(self, msg):
        """Process incoming motor control commands"""
        if len(msg.data) != len(self.topic_sequence):
            self.get_logger().warn(
                f'Expected {len(self.topic_sequence)} values, got {len(msg.data)}'
            )
            return
        
        # Set angle for each motor based on sequence mapping
        # Input: values from -1.0 to 1.0, mapped to servo angles [0, 180]
        for seq_idx, value in enumerate(msg.data):
            if seq_idx in self.sequence_to_channel:
                channel = self.sequence_to_channel[seq_idx]
                
                # Map the value from [-1, 1] to servo angle [0, 180]
                angle = self.map_to_servo_angle(value)
                
                try:
                    self.kit.servo[channel].angle = angle
                except Exception as e:
                    self.get_logger().error(f'Failed to set motor {channel}: {e}')
    
    def destroy_node(self):
        """Return all servos to neutral position on shutdown"""
        self.get_logger().info('Returning all servos to neutral position...')
        
        try:
            for channel in self.motor_indices.values():
                self.kit.servo[channel].angle = 90
        except Exception as e:
            self.get_logger().error(f'Error resetting servos: {e}')
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DriveInterface()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()