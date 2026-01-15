#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from adafruit_servokit import ServoKit
import sys
import time


class DriveInterface(Node):
    def __init__(self):
        super().__init__('drive_interface')
        self.declare_parameter('channel_index.front_left', 0)
        self.declare_parameter('channel_index.front_right', 1)
        self.declare_parameter('channel_index.back_left', 2)
        self.declare_parameter('channel_index.back_right', 3)
        self.declare_parameter('channel_index.wrist', 4)
        self.declare_parameter('channel_index.shoulder', 5)
        self.declare_parameter('dead_channels', [6,7])
        
        # Declare nested parameters for servo init angles
        self.declare_parameter('servo_init_angle.wrist', 90)
        self.declare_parameter('servo_init_angle.shoulder', 180)
        
        # Declare PWM parameters
        self.declare_parameter('servo_min_pwm', 500)
        self.declare_parameter('servo_max_pwm', 2500)
        self.declare_parameter('drive_min_pwm', 1000)
        self.declare_parameter('drive_max_pwm', 2000)
        
        # Declare topic parameters
        self.declare_parameter('drive_topic_sequence', ['back_left', 'back_right', 'front_left', 'right_front'])
        self.declare_parameter('servo_topic_sequence', ['wrist', 'shoulder'])
        self.declare_parameter('drive_topic_name', 'motor_control')
        self.declare_parameter('servo_topic_name', 'arm_angles')
        
        # Declare actuation and init service parameters
        self.declare_parameter('max_actuation_angle', 180)
        self.declare_parameter('init_duration', 10.0)
        self.declare_parameter('init_smoothing_type', 'CUBIC')

        self.declare_parameter('angle_error', 10)
        
        # Get channel indices
        self.channel_index = {
            'front_left': self.get_parameter('channel_index.front_left').value,
            'front_right': self.get_parameter('channel_index.front_right').value,
            'back_left': self.get_parameter('channel_index.back_left').value,
            'back_right': self.get_parameter('channel_index.back_right').value,
            'wrist': self.get_parameter('channel_index.wrist').value,
            'shoulder': self.get_parameter('channel_index.shoulder').value,
        }

        dead_channels=self.get_parameter('dead_channels').value

        
        # Get servo init angles
        self.servo_init_angle = {
            'wrist': self.get_parameter('servo_init_angle.wrist').value,
            'shoulder': self.get_parameter('servo_init_angle.shoulder').value,
        }
        
        # Get PWM ranges
        self.servo_min_pwm = self.get_parameter('servo_min_pwm').value
        self.servo_max_pwm = self.get_parameter('servo_max_pwm').value
        self.drive_min_pwm = self.get_parameter('drive_min_pwm').value
        self.drive_max_pwm = self.get_parameter('drive_max_pwm').value
        
        # Get topic parameters
        self.drive_topic_sequence = self.get_parameter('drive_topic_sequence').value
        self.servo_topic_sequence = self.get_parameter('servo_topic_sequence').value
        self.drive_topic_name = self.get_parameter('drive_topic_name').value
        self.servo_topic_name = self.get_parameter('servo_topic_name').value
        
        # Get actuation parameters
        self.max_actuation_angle = self.get_parameter('max_actuation_angle').value
        self.init_duration = self.get_parameter('init_duration').value
        self.init_smoothing_type = self.get_parameter('init_smoothing_type').value

        self.angle_error=self.get_parameter('angle_error').value

        # Separate drive motors and servo arms
        self.drive_motors = ['front_left', 'front_right', 'back_left', 'back_right']
        self.servo_arms = ['wrist', 'shoulder']
        
        # Initialize ServoKit (PCA9685 - 16 channels)
        try:
            self.kit = ServoKit(channels=16)
            
            # Configure drive motors with drive PWM range
            for motor_name in self.drive_motors:
                channel = self.channel_index[motor_name]
                self.kit.servo[channel].set_pulse_width_range(self.drive_min_pwm, self.drive_max_pwm)
                # Initialize to neutral position (90 degrees)
                self.kit.servo[channel].angle = 100
            
            for dead_channel in dead_channels:
                self.kit.servo[dead_channel].set_pulse_width_range(self.drive_min_pwm, self.drive_max_pwm)
                # Initialize dead channels to neutral position (90 degrees)
                self.kit.servo[dead_channel].angle = 90
            
            
            # Configure servo arms with servo PWM range and actuation range
            for servo_name in self.servo_arms:
                channel = self.channel_index[servo_name]
                self.kit.servo[channel].set_pulse_width_range(self.servo_min_pwm, self.servo_max_pwm)
                self.kit.servo[channel].actuation_range = self.max_actuation_angle
            
            self.kit.servo[self.channel_index['wrist']].angle=self.servo_init_angle['wrist']
            self.kit.servo[self.channel_index['shoulder']].angle=self.servo_init_angle['shoulder']
            
            
            self.get_logger().info('ServoKit initialized successfully')
            self.get_logger().info(f'Drive motors mapping: {[(m, self.channel_index[m]) for m in self.drive_motors]}')
            self.get_logger().info(f'Servo arms mapping: {[(s, self.channel_index[s]) for s in self.servo_arms]}')
            self.get_logger().info(f'Drive PWM range: {self.drive_min_pwm} to {self.drive_max_pwm}')
            self.get_logger().info(f'Servo PWM range: {self.servo_min_pwm} to {self.servo_max_pwm}')
            self.get_logger().info(f'Servo actuation range: 0 to {self.max_actuation_angle}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize ServoKit: {e}')
            sys.exit(1)
        
        # Create mapping from drive topic sequence to motor channels
        self.drive_sequence_to_channel = self._create_drive_sequence_mapping()
        
        # Subscribe to drive motor control topic
        self.drive_subscription = self.create_subscription(
            Float32MultiArray,
            self.drive_topic_name,
            self.drive_motor_callback,
            10
        )
        
        # Subscribe to servo arm angles topic (JointState)
        self.servo_subscription = self.create_subscription(
            JointState,
            self.servo_topic_name,
            self.servo_callback,
            10
        )
        
        # Publisher for servo angles (for debugging/monitoring)
        self.angle_pub = self.create_publisher(
            Float32MultiArray,
            'servo_angles_debug',
            10
        )
        
        self.get_logger().info(f'Subscribed to drive topic: {self.drive_topic_name}')
        self.get_logger().info(f'Subscribed to servo topic: {self.servo_topic_name}')
        self.get_logger().info(f'Drive channel mapping: {self.drive_sequence_to_channel}')
        
        # Wait a bit for subscriptions to be established
        time.sleep(1.0)
        
    
    def _create_drive_sequence_mapping(self):
        """Create mapping from drive topic sequence index to motor channel"""
        mapping = {}
        
        for idx, motor_name in enumerate(self.drive_topic_sequence):
            if motor_name in self.channel_index and motor_name in self.drive_motors:
                mapping[idx] = self.channel_index[motor_name]
            else:
                self.get_logger().warn(f'Unknown motor name in drive sequence: {motor_name}')
        
        return mapping
        
    def map_to_servo_angle(self, value):
        """
        Maps a float value from range [-1, 1] to an integer in range [0, 180].
        Used for drive motors only.
        
        Args:
            value: Float value between -1 and 1
            
        Returns:
            Integer value between 0 and 180
        """
        # Clamp the input to ensure it's within [-1, 1]
        clamped = max(-1.0, min(1.0, value))
        
        # Map from [-1, 1] to [0, 180]
        mapped = ((clamped - (-1)) / (1 - (-1))) * (180 - 0) + 0
        
        return int(mapped)
    
    def drive_motor_callback(self, msg):
        """Process incoming drive motor control commands"""
        if len(msg.data) != len(self.drive_topic_sequence):
            self.get_logger().warn(
                f'Expected {len(self.drive_topic_sequence)} values, got {len(msg.data)}'
            )
            return
        
        # Store mapped angles for publishing
        mapped_angles = []
        
        # Set angle for each drive motor based on sequence mapping
        # Input: values from -1.0 to 1.0, mapped to servo angles [0, 180]
        for seq_idx, value in enumerate(msg.data):
            if seq_idx in self.drive_sequence_to_channel:
                channel = self.drive_sequence_to_channel[seq_idx]
                
                # Map the value from [-1, 1] to servo angle [0, 180]
                angle = self.map_to_servo_angle(value)
                mapped_angles.append(float(angle))
                
                try:
                    self.kit.servo[channel].angle = angle
                except Exception as e:
                    self.get_logger().error(f'Failed to set drive motor {channel}: {e}')
            else:
                mapped_angles.append(0.0)
        
        # Publish debug info
        angle_msg = Float32MultiArray()
        angle_msg.data = mapped_angles
        self.angle_pub.publish(angle_msg)
    
    def servo_callback(self, msg):
        """Process incoming servo arm angle commands from JointState message"""
        # Parse JointState message
        for i, joint_name in enumerate(msg.name):
            # Check if this joint is in our servo arms list
            if joint_name in self.servo_arms:
                if i < len(msg.position):
                    angle = msg.position[i]
                    
                    # Clamp angle to max actuation range
                    clamped_angle = max(0.0, min(float(self.max_actuation_angle), angle))
                    
                    # Get channel for this servo
                    channel = self.channel_index[joint_name]
                    
                    try:
                        self.kit.servo[channel].angle = clamped_angle
                        self.get_logger().debug(f'Set {joint_name} (channel {channel}) to {clamped_angle}°')
                    except Exception as e:
                        self.get_logger().error(f'Failed to set servo {joint_name} (channel {channel}): {e}')
    
    def destroy_node(self):
        """Return all motors to neutral position on shutdown"""
        self.get_logger().info('Returning all motors to neutral position...')
        
        try:
            for motor_name in self.drive_motors:
                channel = self.channel_index[motor_name]
                self.kit.servo[channel].angle = 100
            
        except Exception as e:
            self.get_logger().error(f'Error resetting motors: {e}')
        
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