#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray
from arm_controller.srv import SetArmState  

class ModeController(Node):
    def __init__(self):
        super().__init__('mode_controller')
        
        # Declare parameters
        self.declare_parameter('rc_topic', 'rc_input')
        self.declare_parameter('channels_sequence', ['pitch', 'throttle', 'yaw', 'roll', 'var', 'mode'])
        self.declare_parameter('pwm_range', [1000, 2000])
        self.declare_parameter('mode_service_name', 'set_mode')
        self.declare_parameter('smoothing_type', 'CUBIC')  # Default smoothing type
        self.declare_parameter('deadband_margin', 50)  # PWM units for deadband around boundaries
        
        # Get parameters
        rc_topic = self.get_parameter('rc_topic').value
        self.channels_sequence = self.get_parameter('channels_sequence').value
        self.pwm_range = self.get_parameter('pwm_range').value
        mode_service_name = self.get_parameter('mode_service_name').value
        self.smoothing_type = self.get_parameter('smoothing_type').value
        self.deadband_margin = self.get_parameter('deadband_margin').value
        
        # Find index for mode channel
        try:
            self.mode_idx = self.channels_sequence.index('mode')
        except ValueError as e:
            self.get_logger().error(f'mode not found in channels_sequence: {e}')
            raise
        
        # Define mode states
        self.modes = ['IDEAL', 'DIG', 'DUMP']
        self.current_mode = None
        
        # Calculate PWM thresholds for 3 positions with deadband
        self.calculate_mode_thresholds()
        
        # Service client for mode switching
        self.mode_client = self.create_client(ModeSwitch, mode_service_name)
        
        # Wait for service to be available
        self.get_logger().info(f'Waiting for service: {mode_service_name}')
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service {mode_service_name} not available, waiting...')
        
        # Subscriber for RC input
        self.rc_sub = self.create_subscription(
            UInt16MultiArray,
            rc_topic,
            self.rc_callback,
            10
        )
        
        self.get_logger().info('Mode controller initialized')
        self.get_logger().info(f'Subscribing to RC: {rc_topic}')
        self.get_logger().info(f'Mode channel index: {self.mode_idx}')
        self.get_logger().info(f'PWM range: {self.pwm_range}')
        self.get_logger().info(f'Mode thresholds: {self.mode_thresholds}')
        self.get_logger().info(f'Smoothing type: {self.smoothing_type}')
    
    def calculate_mode_thresholds(self):
        """
        Calculate PWM thresholds for 3 modes with deadband margins
        Divides the PWM range into 3 equal zones with deadband around boundaries
        """
        pwm_min = self.pwm_range[0]
        pwm_max = self.pwm_range[1]
        pwm_span = pwm_max - pwm_min
        
        # Divide into 3 equal zones
        zone_size = pwm_span / 3.0
        
        # Calculate boundary points
        boundary1 = pwm_min + zone_size
        boundary2 = pwm_min + 2 * zone_size
        
        # Create thresholds with deadband
        # Format: [(lower_bound, upper_bound), ...]
        self.mode_thresholds = [
            (pwm_min, boundary1 - self.deadband_margin),  # IDEAL zone
            (boundary1 + self.deadband_margin, boundary2 - self.deadband_margin),  # DIG zone
            (boundary2 + self.deadband_margin, pwm_max)  # DUMP zone
        ]
        
        self.get_logger().info(f'Mode zones calculated:')
        for i, (mode, threshold) in enumerate(zip(self.modes, self.mode_thresholds)):
            self.get_logger().info(f'  {mode}: {threshold[0]:.0f} - {threshold[1]:.0f}')
    
    def get_mode_from_pwm(self, pwm_value):
        """
        Determine which mode the PWM value corresponds to
        Returns mode string or None if in deadband
        """
        for i, (lower, upper) in enumerate(self.mode_thresholds):
            if lower <= pwm_value <= upper:
                return self.modes[i]
        
        # If not in any zone, we're in a deadband
        return None
    
    def call_mode_service(self, mode):
        """
        Call the mode switching service asynchronously
        """
        request = ModeSwitch.Request()
        request.state = mode
        request.smoothing_type = self.smoothing_type
        
        self.get_logger().info(f'Calling mode service: {mode} with smoothing: {self.smoothing_type}')
        
        future = self.mode_client.call_async(request)
        future.add_done_callback(self.mode_service_callback)
    
    def mode_service_callback(self, future):
        """
        Handle service response
        """
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Mode switch successful: {response.message}')
            else:
                self.get_logger().warn(f'Mode switch failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
    
    def rc_callback(self, msg):
        """Process RC input and switch modes when needed"""
        # Check if we have the mode channel
        if len(msg.data) <= self.mode_idx:
            self.get_logger().warn(
                f'Not enough channels in RC data. Expected at least {self.mode_idx + 1}, got {len(msg.data)}'
            )
            return
        
        # Extract mode channel PWM value
        mode_pwm = msg.data[self.mode_idx]
        
        # Determine which mode this PWM corresponds to
        detected_mode = self.get_mode_from_pwm(mode_pwm)
        
        # If in deadband, don't change mode
        if detected_mode is None:
            # Optionally log deadband detection (comment out if too verbose)
            # self.get_logger().debug(f'Mode PWM {mode_pwm} in deadband, keeping current mode')
            return
        
        # If mode changed, call service
        if detected_mode != self.current_mode:
            self.get_logger().info(
                f'Mode switch detected: {self.current_mode} -> {detected_mode} (PWM: {mode_pwm})'
            )
            self.current_mode = detected_mode
            self.call_mode_service(detected_mode)


def main(args=None):
    rclpy.init(args=args)
    node = ModeController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()