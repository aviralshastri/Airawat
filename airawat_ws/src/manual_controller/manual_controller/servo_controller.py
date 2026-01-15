#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray
from rclpy.client import Client


class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        
        # Declare and get parameters
        self.declare_parameter('pwm_range', [1000, 2000])
        self.declare_parameter('channels_sequence', ["roll", "pitch", "throttle", "yaw", "var", "mode"])
        self.declare_parameter('rc_topic', 'rc_input')
        
        self.pwm_range = self.get_parameter('pwm_range').value
        self.channels_sequence = self.get_parameter('channels_sequence').value
        self.rc_topic = self.get_parameter('rc_topic').value
        
        # Find mode channel index
        try:
            self.mode_channel_index = self.channels_sequence.index("mode")
            self.get_logger().info(f'Mode channel index: {self.mode_channel_index}')
        except ValueError:
            self.get_logger().error('Mode channel not found in channels_sequence!')
            raise
        
        # Calculate PWM thresholds for 3 positions
        pwm_min, pwm_max = self.pwm_range
        pwm_span = pwm_max - pwm_min
        band_size = pwm_span / 3
        
        self.ideal_max = pwm_min + band_size  # 1000 + 333.33 = 1333.33
        self.dig_max = pwm_min + 2 * band_size  # 1000 + 666.67 = 1666.67
        
        self.get_logger().info(f'PWM thresholds - IDEAL: {pwm_min}-{self.ideal_max:.1f}, '
                              f'DIG: {self.ideal_max:.1f}-{self.dig_max:.1f}, '
                              f'DUMP: {self.dig_max:.1f}-{pwm_max}')
        
        # State tracking
        self.current_state = None
        self.first_message_received = False
        
        # Create service client
        self.service_client = self.create_client(
            self.create_set_arm_state_service_type(),
            '/set_arm_state'
        )
        
        # Wait for service to be available
        self.get_logger().info('Waiting for /set_arm_state service...')
        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.get_logger().info('Service /set_arm_state is available')
        
        # Subscribe to RC input
        self.subscription = self.create_subscription(
            UInt16MultiArray,
            self.rc_topic,
            self.rc_input_callback,
            10
        )
        
        self.get_logger().info(f'Subscribed to {self.rc_topic}')
    
    def create_set_arm_state_service_type(self):
        """Dynamically create the service type"""
        try:
            from example_interfaces.srv import Trigger
            # Try to import custom service if available
            from arm_controller.srv import SetArmState
            return SetArmState
        except ImportError:
            # Fallback: Create a custom service type class
            self.get_logger().warn('Custom service type not found. Using mock type.')
            # For now, we'll assume the service exists
            # You need to replace this with your actual service import
            from example_interfaces.srv import Trigger
            return Trigger
    
    def pwm_to_state(self, pwm_value):
        """Convert PWM value to state string"""
        if pwm_value <= self.ideal_max:
            return "IDEAL"
        elif pwm_value <= self.dig_max:
            return "DIG"
        else:
            return "DUMP"
    
    def rc_input_callback(self, msg):
        """Callback for RC input messages"""
        # Check if mode channel exists in the message
        if len(msg.data) <= self.mode_channel_index:
            self.get_logger().warn(f'Mode channel index {self.mode_channel_index} '
                                  f'out of range (received {len(msg.data)} channels)')
            return
        
        # Get mode channel PWM value
        mode_pwm = msg.data[self.mode_channel_index]
        
        # Convert to state
        new_state = self.pwm_to_state(mode_pwm)
        
        # Check if this is the first message or if state has changed
        if not self.first_message_received:
            self.first_message_received = True
            self.current_state = new_state
            self.get_logger().info(f'Initial state: {new_state} (PWM: {mode_pwm})')
            self.call_set_arm_state_service(new_state)
        elif new_state != self.current_state:
            self.get_logger().info(f'State changed: {self.current_state} -> {new_state} (PWM: {mode_pwm})')
            self.current_state = new_state
            self.call_set_arm_state_service(new_state)
        else:
            self.get_logger().debug(f'State unchanged: {self.current_state} (PWM: {mode_pwm})')
    
    def call_set_arm_state_service(self, state):
        """Call the /set_arm_state service"""
        # Create request
        request = self.service_client.srv_type.Request()
        request.state = state
        request.smoothing_type = "CUBIC"
        
        self.get_logger().info(f'Calling service with state: {state}, smoothing: CUBIC')
        
        # Call service asynchronously
        future = self.service_client.call_async(request)
        future.add_done_callback(self.service_response_callback)
    
    def service_response_callback(self, future):
        """Callback for service response"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Service call successful: {response.message}')
            else:
                self.get_logger().warn(f'Service call failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed with exception: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ServoController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()