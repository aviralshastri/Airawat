#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray
import serial
import threading


class RCInputPublisher(Node):
    def __init__(self):
        super().__init__('rc_input_publisher')
        
        # Declare parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 230400)
        self.declare_parameter('num_channels', 6)
        self.declare_parameter('topic_name', 'rc_input')
        
        # Get parameters
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.num_channels = self.get_parameter('num_channels').value
        topic_name = self.get_parameter('topic_name').value
        
        # Protocol bytes
        self.START_BYTE = 0xFF
        self.END_BYTE = 0xFE
        
        # Create publisher
        self.publisher = self.create_publisher(UInt16MultiArray, topic_name, 10)
        
        # Initialize serial connection
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1.0)
            self.get_logger().info(f'Connected to {port} at {baudrate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise
        
        # Flag to control the reading thread
        self.running = True
        
        # Start reading thread
        self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.read_thread.start()
        
    def read_serial_loop(self):
        """Continuously read from serial port in blocking mode"""
        while self.running and rclpy.ok():
            try:
                # Wait for start byte (blocking read)
                byte = self.ser.read(1)
                if len(byte) > 0 and byte[0] == self.START_BYTE:
                    # Read channel data + end byte
                    data_length = self.num_channels * 2 + 1
                    data = self.ser.read(data_length)
                    
                    # Verify end byte and correct length
                    if len(data) == data_length and data[-1] == self.END_BYTE:
                        # Parse channel values
                        channels = []
                        for i in range(0, self.num_channels * 2, 2):
                            value = data[i] | (data[i+1] << 8)
                            channels.append(value)
                        
                        # Publish data
                        msg = UInt16MultiArray()
                        msg.data = channels
                        self.publisher.publish(msg)
                        
                        self.get_logger().debug(f'Published: {channels}')
                    else:
                        self.get_logger().warn('Invalid packet: incorrect length or end byte')
            
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
                break
            except Exception as e:
                self.get_logger().error(f'Unexpected error: {e}')
                break
    
    def destroy_node(self):
        """Clean up serial connection on shutdown"""
        self.running = False
        if hasattr(self, 'read_thread'):
            self.read_thread.join(timeout=2.0)
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('Serial port closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RCInputPublisher()
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