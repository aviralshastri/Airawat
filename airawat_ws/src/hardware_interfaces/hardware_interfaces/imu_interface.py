#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

import board
import busio
import adafruit_bno055
import time

# BNO055 register addresses
ACC_CONFIG = 0x08
GYRO_CONFIG_0 = 0x0A

# Bitfield helper: sets ACC & GYRO ODR to 200 Hz
ACC_200HZ = (0b000 << 5) | (0b011 << 2) | 0b10
GYRO_200HZ = (0b00111 << 3) | 0b000


class IMUInterface(Node):
    def __init__(self):
        super().__init__('imu_interface')

        # Sensor data QoS profile for better performance
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.publisher = self.create_publisher(Imu, 'imu', qos)

        
        i2c = busio.I2C(board.SCL, board.SDA)
        
        self.bno = adafruit_bno055.BNO055_I2C(i2c)
        self.setup_bno055()

        # Pre-allocate message to avoid repeated allocations
        self.msg = Imu()
        self.msg.header.frame_id = "imu_link"
        
        # Pre-set constant values
        self.msg.orientation_covariance[0] = -1.0
        self.msg.angular_velocity_covariance = [
            0.005, 0.0, 0.0,
            0.0, 0.005, 0.0,
            0.0, 0.0, 0.005
        ]
        self.msg.linear_acceleration_covariance = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1
        ]

        # Cache the clock for faster access
        self.clock = self.get_clock()

        # Timer for 200 Hz publishing (0.005 sec = 5ms)
        # Using CLOCK_MONOTONIC ensures precise timing
        self.timer = self.create_timer(0.005, self.publish_imu)
        
        self.get_logger().info("IMU publisher initialized and running at 200 Hz")

    def setup_bno055(self):
        """Put BNO055 into ACCGYRO mode at 200 Hz."""
        self.get_logger().info("Configuring BNO055 for raw ACC+GYRO 200 Hz")

        # Switch to CONFIG mode
        self.bno.mode = adafruit_bno055.CONFIG_MODE
        time.sleep(0.05)

        # Write raw sensor ODR settings
        self.bno._write_register(ACC_CONFIG, ACC_200HZ)
        self.bno._write_register(GYRO_CONFIG_0, GYRO_200HZ)

        # Switch to ACCGYRO_MODE
        self.bno.mode = adafruit_bno055.ACCGYRO_MODE
        time.sleep(0.05)

        self.get_logger().info("✔ BNO055 set to ACCGYRO_MODE @ 200 Hz")

    def publish_imu(self):
        """Optimized IMU data publishing with minimal latency."""
        try:
            # Timestamp IMMEDIATELY before reading sensor to minimize latency
            # This captures the time closest to actual measurement
            stamp = self.clock.now().to_msg()
            
            # Read sensor data (this is the slowest operation ~2-3ms over I2C)
            accel = self.bno.acceleration
            gyro = self.bno.gyro

            # Validate data
            if accel is None or gyro is None:
                self.failed_reads += 1
                if self.failed_reads % 100 == 0:
                    self.get_logger().warn(
                        f"Failed to read IMU data ({self.failed_reads} failures)"
                    )
                return

            # Update timestamp in pre-allocated message
            self.msg.header.stamp = stamp

            # Unpack directly to message fields (faster than indexing)
            self.msg.linear_acceleration.x = accel[0]
            self.msg.linear_acceleration.y = accel[1]
            self.msg.linear_acceleration.z = accel[2]

            self.msg.angular_velocity.x = gyro[0]
            self.msg.angular_velocity.y = gyro[1]
            self.msg.angular_velocity.z = gyro[2]

            # Publish
            self.publisher.publish(self.msg)

        except Exception as e:
            self.get_logger().error(f"Error in publish_imu: {e}")
            self.failed_reads += 1


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = IMUInterface()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()