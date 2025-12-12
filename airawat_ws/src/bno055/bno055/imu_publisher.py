#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

import board
import busio
import adafruit_bno055
import time

ACC_CONFIG = 0x08
GYRO_CONFIG_0 = 0x0A

ACC_250HZ = 0x28
GYRO_230HZ = 0x08

class BNO055Publisher(Node):
    def __init__(self):
        super().__init__('bno055_imu_pub')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.publisher = self.create_publisher(Imu, 'imu', qos)

        try:
            i2c = busio.I2C(board.SCL, board.SDA)
        except:
            self.get_logger().warn("Failed to set 400kHz I2C, using default")
            i2c = busio.I2C(board.SCL, board.SDA)
        
        self.bno = adafruit_bno055.BNO055_I2C(i2c)
        self.setup_bno055()

        self.msg = Imu()
        self.msg.header.frame_id = "imu_link"
        
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

        self.clock = self.get_clock()
        self.timer = self.create_timer(0.005, self.publish_imu)
        
        self.get_logger().info("IMU publisher initialized and running at 200 Hz")

    def setup_bno055(self):
        """Put BNO055 into ACCGYRO mode with ACC 250Hz + GYRO 230Hz."""
        self.get_logger().info("Configuring BNO055: ACC=250Hz, GYRO=230Hz")

        self.bno.mode = adafruit_bno055.CONFIG_MODE
        time.sleep(0.05)

        self.bno._write_register(0x07, 0x01)

        self.bno._write_register(ACC_CONFIG, ACC_250HZ)
        self.bno._write_register(GYRO_CONFIG_0, GYRO_230HZ)

        self.bno._write_register(0x07, 0x00)

        self.bno.mode = adafruit_bno055.ACCGYRO_MODE
        time.sleep(0.05)

        self.get_logger().info("✔ BNO055 set: ACC=250Hz, GYRO=230Hz (ACCGYRO_MODE)")

    def publish_imu(self):
        try:
            stamp = self.clock.now().to_msg()
            
            accel = self.bno.acceleration
            gyro = self.bno.gyro

            if accel is None or gyro is None:
                return

            self.msg.header.stamp = stamp

            self.msg.linear_acceleration.x = accel[0]
            self.msg.linear_acceleration.y = accel[1]
            self.msg.linear_acceleration.z = accel[2]

            self.msg.angular_velocity.x = gyro[0]
            self.msg.angular_velocity.y = gyro[1]
            self.msg.angular_velocity.z = gyro[2]

            self.publisher.publish(self.msg)

        except Exception as e:
            self.get_logger().error(f"Error in publish_imu: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = BNO055Publisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
