import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from adafruit_servokit import ServoKit


class ServoInterface(Node):

    def __init__(self):
        super().__init__('servo_interface')

        # ---------------------------
        # Parameters
        # ---------------------------
        self.declare_parameter('wrist', 4)
        self.declare_parameter('shoulder', 5)
        self.declare_parameter('elbow', 6)

        self.declare_parameter('servo_min_pwm', 500)
        self.declare_parameter('servo_max_pwm', 2500)

        self.servo_map = {
            'wrist': self.get_parameter('wrist').value,
            'shoulder': self.get_parameter('shoulder').value,
            'elbow': self.get_parameter('elbow').value,
        }

        self.min_pwm = self.get_parameter('servo_min_pwm').value
        self.max_pwm = self.get_parameter('servo_max_pwm').value

        # ---------------------------
        # ServoKit Setup
        # ---------------------------
        self.kit = ServoKit(channels=16)

        for channel in self.servo_map.values():
            self.kit.servo[channel].set_pulse_width_range(
                self.min_pwm, self.max_pwm
            )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscriber = self.create_subscription(
            JointState,
            '/arm_angles',
            self.joint_state_callback,
            qos
        )

        self.get_logger().info('Servo Interface Node Started')

    def joint_state_callback(self, msg: JointState):
        for name, position in zip(msg.name, msg.position):

            if name not in self.servo_map:
                continue

            channel = self.servo_map[name]

            angle_deg = max(0.0, min(180.0, position))

            try:
                self.kit.servo[channel].angle = angle_deg
            except Exception as e:
                self.get_logger().error(
                    f'Failed to move servo {name} (ch {channel}): {e}'
                )


def main():
    rclpy.init()
    node = ServoInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
