import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

import math
import tf_transformations

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('fake_proxops_target_publisher')
        self.yawrate_pub = self.create_publisher(Float32, '/lolo_auv_v1/proxops/yawrate_request', 10)
        self.rpm_pub = self.create_publisher(Float32, '/lolo_auv_v1/proxops/rpm_request', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0.0

    def timer_callback(self):

        yawrate = Float32()
        rpm = Float32()
        yawrate.data = 0.05*math.cos(self.i)
        rpm.data = 500 + 300*math.sin(self.i)


        self.yawrate_pub.publish(yawrate)
        self.rpm_pub.publish(rpm)

        self.get_logger().info("Publishing")
        self.i += 0.05


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()