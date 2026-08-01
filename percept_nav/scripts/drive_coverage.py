import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time


class CoverageDriver(Node):
    def __init__(self):
        super().__init__('coverage_driver')
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)

    def send_cmd(self, linear, angular, duration):
        msg = TwistStamped()
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        end_time = time.time() + duration
        while time.time() < end_time:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(msg)
            time.sleep(0.1)
        self.stop()

    def stop(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)
        time.sleep(0.3)


def main():
    rclpy.init()
    driver = CoverageDriver()

    moves = [
        (0.15, 0.0, 4.0),
        (0.0, 0.5, 3.0),
        (0.15, 0.0, 4.0),
        (0.0, 0.5, 3.0),
        (0.15, 0.0, 4.0),
        (0.0, 0.5, 3.0),
        (0.15, 0.0, 4.0),
        (0.0, 0.5, 3.0),
        (0.15, 0.0, 2.0),
    ]

    for linear, angular, duration in moves:
        driver.get_logger().info(f'Moving: linear={linear}, angular={angular}, for {duration}s')
        driver.send_cmd(linear, angular, duration)

    driver.get_logger().info('Coverage drive complete')
    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
