import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraViewerNode(Node):
    def __init__(self):
        super().__init__('camera_viewer_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.frame_count = 0
        self.get_logger().info('camera_viewer_node started, waiting for frames on /camera/image_raw')

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.frame_count += 1

        if self.frame_count % 30 == 1:
            h, w, c = cv_image.shape
            self.get_logger().info(
                f'Frame #{self.frame_count}: {w}x{h}, {c} channels, '
                f'stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}'
            )

        if self.frame_count == 5:
            cv2.imwrite('/tmp/percept_nav_sample_frame.png', cv_image)
            self.get_logger().info('Saved sample frame to /tmp/percept_nav_sample_frame.png')


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
