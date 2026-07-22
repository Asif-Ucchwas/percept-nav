import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from vision_msgs.msg import Detection2DArray
import message_filters


class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Camera intrinsics -- see docs/notes/sensor_fusion_math.md for the
        # full derivation of these numbers.
        self.image_width = 320
        self.fov_rad = 1.02974
        self.crop_x = 64
        self.focal_length_px = (self.image_width / 2.0) / math.tan(self.fov_rad / 2.0)
        self.half_fov = self.fov_rad / 2.0

        scan_sub = message_filters.Subscriber(self, LaserScan, '/scan')
        detections_sub = message_filters.Subscriber(self, Detection2DArray, '/camera/detections_2d')

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [scan_sub, detections_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.fused_callback)

        self.publisher = self.create_publisher(
            Detection2DArray,
            '/detected_obstacles',
            10
        )

        self.frame_count = 0
        self.get_logger().info('sensor_fusion_node started, waiting for synced scan + detections')

    def fused_callback(self, scan_msg, detections_msg):
        self.frame_count += 1
        fused_count = 0

        for det in detections_msg.detections:
            # Concept 4 (reversed): detection bbox is in cropped-image
            # coordinates, so shift back to raw-image coordinates first.
            raw_pixel_x = det.bbox.center.position.x + self.crop_x
            box_left = raw_pixel_x - det.bbox.size_x / 2.0
            box_right = raw_pixel_x + det.bbox.size_x / 2.0

            best_range = None

            for i, r in enumerate(scan_msg.ranges):
                if not math.isfinite(r):
                    continue
                if r < scan_msg.range_min or r > scan_msg.range_max:
                    continue

                theta = scan_msg.angle_min + i * scan_msg.angle_increment
                # Normalize to [-pi, pi] so angles behind the robot (near 2*pi)
                # read as negative instead of a huge positive number.
                if theta > math.pi:
                    theta -= 2 * math.pi

                # Concept 3: skip anything outside the camera's field of view.
                if theta < -self.half_fov or theta > self.half_fov:
                    continue

                # Concepts 1 & 2: project this LiDAR angle to a pixel column.
                pixel_x = (self.image_width / 2.0) + self.focal_length_px * math.tan(theta)

                if box_left <= pixel_x <= box_right:
                    if best_range is None or r < best_range:
                        best_range = r

            if best_range is not None:
                fused_count += 1
                # Store the fused distance in the pose field vision_msgs
                # provides for exactly this purpose.
                for result in det.results:
                    result.pose.pose.position.x = best_range

        if self.frame_count % 30 == 1:
            self.get_logger().info(
                f'Sync #{self.frame_count}: {len(detections_msg.detections)} detections, '
                f'{fused_count} matched to a LiDAR range'
            )

        self.publisher.publish(detections_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
