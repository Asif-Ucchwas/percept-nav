import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector_node')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.publisher = self.create_publisher(
            Image,
            '/camera/obstacle_detections',
            10
        )

        self.detections_publisher = self.create_publisher(
            Detection2DArray,
            '/camera/detections_2d',
            10
        )

        self.crop_x = 64
        self.crop_y = 48
        self.crop_w = 192
        self.crop_h = 144

        self.min_contour_area = 150
        # Reject anything bigger than half the cropped frame -- that's almost
        # certainly a merged background blob, not a single real obstacle.
        self.max_contour_area = (self.crop_w * self.crop_h) // 2

        self.morph_kernel = np.ones((3, 3), np.uint8)

        self.frame_count = 0
        self.get_logger().info('obstacle_detector_node started (fixed threshold=128 segmentation)')

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.frame_count += 1

        cropped = cv_image[
            self.crop_y:self.crop_y + self.crop_h,
            self.crop_x:self.crop_x + self.crop_w
        ]

        gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Otsu automatically finds the brightness cutoff that best separates
        # two groups of pixels -- here, bright floor vs. darker obstacles.
        # THRESH_BINARY_INV flags the darker group (obstacles) as foreground.
        # Adaptive threshold: the floor gets darker toward the horizon due to
        # perspective/lighting falloff, so a single global cutoff (tried above,
        # failed) can't separate floor from obstacles everywhere in the frame.
        # This computes a local cutoff per-region instead, following the
        # brightness gradient. Block size 41 chosen to exceed typical object
        # width (~30-50px) so each region compares an object against its true
        # surroundings, not against itself.
        thresh = cv2.adaptiveThreshold(
            blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV, blockSize=41, C=10
        )

        # Clean up small speckle noise from the threshold step.
        opened = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, self.morph_kernel)

        contours, _ = cv2.findContours(
            opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        detections = 0
        detection_array = Detection2DArray()
        detection_array.header = msg.header

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_contour_area or area > self.max_contour_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(cropped, (x, y), (x + w, y + h), (0, 255, 0), 2)
            detections += 1

            det = Detection2D()
            det.bbox.center.position.x = float(x + w / 2.0)
            det.bbox.center.position.y = float(y + h / 2.0)
            det.bbox.size_x = float(w)
            det.bbox.size_y = float(h)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = 'obstacle'
            # Classical CV gives no learned confidence score; 1.0 marks
            # "detected", not a probability estimate (that's a deep-learning
            # concept -- being honest about what this pipeline can and can't
            # produce).
            hyp.hypothesis.score = 1.0
            det.results.append(hyp)

            detection_array.detections.append(det)

        if self.frame_count % 30 == 1:
            self.get_logger().info(
                f'Frame #{self.frame_count}: {detections} obstacle(s) detected'
            )

        annotated_msg = self.bridge.cv2_to_imgmsg(cropped, encoding='bgr8')
        annotated_msg.header = msg.header
        self.publisher.publish(annotated_msg)
        self.detections_publisher.publish(detection_array)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
