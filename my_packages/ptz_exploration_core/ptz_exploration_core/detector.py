#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
    BoundingBox2D,
    Pose2D,
)
from ultralytics import YOLO
from ultralytics.utils.downloads import attempt_download_asset
from ament_index_python.packages import get_package_share_directory
from ptz_exploration_core.utils.ros import evaluate_pattern, imgmsg_to_bgr
from cv_bridge import CvBridge


class Detector(Node):
    def __init__(self):
        super().__init__("yolo_node")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("pkg_name", "simulation"),
                ("robot_name", "spot"),
                ("yolo_model", "yolo26n.pt"),
                ("yolo_device", "cpu"),
                ("confidence_threshold", 0.2),
                ("target_classes", ["person"]),
                ("detection_frequency", 1.0),
                ("cameras", [""]),
                ("image_pattern", ""),
                ("output_pattern", ""),
                ("publish_annotated_image", False),
            ],
        )

        self.pkg_name = self.get_parameter("pkg_name").value
        self.robot_name = self.get_parameter("robot_name").value
        self.yolo_model_name = self.get_parameter("yolo_model").value
        self.yolo_device = str(self.get_parameter("yolo_device").value).strip()
        self.confidence_threshold = self.get_parameter("confidence_threshold").value
        self.target_classes = self.get_parameter("target_classes").value
        self.detection_frequency = float(self.get_parameter("detection_frequency").value)
        self.cameras = self.get_parameter("cameras").value
        self.image_pattern = self.get_parameter("image_pattern").value
        self.output_pattern = self.get_parameter("output_pattern").value
        self.publish_annotated_image = bool(self.get_parameter("publish_annotated_image").value)

        # Convert detection frequency (Hz) to nanoseconds
        self.min_detection_interval_ns = (
            None if self.detection_frequency == 0 else int(1e9 / self.detection_frequency)
        )

        # Initialize YOLO
        model_dir = os.path.join(get_package_share_directory(self.pkg_name), "models")
        yolo_model_path = os.path.join(model_dir, self.yolo_model_name)
        if not os.path.exists(yolo_model_path):
            os.makedirs(model_dir, exist_ok=True)
            attempt_download_asset(yolo_model_path)
        self.model = YOLO(yolo_model_path)
        self.get_logger().info(f"YOLO inference device: {self.yolo_device or 'auto'}")

        # Pre-compute class IDs
        self.target_class_ids = []
        if self.target_classes:
            name_to_id = {v: k for k, v in self.model.names.items()}
            for name in self.target_classes:
                if name in name_to_id:
                    self.target_class_ids.append(name_to_id[name])
                else:
                    self.get_logger().warn(f"Class '{name}' not found in model.")

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Subscribers and Publishers dictionaries
        self.image_subs = {}
        self.results_pubs = {}
        self.last_processed_ns = {}

        # Detection Publisher
        self.detection_pub = self.create_publisher(Detection2DArray, "/detections", 10)

        # Initialize for each camera
        for camera in self.cameras:
            # Image Subscriber
            self.image_subs[camera] = self.create_subscription(
                Image,
                evaluate_pattern(self.image_pattern, robot=self.robot_name, camera=camera),
                lambda msg, cam=camera: self.image_callback(msg, cam),
                10,
            )

            # Debug Publisher
            if self.publish_annotated_image:
                self.results_pubs[camera] = self.create_publisher(
                    Image,
                    evaluate_pattern(self.output_pattern, robot=self.robot_name, camera=camera),
                    10,
                )

            self.get_logger().info(f"Initialized subscribers/publishers for {camera}")

    def image_callback(self, img_msg, camera):
        current_ns = img_msg.header.stamp.sec * 1_000_000_000 + img_msg.header.stamp.nanosec
        if self.min_detection_interval_ns is not None:
            last_ns = self.last_processed_ns.get(camera)
            if last_ns is not None and (current_ns - last_ns) < self.min_detection_interval_ns:
                return
            self.last_processed_ns[camera] = current_ns

        # Convert image message to numpy
        try:
            cv_image = imgmsg_to_bgr(self.bridge, img_msg)
        except Exception as e:
            self.get_logger().error(f"Image conversion error for {camera}: {e}")
            return

        # Run YOLO inference
        results = self.model.predict(
            cv_image,
            verbose=False,
            classes=self.target_class_ids if self.target_class_ids else None,
            conf=self.confidence_threshold,
            device=self.yolo_device if self.yolo_device else None,
        )
        detection_array = self._build_detection_array(results, img_msg.header)

        # Publish detections
        if detection_array.detections:
            self.detection_pub.publish(detection_array)

        # Publish annotated image
        if self.publish_annotated_image:
            self._publish_annotated_image(camera, results, img_msg.header)

    def _build_detection_array(self, results, header):
        detection_array = Detection2DArray()
        detection_array.header = header

        for result in results:
            for box in result.boxes:
                detection = Detection2D()

                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                center_pose = Pose2D()
                center_pose.position.x = float((x1 + x2) / 2.0)
                center_pose.position.y = float((y1 + y2) / 2.0)
                center_pose.theta = 0.0

                detection.bbox = BoundingBox2D()
                detection.bbox.center = center_pose
                detection.bbox.size_x = float(x2 - x1)
                detection.bbox.size_y = float(y2 - y1)

                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(int(box.cls[0]))
                hypothesis.hypothesis.score = float(box.conf[0])
                detection.results.append(hypothesis)

                detection_array.detections.append(detection)

        return detection_array

    def _publish_annotated_image(self, camera, results, header):
        try:
            annotated = results[0].plot()
            out_msg = Image()
            out_msg.header = header
            out_msg.height, out_msg.width = annotated.shape[0], annotated.shape[1]
            out_msg.encoding = "bgr8"
            out_msg.data = annotated.tobytes()
            out_msg.step = len(out_msg.data) // out_msg.height
            self.results_pubs[camera].publish(out_msg)
        except Exception as e:
            self.get_logger().error(f"Annotated image publish error for {camera}: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = Detector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
