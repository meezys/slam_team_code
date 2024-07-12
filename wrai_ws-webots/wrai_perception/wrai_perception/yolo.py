import os
import time
import inspect
import math
import numpy as np

from typing import Union, List

import cv2
from ultralytics import YOLO

import pyzed.sl as sl

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from cv_bridge import CvBridge

# from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D
from sensor_msgs.msg import CompressedImage  # , Image
from wrai_msgs.msg import ConeMeasurementArray, ConeMeasurement

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
if currentdir.startswith("/wrai"):
    homedir = "/wrai"
else:
    homedir = os.path.join(currentdir.split("WRAI1-Main")[0], "WRAI1-Main")

FPS = 30

CLASS_MAPPING = [
    ConeMeasurement.CONE_TYPE_BLUE,
    ConeMeasurement.CONE_TYPE_BIG_ORANGE,
    ConeMeasurement.CONE_TYPE_ORANGE,
    ConeMeasurement.CONE_TYPE_UNKNOWN,
    ConeMeasurement.CONE_TYPE_YELLOW,
]


def draw_detections(
    img: np.ndarray,
    bboxes: List[List[int]],
    # classes: List[int],
    # class_labels: Union[List[str], None],
    distances: List[float],
    angles: List[float],
):
    for bbox, distance, angle in zip(bboxes, distances, angles):
        x1, y1, x2, y2 = bbox

        # color = get_random_color(int(cls))
        color = (0, 250, 50)
        img = cv2.rectangle(
            cv2.UMat(img), (int(x1), int(y1)), (int(x2), int(y2)), color, 3
        )

        # if class_labels:
        #     label = class_labels[int(cls)]

        #     x_text = int(x1)
        #     y_text = max(15, int(y1 - 10))
        #     img = cv2.putText(
        #         img,
        #         label,
        #         (x_text, y_text),
        #         cv2.FONT_HERSHEY_SIMPLEX,
        #         0.5,
        #         color,
        #         1,
        #         cv2.LINE_AA,
        #   )
        angle_deg = np.degrees(angle)
        label = f"{distance:.2f} | {angle_deg:.0f}"

        x_text = int(x1)
        y_text = max(15, int(y1 - 10))
        img = cv2.putText(
            img,
            label,
            (x_text, y_text),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            1,
            cv2.LINE_AA,
        )

    if isinstance(img, np.ndarray):
        return img
    elif isinstance(img, cv2.UMat):
        return img.get()


class PerceptionPublisher(Node):
    """The ROS Node that detects cones in camera images, then publishes the locations."""

    def __init__(
        self,
        class_labels: Union[List, None] = None,
    ):
        """Construct the node.

        :param img_topic: name of the image topic to listen to
        :param weights: path/to/yolo_weights.pt
        :param conf_thresh: confidence threshold
        :param iou_thresh: intersection over union threshold
        :param pub_topic: name of the output topic (will be published under the
            namespace '/yolov7')
        :param device: device to do inference on (e.g., '0' (GPU) or 'cpu')
        :param queue_size: queue size for publishers
        :visualize: flag to enable publishing the detections visualized in the image
        :param img_size: (height, width) to which the img is resized before being
            fed into the yolo network. Final output coordinates will be rescaled to
            the original img size.
        :param class_labels: List of length num_classes, containing the class
            labels. The i-th element in this list corresponds to the i-th
            class id. Only for viszalization. If it is None, then no class
            labels are visualized.
        """
        super().__init__("yolo")

        weights_path = self.declare_parameter(
            "weights_path", "yolov8n-cones-300-i16.engine"
        ).value
        self.conf_thresh = self.declare_parameter("conf_thresh", 0.25).value
        self.iou_thresh = self.declare_parameter("iou_thresh", 0.7).value
        img_size = self.declare_parameter("img_size", (640, 640)).value
        visualize = self.declare_parameter("visualize", True).value
        device = self.declare_parameter("device", "0").value

        weights_path = os.path.join(homedir, "wrai_perception/", weights_path)
        print(f"Loading weights from {weights_path}")

        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        # init_params.sdk_verbose = True # Enable verbose logging
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = FPS
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        init_params.coordinate_units = sl.UNIT.METER
        init_params.depth_stabilization = 50
        # init_params.coordinate_system = sl.COORDINATE_SYSTEM.LEFT_HANDED_Z_UP

        # opening the camera
        err = self.zed.open(init_params)
        start_time = time.time()
        # 10 second timer
        while err != sl.ERROR_CODE.SUCCESS and (time.time() - start_time < 10):
            err = self.zed.open(init_params)
            time.sleep(0.5)

        if err != sl.ERROR_CODE.SUCCESS:
            print("cannot connect to zed")
            exit(-1)

        # preallocate big objects and reuse them
        resolution = self.zed.get_camera_information().camera_resolution
        self.image_mat = sl.Mat(resolution.width, resolution.height, sl.MAT_TYPE.U8_C4)
        self.point_cloud_mat = sl.Mat(
            resolution.width, resolution.height, sl.MAT_TYPE.F32_C4
        )
        self.depth_mat = sl.Mat(resolution.width, resolution.height, sl.MAT_TYPE.U8_C4)

        self.runtime_params = sl.RuntimeParameters()
        self.runtime_params.sensing_mode = sl.SENSING_MODE.STANDARD
        self.runtime_params.enable_depth = True
        self.runtime_params.remove_saturated_areas = True

        self.img_size = img_size
        self.device = device
        self.class_labels = class_labels

        # self.img_subscriber = rclpy.Subscriber(
        #     img_topic,
        #     Image,
        #     self.process_img_msg,
        # )
        # self.detection_publisher = rclpy.Publisher(
        #     pub_topic, Detection2DArray, queue_size=queue_size
        # )

        # visualize = False
        if visualize:
            self.detection_publisher = self.create_publisher(
                CompressedImage, "/perception/detections", 2
            )
            self.depth_publisher = self.create_publisher(
                CompressedImage, "/perception/depth", 2
            )
        else:
            self.detection_publisher = None
            self.depth_publisher = None

        self.cone_publisher = self.create_publisher(
            ConeMeasurementArray, "/camera_cones", 10
        )

        self.bridge = CvBridge()

        self.model = YOLO(weights_path, task="detect")

        print("Perception initialization complete. Ready to start inference")

    def run(self):
        """Process images until the node is ended."""
        while rclpy.ok():
            self.process_img()

    def process_img(self):
        """Take an image from the camera and detect cones, publishing the results."""

        # blocks until a new frame arrives, and reruns calculations
        error = self.zed.grab(self.runtime_params)
        if error != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"ZED grab threw error {error}")

        # get the new frame
        self.zed.retrieve_image(self.image_mat)

        np_img_orig = self.image_mat.get_data()[:, :, 0:3]  # as an np array
        # vis_msg = self.bridge.cv2_to_imgmsg(np_img_orig)
        # self.detection_publisher.publish(vis_msg)
        # return

        # handle possible different img formats
        # if len(np_img_orig.shape) == 2:
        #     np_img_orig = np.stack([np_img_orig] * 3, axis=2)

        # automatically resize the image to the next smaller possible size
        h_orig, w_orig, c = np_img_orig.shape
        # w_scaled, h_scaled = self.img_size
        # np_img_resized = cv2.resize(np_img_orig, (w_scaled, h_scaled))
        w_scaled, h_scaled = w_orig, h_orig
        np_img_resized = np_img_orig

        xscale = w_orig / w_scaled
        yscale = h_orig / h_scaled

        # predict
        detections = self.model.predict(
            np_img_resized,
            conf=self.conf_thresh,
            iou=self.iou_thresh,
            device=self.device,
            imgsz=self.img_size[::-1],
            verbose=False,
        )

        measurements = []
        distances = []
        angles = []
        self.zed.retrieve_measure(self.point_cloud_mat, sl.MEASURE.XYZ)
        for box in detections[0].boxes:
            x1, y1, x2, y2, conf, class_id = box.data[0].tolist()

            x1 *= xscale
            x2 *= xscale
            y1 *= yscale
            y2 *= yscale

            err, point = self.point_cloud_mat.get_value((x1 + x2) / 2, (y1 + y2) / 2)

            distance = math.hypot(point[0], point[2])
            angle = math.atan2(point[2], point[0])
            distances.append(distance)
            angles.append(angle)
            if (
                np.isnan(distance)
                or np.isinf(distance)
                or np.isnan(angle)
                or np.isinf(angle)
            ):
                continue

            measurement = ConeMeasurement()
            measurement.radius = distance
            measurement.angle = angle
            measurement.cone_type = CLASS_MAPPING[int(class_id)]
            print(class_id)

            measurements.append(measurement)

        zed_timestamp = self.image_mat.timestamp
        t = int(np.floor(zed_timestamp.get_nanoseconds()))
        t_sec = int(np.floor(zed_timestamp.get_seconds()))

        msg = ConeMeasurementArray()
        msg.header = Header()
        msg.header.frame_id = "camera_footprint"
        msg.header.stamp.sec = t_sec
        msg.header.stamp.nanosec = int(t - (1e9 * t_sec))

        msg.cones = measurements

        self.cone_publisher.publish(msg)

        # visualizing if required
        if self.detection_publisher:
            bboxes = [
                [int(x1 * xscale), int(y1 * yscale), int(x2) * xscale, int(y2 * yscale)]
                for x1, y1, x2, y2, _, _ in [
                    box.data[0].tolist() for box in detections[0].boxes
                ]
            ]
            # # classes = [int(c) for c in detections[:, 5].tolist()]
            vis_img = draw_detections(np_img_orig, bboxes, distances, angles)
            # vis_img = detections[0].plot(line_width=1, labels=False, probs=False)
            vis_msg = self.bridge.cv2_to_compressed_imgmsg(vis_img)
            self.detection_publisher.publish(vis_msg)

            self.zed.retrieve_image(self.depth_mat, sl.VIEW.DEPTH)
            depth_msg = self.bridge.cv2_to_compressed_imgmsg(
                self.depth_mat.get_data()[:, :, :3]
            )
            self.depth_publisher.publish(depth_msg)


def main():
    rclpy.init(args=None)

    node = PerceptionPublisher()

    # rclpy.spin(node)
    node.run()
    node.zed.close()


if __name__ == "__main__":
    main()
