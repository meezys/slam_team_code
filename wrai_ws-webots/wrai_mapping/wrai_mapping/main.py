import rclpy
from rclpy.node import Node
from rclpy.time import Time
from message_filters import ApproximateTimeSynchronizer, Subscriber
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from typing import List

from wrai_msgs.msg import ConeMeasurementArray, ConeArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import (
    Quaternion,
    Point,
    PoseWithCovarianceStamped,
    TransformStamped,
)
from std_msgs.msg import ColorRGBA

import numpy as np

#from .fastslam import FastSLAM
from planarslam import GraphSLAM
from .cone_type import ConeType

# TODO: maybe look at instead of giving each particle (dx,dy), give it the proposed position and
# get the particles themselves to move themselves towards that proposed position.
# TODO: lap counting
# TODO: make faster

# TODO: correctly initialise landmarks with cov
# TODO: FastSLAM 2
# TODO: store landmarks in a kd tree
# TODO: multi threading (update each particle seperately)
# TODO: use the fact that a landmark cant be observed twice in one go
#   (enforce mutual exclusion, using a queue?)
# TODO: maybe try to use the fact that the vehicle moves in the direction it is facing to
#   improve position estimates
# TODO: Use cone type as part of data association to figure out probabilities.
# TODO: sigma values for landmarks are probably wrong


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = np.cos(ai)
    si = np.sin(ai)
    cj = np.cos(aj)
    sj = np.sin(aj)
    ck = np.cos(ak)
    sk = np.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q = np.empty((4,))
    q[0] = cj * sc - sj * cs
    q[1] = cj * ss + sj * cc
    q[2] = cj * cs - sj * sc
    q[3] = cj * cc + sj * ss

    return q


class SLAMNode(Node):
    """Takes cone measurements and maps them."""

    def __init__(self) -> None:
        super().__init__("slam")

        # ROS Parameters

        # Where the car starts
        start_state = self.declare_parameter("start_state", [0.0, 0.0, 0.0]).value
        num_particles = self.declare_parameter("num_particles", 5).value
        sync_slop = self.declare_parameter("sync_slop", 0.05).value
        self.type_obs_thresh = self.declare_parameter("type_obs_thresh", 0.75).value
        association_threshold = self.declare_parameter(
            "association_threshold", 0.75
        ).value
        landmark_combination_threshold = self.declare_parameter(
            "landmark_combination_threshold", 0.5
        ).value
        obs_distance_thresh = self.declare_parameter("obs_distance_thresh", 15).value
        obs_angle_thresh = self.declare_parameter("obs_angle_thresh", np.pi / 2).value
        type_obs_prob = self.declare_parameter("type_obs_prob", 0.75).value

        motion_dist_noise = self.declare_parameter("motion_dist_noise", 0.05).value
        motion_angle_noise = self.declare_parameter("motion_angle_noise", 0.005).value
        obs_dist_noise = self.declare_parameter("obs_dist_noise", 0.05).value
        obs_angle_noise = self.declare_parameter("obs_angle_noise", 0.0025).value

        # setup

        self.publisher = self.create_publisher(ConeArray, "/cones", 10)
        self.visualization_pub = self.create_publisher(Marker, "/slam/viz", 1)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cone_subscriber = Subscriber(self, ConeMeasurementArray, "/camera_cones")
        self.pos_subscriber = Subscriber(
            self, PoseWithCovarianceStamped, "/vectornav/pose"
        )

        self.subscriber = ApproximateTimeSynchronizer(
            [self.cone_subscriber, self.pos_subscriber], 50, slop=sync_slop
        )
        self.subscriber.registerCallback(self._cone_measurement_callback)

        start_state = np.array(start_state)

        self.slam = FastSLAM(
            start_state,
            num_particles,
            association_threshold,
            landmark_combination_threshold,
            obs_distance_thresh,
            obs_angle_thresh,
            type_obs_prob,
            motion_dist_noise,
            motion_angle_noise,
            obs_dist_noise,
            obs_angle_noise,
        )
        self.last_pose = start_state
        print("SLAM Ready")

    def _cone_measurement_callback(
        self, cone_msg: ConeMeasurementArray, pose_msg: PoseWithCovarianceStamped
    ) -> None:
        # print("doing some slam")

        # --- Perform SLAM update ---
        full_pose = pose_msg.pose.pose

        # Lookup the transform between the camera frame and the chassis frame
        try:
            from_frame = "camera_footprint"
            to_frame = "base_footprint"
            # time of 0 gets the latest transform it can
            t = self.tf_buffer.lookup_transform(
                to_frame, from_frame, Time(seconds=0, nanoseconds=0)
            )
        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform {to_frame} to {from_frame}: {ex}"
            )
            return

        position = np.array(
            [
                full_pose.position.x,
                full_pose.position.y,
            ]
        )
        yaw = self.yaw_from_quaternion(full_pose.orientation)

        # x, y, theta
        pose = np.array([position[0], position[1], yaw])

        motion = pose - self.last_pose
        self.last_pose = pose

        observations = []
        for cone in cone_msg.cones:
            x = cone.radius * np.cos(cone.angle)
            y = cone.radius * np.sin(cone.angle)

            # go from camera frame to chassis frame
            x += t.transform.translation.x
            y += t.transform.translation.y

            radius = np.hypot(x, y)
            angle = np.arctan2(y, x)

            polar = np.array([radius, angle])
            cone_type = ConeType(cone.cone_type)
            observations.append((polar, cone_type))

        self.slam.step(motion, observations)

        # --- publish new cone positions ---
        slam_cones = self.slam.estimated_landmarks
        blue_cones = []
        yellow_cones = []
        orange_cones = []
        big_orange_cones = []
        unknown_cones = []

        slam_pose = self.slam.estimated_state
        # slam_pose = pose

        # convert to polar
        r = np.hypot(slam_pose[0], slam_pose[1])
        theta = np.arctan2(-slam_pose[1], -slam_pose[0])

        # transform from global to local coord frame
        theta -= np.arctan2(np.sin(slam_pose[2]), np.cos(slam_pose[2]))

        transform_x = r * np.cos(theta)
        transform_y = r * np.sin(theta)

        # we publish the translation between the odom pos and the slam pos,
        # so we can translate the cone positions so that the relative position of the cones
        # to the odom pos stays the same as to the slam pos.

        t = TransformStamped()

        t.header.stamp = cone_msg.header.stamp
        t.header.frame_id = "base_footprint"
        t.child_frame_id = "track"

        # Car only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = transform_x
        t.transform.translation.y = transform_y
        t.transform.translation.z = 0.0

        # For the same reason, car can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = quaternion_from_euler(0, 0, -slam_pose[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

        for c in slam_cones:

            if c.cone_type(self.type_obs_thresh) == ConeType.BLUE:
                blue_cones.append(c.pos())
            elif c.cone_type(self.type_obs_thresh) == ConeType.YELLOW:
                yellow_cones.append(c.pos())
            elif c.cone_type(self.type_obs_thresh) == ConeType.ORANGE:
                orange_cones.append(c.pos())
            elif c.cone_type(self.type_obs_thresh) == ConeType.BIG_ORANGE:
                big_orange_cones.append(c.pos())
            else:
                unknown_cones.append(c.pos())

        cone_array = ConeArray()
        # inherit the timestamp of the measurement
        cone_array.header.stamp = cone_msg.header.stamp
        cone_array.header.frame_id = "track"

        for cone_pos in blue_cones:
            cone = Point()
            cone.x = float(cone_pos[0])
            cone.y = float(cone_pos[1])
            cone.z = 0.0
            cone_array.blue_cones.append(cone)

        for cone_pos in yellow_cones:
            cone = Point()
            cone.x = float(cone_pos[0])
            cone.y = float(cone_pos[1])
            cone.z = 0.0
            cone_array.yellow_cones.append(cone)

        for cone_pos in orange_cones:
            cone = Point()
            cone.x = float(cone_pos[0])
            cone.y = float(cone_pos[1])
            cone.z = 0.0
            cone_array.orange_cones.append(cone)

        for cone_pos in big_orange_cones:
            cone = Point()
            cone.x = float(cone_pos[0])
            cone.y = float(cone_pos[1])
            cone.z = 0.0
            cone_array.big_orange_cones.append(cone)

        for cone_pos in unknown_cones:
            cone = Point()
            cone.x = float(cone_pos[0])
            cone.y = float(cone_pos[1])
            cone.z = 0.0
            cone_array.unknown_color_cones.append(cone)

        self.publisher.publish(cone_array)
        self.__publish_visualisation(blue_cones, yellow_cones, unknown_cones)

    @staticmethod
    def yaw_from_quaternion(q: Quaternion) -> float:
        """Calculate the yaw of an object from a quaternion.

        Args:
            q (Quaternion): the quaternion describing the object's orientation
        Returns:
            float: The yaw of the object
        """
        q0 = q.w
        q1 = q.x
        q2 = q.y
        q3 = q.z

        yaw = np.arctan2(
            2 * ((q1 * q2) + (q0 * q3)), q0**2 + q1**2 - q2**2 - q3**2
        )

        return yaw

    def __publish_visualisation(
        self,
        blue_cones: List[np.ndarray],
        yellow_cones: List[np.ndarray],
        unknown_cones: List[np.ndarray],
    ) -> None:
        marker = Marker()
        marker.header.frame_id = "track"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.action = Marker.ADD
        marker.type = Marker.POINTS

        marker.id = 0
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.ns = "slam"

        for cone in blue_cones:
            marker.points.append(Point(x=float(cone[0]), y=float(cone[1])))

            rgb_values = (0.0, 0.0, 0.9)
            marker.colors.append(
                ColorRGBA(a=1.0, r=rgb_values[0], g=rgb_values[1], b=rgb_values[2])
            )

        for cone in yellow_cones:
            marker.points.append(Point(x=float(cone[0]), y=float(cone[1])))

            rgb_values = (0.9, 0.9, 0.0)
            marker.colors.append(
                ColorRGBA(a=1.0, r=rgb_values[0], g=rgb_values[1], b=rgb_values[2])
            )

        for cone in unknown_cones:
            marker.points.append(Point(x=float(cone[0]), y=float(cone[1])))

            rgb_values = (0.6, 0.6, 0.6)
            marker.colors.append(
                ColorRGBA(a=1.0, r=rgb_values[0], g=rgb_values[1], b=rgb_values[2])
            )

        self.visualization_pub.publish(marker)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
