import rclpy

from rclpy.node import Node
from typing import List
from math import hypot, atan2

from wrai_msgs.msg import ConeMeasurementArray, ConeMeasurement
from eufs_msgs.msg import ConeArrayWithCovariance as EUFSConeArray
from eufs_msgs.msg import ConeWithCovariance
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


class ConesBridgeNode(Node):
    """A bridge between WRAI ConeMeasurementArray and the EUFS ConeArray."""

    def __init__(self) -> None:
        super().__init__("cones_bridge")

        self.publisher_ = self.create_publisher(
            ConeMeasurementArray, "/camera_cones", 10
        )

        self.subscription = self.create_subscription(
            EUFSConeArray, "/cones", self.__cone_callback, 10
        )

        self.subscription = self.create_subscription(
            Odometry, "/ground_truth/odom", self.__odom_callback, 10
        )

        self.car_pos = [0, 0]
        self.car_yaw = 0.0

    def __cone_callback(self, msg: EUFSConeArray) -> None:
        wrai_msg = ConeMeasurementArray()

        wrai_msg.header = msg.header

        blue_cones = self.__convert_cones(
            msg.blue_cones, ConeMeasurement.CONE_TYPE_BLUE
        )
        yellow_cones = self.__convert_cones(
            msg.yellow_cones, ConeMeasurement.CONE_TYPE_YELLOW
        )
        orange_cones = self.__convert_cones(
            msg.orange_cones, ConeMeasurement.CONE_TYPE_ORANGE
        )
        big_orange_cones = self.__convert_cones(
            msg.big_orange_cones, ConeMeasurement.CONE_TYPE_BIG_ORANGE
        )
        unknown_color_cones = self.__convert_cones(
            msg.unknown_color_cones, ConeMeasurement.CONE_TYPE_UNKNOWN
        )

        cones = []
        cones.extend(blue_cones)
        cones.extend(yellow_cones)
        cones.extend(orange_cones)
        cones.extend(big_orange_cones)
        cones.extend(unknown_color_cones)

        wrai_msg.cones = cones

        self.publisher_.publish(wrai_msg)

    def __odom_callback(self, msg: Odometry) -> None:
        full_pose = msg.pose.pose

        self.car_pos = [full_pose.position.x, full_pose.position.y]
        self.car_yaw = self.yaw_from_quaternion(full_pose.orientation)

    def __convert_cones(
        self, cones: List[ConeWithCovariance], cone_type: int
    ) -> List[ConeMeasurement]:
        converted_cones = []
        for cone in cones:
            point = cone.point

            r = hypot(point.x, point.y)
            theta = atan2(point.y, point.x)

            measurement = ConeMeasurement()
            measurement.radius = r
            measurement.angle = theta
            measurement.cone_type = cone_type

            converted_cones.append(measurement)

        return converted_cones

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

        yaw = atan2(2 * ((q1 * q2) + (q0 * q3)), q0**2 + q1**2 - q2**2 - q3**2)

        return yaw


def main(args=None):
    rclpy.init(args=args)

    node = ConesBridgeNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
