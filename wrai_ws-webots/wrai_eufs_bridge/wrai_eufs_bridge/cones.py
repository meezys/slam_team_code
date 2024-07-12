import rclpy

from rclpy.node import Node
from typing import List
from math import hypot, atan2, sin, cos

from wrai_msgs.msg import ConeArray
from eufs_msgs.msg import ConeArrayWithCovariance as EUFSConeArray
from eufs_msgs.msg import ConeWithCovariance
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA


class ConesBridgeNode(Node):
    """A bridge between WRAI ConeArray and the EUFS ConeArray."""

    def __init__(self) -> None:
        super().__init__("cones_bridge")

        # Declare parameters

        # path_smoothing = self.declare_parameter("path_smoothing", 0.95).value

        self.publisher_ = self.create_publisher(ConeArray, "/wrai/cones", 10)
        self.visualization_pub = self.create_publisher(Marker, "/bridge/viz", 1)

        self.subscription = self.create_subscription(
            EUFSConeArray, "/ground_truth/cones", self.__cone_callback, 10
        )

        self.subscription = self.create_subscription(
            Odometry, "/ground_truth/odom", self.__odom_callback, 10
        )

        self.car_pos = [0, 0]
        self.car_yaw = 0.0

    def __cone_callback(self, msg: EUFSConeArray) -> None:
        wrai_msg = ConeArray()

        wrai_msg.header = msg.header

        wrai_msg.blue_cones = self.__convert_cones(msg.blue_cones)
        wrai_msg.yellow_cones = self.__convert_cones(msg.yellow_cones)
        wrai_msg.orange_cones = self.__convert_cones(msg.orange_cones)
        wrai_msg.big_orange_cones = self.__convert_cones(msg.big_orange_cones)
        wrai_msg.unknown_color_cones = self.__convert_cones(msg.unknown_color_cones)

        self.publisher_.publish(wrai_msg)
        self.__publish_visualisation(wrai_msg.blue_cones, wrai_msg.yellow_cones)

    def __odom_callback(self, msg: Odometry) -> None:
        full_pose = msg.pose.pose

        self.car_pos = [full_pose.position.x, full_pose.position.y]
        self.car_yaw = self.yaw_from_quaternion(full_pose.orientation)

    def __convert_cones(self, cones: List[ConeWithCovariance]) -> List[Point]:
        converted_cones = []
        for cone in cones:
            point = cone.point

            r = hypot(point.x, point.y)
            theta = atan2(point.y, point.x)

            theta += self.car_yaw

            new_point = Point()
            new_point.x = self.car_pos[0] + (r * cos(theta))
            new_point.y = self.car_pos[1] + (r * sin(theta))
            converted_cones.append(new_point)

        return converted_cones

    def __publish_visualisation(
        self, blue_cones: List[Point], yellow_cones: List[Point]
    ) -> None:
        marker = Marker()
        marker.header.frame_id = "base_footprint"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.action = Marker.ADD
        marker.type = Marker.POINTS

        marker.id = 0
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.ns = "cones"

        for cone in blue_cones:
            marker.points.append(cone)

            rgb_values = (0.0, 0.0, 0.9)
            marker.colors.append(
                ColorRGBA(a=1.0, r=rgb_values[0], g=rgb_values[1], b=rgb_values[2])
            )

        for cone in yellow_cones:
            marker.points.append(cone)

            rgb_values = (0.9, 0.9, 0.0)
            marker.colors.append(
                ColorRGBA(a=1.0, r=rgb_values[0], g=rgb_values[1], b=rgb_values[2])
            )

        self.visualization_pub.publish(marker)

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
