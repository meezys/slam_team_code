import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


class OdomBridgeNode(Node):
    """A bridge between WRAI PoseWithCovarianceStamped and the EUFS Odometry."""

    def __init__(self) -> None:
        super().__init__("odom_bridge")

        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped, "/vectornav/pose", 10
        )

        self.subscription = self.create_subscription(
            Odometry, "/ground_truth/odom", self.__odom_callback, 10
        )

    def __odom_callback(self, msg: Odometry) -> None:

        new_msg = PoseWithCovarianceStamped()
        new_msg.header = msg.header
        new_msg.pose = msg.pose
        msg.header.frame_id = "earth"

        self.publisher_.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)

    node = OdomBridgeNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
