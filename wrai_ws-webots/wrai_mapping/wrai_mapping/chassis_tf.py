import math

from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, Quaternion

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
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

    yaw = np.arctan2(2 * ((q1 * q2) + (q0 * q3)), q0**2 + q1**2 - q2**2 - q3**2)

    return yaw


class FramePublisher(Node):
    """Publishes the transform frame for the INS."""

    def __init__(self):
        super().__init__("chassis_tf2_frame_publisher")

        # Initialize the transform broadcaster
        self.tf_buffer = Buffer()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, "/vectornav/pose", self.handle_chassis_pose, 1
        )

    def handle_chassis_pose(self, msg: PoseWithCovarianceStamped):
        """Calculate the transform from chassis to earth.

        Args:
            msg (PoseWithCovarianceStamped): The position of the car.
        """
        # calculate the transform from chassis to earth, which is the reverse
        # of the transform from earth to chassis, provided by the INS

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        r = np.hypot(x, y)
        theta = np.arctan2(-y, -x)

        theta -= np.arctan2(np.sin(yaw), np.cos(yaw))

        x = r * np.cos(theta)
        y = r * np.sin(theta)

        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "base_footprint"
        t.child_frame_id = "earth"

        # Car is only considered in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        # For the same reason, car can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = quaternion_from_euler(0, 0, -yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
