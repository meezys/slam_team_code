import rclpy
import numpy as np
import colorsys

from std_msgs.msg import Header, ColorRGBA
from rclpy.node import Node
from rclpy.time import Time
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import Quaternion, Point, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from wrai_msgs.msg import Midpoints
from .pure_pursuit import PurePursuitFollower
from .path import Path

# TODO: get midpoints from ROS topic

# TODO: KD Tree for closest point on path
# TODO: use lookahead point finding optimisation
# TODO: output lots of relevant debug values as ros topics for debugging (lookahead distance etc.)
# TODO: deal with consistent timing (make sure we are outputting commands at some high update rate)
# TODO: deal with adaptive lookahead (give it a parameter)
# TODO: segments size as parameter
# TODO: comment
# TODO: smoothing function in c++, get python to call c func

# TODO: test with changing path, increasingly growing path x, and path with injected noise x,
# incomplete path x, empty path x
# TODO: deal with path changing (find closest point on new path that matches
# last point on old path etc.)
# TODO: only generate path for next few midpoints in front of the car when on learning lap
# TODO: path generation performance (tune tolerance, distance between points)
# TODO: action to switch to mapped mode and use a map of the whole track
# TODO: get pure pursuit to form the loop closure itself, aka not require the start point at the
# end of the path
# TODO: if waypoints is empty, submit a 0,0 command

# distance between front and rear axles (m)
EUFS_WHEELBASE = 1.58
ADS_DV_WHEELBASE = 1.532


class PurePursuitNode(Node):
    """The ROS Node that drives the Pure Pursuit Controller."""

    def __init__(self) -> None:
        super().__init__("pure_pursuit")

        # Declare parameters

        # how much to smooth the path, higher is smoother (0 - 1)
        path_smoothing = self.declare_parameter("path_smoothing", 0.95).value
        # the space inbetween injected points (m)
        path_spacing = self.declare_parameter("path_spacing", 0.2).value
        # how well to smooth the path, lower number is a higher quality path
        path_tolerance = self.declare_parameter("path_tolerance", 0.1).value

        # pure pursuit lookahead distance (m)
        lookahead_distance = self.declare_parameter("lookahead_distance", 2.5).value
        # maximum amount of braking (ms^-2)
        max_deceleration = self.declare_parameter("max_deceleration", 1.0).value
        # length of the vehicle wheelbase (m)
        self.declare_parameter("wheelbase", EUFS_WHEELBASE)
        # minimum vehicle speed (ms^-1)
        min_speed = self.declare_parameter("min_speed", 0.5).value
        # maximum vehicle speed (ms^-1)
        max_speed = self.declare_parameter("max_speed", 3.0).value
        # maximum centripetal acceleration,
        # this affects the velocity the vehicle will take corners at (ms^-2)
        max_centripetal_accel = self.declare_parameter(
            "max_centripetal_accel", 5.0
        ).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher_ = self.create_publisher(AckermannDriveStamped, "cmd", 10)
        # self.publisher_ = self.create_publisher(AckermannDriveStamped, "lookahead_distance", 10)
        # self.publisher_ = self.create_publisher(PointStamped, "lookahead_point", 10)
        self.visualization_pub = self.create_publisher(Marker, "/planner/viz", 1)

        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, "/vectornav/pose", self.odom_truth_callback, 10
        )

        self.subscription = self.create_subscription(
            Midpoints, "/midpoints", self.midpoints_callback, 10
        )

        points = np.array([[]], dtype=np.float32)
        # start = time.perf_counter_ns()
        self.path = Path(
            points,
            max_decel=max_deceleration,
            b=path_smoothing,
            spacing=path_spacing,
            tolerance=path_tolerance,
            min_speed=min_speed,
            max_speed=max_speed,
            max_centripetal_accel=max_centripetal_accel,
        )
        # end = time.perf_counter_ns()
        # print(f"path generation took {(end-start)/(10**9)} s")

        self.follower = PurePursuitFollower(lookahead_distance, self.path)

    def odom_truth_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """Publish an Ackermann drive command based on a new odometry update.

        Args:
            msg (Odometry): The new Odometry message
        """
        full_pose = msg.pose.pose

        # Lookup the transform between the INS frame (earth frame) and the track frame
        try:
            from_frame = "earth"
            to_frame = "track"
            # time of 0 gets the latest transform it can
            t = self.tf_buffer.lookup_transform(
                to_frame, from_frame, Time(seconds=0, nanoseconds=0)
            )
        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform {to_frame} to {from_frame}: {ex}"
            )
            return

        # apply transformations
        position = np.array(
            [
                full_pose.position.x + t.transform.translation.x,
                full_pose.position.y + t.transform.translation.y,
            ]
        )
        yaw = self.yaw_from_quaternion(
            full_pose.orientation
        ) + self.yaw_from_quaternion(t.transform.rotation)
        # x, y, theta
        pose = np.array([position[0], position[1], yaw])

        commands = self.calculate_commands(pose)

        self.publish_visualisation(self.follower.path.pathpoints)

        # setup and publish command message
        control_msg = AckermannDriveStamped()
        control_msg.header = Header()
        control_msg.drive = AckermannDrive()

        control_msg.header.frame_id = ""
        control_msg.header.stamp = self.get_clock().now().to_msg()

        control_msg.drive.steering_angle = float(commands[1])  # rads
        control_msg.drive.steering_angle_velocity = 0.0
        control_msg.drive.speed = float(commands[0])  # speed in m/s
        control_msg.drive.acceleration = 0.0
        control_msg.drive.jerk = 0.0

        self.publisher_.publish(control_msg)

    def midpoints_callback(self, msg: Midpoints) -> None:
        """Publish an Ackermann drive command based on a new odometry update.

        Args:
            msg (Odometry): The new Odometry message
        """
        midpoints = msg.midpoints
        midpoints = list(map(lambda m: [m.x, m.y], midpoints))
        points = np.array(midpoints, dtype=np.float32)
        self.follower.set_path(points)

    def publish_visualisation(self, points: np.ndarray):
        """Publish markers that show the position of the path being followed.

        Args:
            points (np.ndarray): The points to visualise
        """
        marker = Marker()
        marker.header.frame_id = "track"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.action = Marker.ADD
        marker.type = Marker.LINE_STRIP

        marker.id = 0
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.ns = "wrai_vehicle_control"

        for i in range(len(points)):
            point = points[i]
            if len(point) <= 1:
                continue

            marker.points.append(Point(x=float(point[0]), y=float(point[1])))

            rgb_values = colorsys.hsv_to_rgb(i / len(points), 1.0, 1.0)
            marker.colors.append(
                ColorRGBA(a=1.0, r=rgb_values[0], g=rgb_values[1], b=rgb_values[2])
            )

        self.visualization_pub.publish(marker)

    def calculate_commands(self, pose: np.ndarray) -> np.ndarray:
        """Run the path following algorithm for a given pose and returns steering commands.

        Args:
            pose (np.ndarray): the current post

        Returns:
            np.ndarray: commands(linear velocity, steering angle)
        """
        wheelbase = self.get_parameter("wheelbase").value
        (v, steer_angle) = self.follower.update(pose, wheelbase)

        if abs(v) <= 0:
            steer_angle = 0

        return np.array((v, steer_angle))

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


def main(args=None):
    rclpy.init(args=args)

    pure_pursuit_node = PurePursuitNode()

    rclpy.spin(pure_pursuit_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
