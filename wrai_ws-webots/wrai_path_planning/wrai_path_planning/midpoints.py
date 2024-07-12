import rclpy
import numpy as np
import math
import colorsys

from rclpy.node import Node
from scipy.spatial import Delaunay
from typing import List, Tuple

from geometry_msgs.msg import Point
from wrai_msgs.msg import Midpoints, ConeArray
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

# TODO: If we only see 2/3 cones, calculate a simple midpoint without Delaunay and use that.


class MidpointsNode(Node):
    """The ROS Node that generates the midpoints between cones."""

    def __init__(self, name: str):
        super().__init__(name)

        # Declare ROS parameters
        self.distance_between_cones = self.declare_parameter(
            "distance_between_cones", 7.0
        ).value

        self.first_midpoint = None  # this gets set in the first iteration

        # Create subscribers
        self.cone_sub = self.create_subscription(
            ConeArray, "/cones", self.cone_callback, 1
        )

        # Create publishers
        self.midpoint_pub = self.create_publisher(Midpoints, "/midpoints", 1)

        self.visualization_pub = self.create_publisher(Marker, "/midpoint/viz", 1)

    def cone_callback(self, msg: ConeArray) -> None:
        """Generate the midpoints from a list of cones.

        Args:
            msg (ConeArray): The list of cones.
        """
        # We can't publish anything without cones
        # we require at least 4 cones for Delaunay
        if len(msg.blue_cones) < 2 or len(msg.yellow_cones) < 2:
            return

        # Convert ConeArray to usable data.
        cones = []
        for cone in msg.blue_cones:
            cones.append((cone.x, cone.y, "b"))
        for cone in msg.yellow_cones:
            cones.append((cone.x, cone.y, "y"))

        midpoints = self.find_midpoints(cones)
        midpoints = np.unique(midpoints, axis=0)  # remove duplicate midpoints
        midpoints = self.order_midpoints(midpoints)

        delta = midpoints[-1] - midpoints[0]
        distance = np.hypot(delta[0], delta[1])  # distance between start and end
        # TODO: fix jank
        if distance < 10 and len(midpoints) > 10:
            # add the first midpoint to the end to signify a closed loop
            midpoints = np.append(midpoints, [midpoints[0]], axis=0)

        new_msg = Midpoints()
        new_msg.header.stamp = msg.header.stamp
        new_msg.header.frame_id = "track"
        for m in midpoints:
            p = Point()
            p.x = float(m[0])
            p.y = float(m[1])
            p.z = 0.0
            new_msg.midpoints.append(p)

        self.midpoint_pub.publish(new_msg)

        self.publish_visualisation(midpoints)

    def find_midpoints(self, cones: List[Tuple[float, float, str]]) -> np.ndarray:
        """Generate midpoints from a list of cones.

        Args:
            cones (List[Tuple[float,float,str]]): The list of cones
        """
        valid_cones = []
        points = []
        for c in cones:
            # only pick out the positions of the yellow or blue cones
            if c[2] == "y" or c[2] == "b":
                valid_cones.append(c)
                points.append(c[:2])

        # calculate the delaunany triangulation between all the cones
        tri = Delaunay(points)

        simplices = tri.simplices

        lines = set()
        for i1, i2, i3 in simplices:
            lines.add((i1, i2))
            lines.add((i1, i3))
            lines.add((i2, i3))

        midpoints = []
        for i1, i2 in lines:
            c1 = valid_cones[i1]
            c2 = valid_cones[i2]

            # if cone color is not the same and the distance between them is less than or equal to
            # the distance between cones
            distance = self.find_dis_points(c1, c2)
            if c1[2] != c2[2] and distance <= self.distance_between_cones:
                # find the midpoint between the two cones
                mid = self.find_midpoint(c1, c2)
                # add the midpoint to the list of midpoints
                midpoints.append(mid)

        # convert the mid points into an array
        return np.array(np.array(midpoints)[:, :2], dtype=np.float32)

    # find the distance between two points given
    def find_dis_points(self, point1, point2):
        """Find the distance between two points.

        Args:
            point1 (List): The first point
            point2 (List): The second point

        Returns:
            float: The distance between the points.
        """
        # work out the distance between two points using the maths library
        distance = math.hypot(point2[0] - point1[0], point2[1] - point1[1])
        return distance

    # find the midpoint between two points
    def find_midpoint(self, point1, point2):
        """Find the middle between two points.

        Args:
            point1 (List): The first point
            point2 (List): The second point

        Returns:
            List: The middle point
        """
        midpoint = []  # setup a list to store the x and y coordinates
        midpoint.append((point1[0] + point2[0]) / 2)  # x coordinate
        midpoint.append((point1[1] + point2[1]) / 2)  # y coordinate
        return midpoint

    def order_midpoints(self, midpoints: np.ndarray) -> np.ndarray:
        """Order midpoints so they form a path.

        Args:
            midpoints (np.ndarray): The midpoints

        Returns:
            np.ndarray: The ordered midpoints
        """
        idx = 0  # the index of the first cone
        if self.first_midpoint is None:
            # get the point closest to the origin
            self.first_midpoint = min(midpoints, key=lambda x: np.dot(x, x))
        start_point = self.first_midpoint

        points = list(midpoints)

        # figure out the index of the first point
        # do this by finding the point closest to the start
        closest_start = np.inf
        for i, p in enumerate(points):
            delta = p - start_point
            d = np.hypot(delta[0], delta[1])

            if d < closest_start:
                closest_start = d
                idx = i

        # initialize a new list of points with the known first point
        ordered_points = [points.pop(idx)]
        # initialize the current point (as the known point)
        pcurr = ordered_points[0]
        while len(points) > 0:
            # distances between pcurr and all other remaining points
            d = np.linalg.norm(np.array(points) - np.array(pcurr), axis=1)
            idx = d.argmin()  # index of the closest point
            # append the closest point to points_new
            next_point = points.pop(idx)

            # if the potential connection between midpoints is too long, then it is invalid
            if d[idx] > 4.5:
                continue

            ordered_points.append(next_point)
            pcurr = ordered_points[-1]  # update the current point
        return np.array(ordered_points)

    def publish_visualisation(self, points: np.ndarray):
        """Publish markers that show the position of the path being followed.

        Args:
            points (np.ndarray): The points to visualise
        """
        marker = Marker()
        marker.header.frame_id = "track"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.action = Marker.ADD
        marker.type = Marker.POINTS

        marker.id = 0
        marker.scale.x = 0.45
        marker.scale.y = 0.45
        marker.ns = "midpoints"

        for i in range(len(points)):
            point = points[i]
            if len(point) <= 1:
                continue

            marker.points.append(Point(x=float(point[0]), y=float(point[1])))

            # rgb_values = (0.8, 0.8, 0.98)
            rgb_values = colorsys.hsv_to_rgb(i / len(points), 0.7, 1.0)
            marker.colors.append(
                ColorRGBA(a=1.0, r=rgb_values[0], g=rgb_values[1], b=rgb_values[2])
            )

        self.visualization_pub.publish(marker)


def main():
    rclpy.init(args=None)
    node = MidpointsNode("midpoints")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
