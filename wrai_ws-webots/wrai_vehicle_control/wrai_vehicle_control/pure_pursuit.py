from typing import Tuple
import numpy as np
from threading import Lock
from .path import Path


class PurePursuitFollower:
    """A class to generate steering commands that keep a car on a path through pure pursuit.

    Args:
        lookahead_distance (float): the lookahead distance

    """

    def __init__(self, lookahead_distance: float, path: Path) -> None:
        self.last_point_index = 0.0
        self.base_lookahead_distance = lookahead_distance
        self.lookahead_distance = lookahead_distance
        self.lookahead_point = np.array([0, 0])
        self.closest_distance = 0.0  # distance to path
        self.starting = True  # flag set at the start of the run
        self.path: Path = path
        self.first_update = True
        self.mutex = Lock()
        self.new_path_available = True
        self.path_updated = False

    # updates the lookahead point
    def _update_lp(self, pose: np.ndarray) -> None:
        """Get the lookahead point based on the controller's state.

        If in the 'starting' state, then the lookahead point is just the first point on the path.
        If we aren't in that state, then the lookahead point is calculated through the normal
        path intersection method. This function also updates the 'starting' state depending on our
        distance from the first point on the path. The intention of this is to make sure the vehicle
        gets on the path at the start before calculating intersections. This protects against
        situations where the vehicle starts behind the start of the path, and may intersect points
        at the end of the path.

        Args:
            pose (np.ndarray): The pose of the vehicle

        """
        path = self.path
        if self.starting:
            start_point = path.pathpoints[0]
            delta = pose[:2] - start_point
            # once we are close enough to the start point, run as normal
            if np.hypot(delta[0], delta[1]) < self.base_lookahead_distance:
                self.starting = False
                self.lookahead_point = self._get_lookahead_point(pose)
            else:
                # if we haven't made it to the start point, drive towards it
                self.lookahead_point = np.array(start_point)
                self.closest_point_idx = 0  # make sure it gets the correct velocity too
        else:  # not starting, so run as normal
            self.lookahead_point = self._get_lookahead_point(pose)

    def set_path(self, waypoints: np.ndarray) -> None:
        """Sets/Updates the current path.

        Args:
            waypoints (np.ndarray): The new waypoints
        """
        if len(waypoints) == 0 or len(waypoints[0]) == 0:
            return

        self.mutex.acquire(blocking=True)

        try:
            # get the position of the last intersected point
            last_point = self.path.pathpoints[int(np.floor(self.last_point_index))]
            t = self.last_point_index % 1  # get the fraction along the line (0 - 1)

            self.path.update_waypoints(waypoints)

            # the corresponding point on the new path should be the closest one, but in some
            # edge cases (overlapping) it may not be, but we shouldn't have to worry about this
            last_point_index, _ = self._get_closest_point(last_point)
            # technically... the t value might not line up perfectly, but it's close enough.
            self.last_point_index = float(last_point_index) + t
        except IndexError:
            self.path.update_waypoints(waypoints)

        self.path_updated = True
        self.mutex.release()

    def update(self, pose: np.ndarray, wheelbase: float) -> Tuple[float, float]:
        """Run the pure pursuit algorithm, given a pose.

        Args:
            pose (np.ndarray): The pose of the vehicle

        Returns:
            Tuple[float, float]: A tuple of linear velocity, and steer angle
        """

        if len(self.path.waypoints) == 0 or len(self.path.waypoints[0]) == 0:
            return (0, 0)  # do not move if we don't have a path to follow!

        self.mutex.acquire(blocking=True)

        pose = pose.astype("float32")
        if self.first_update:
            self.first_update = False
            self.path.generate_path_segment(pose)

        if self.path_updated:
            self.path_updated = False
            self.path.generate_path_segment(pose)

        # as the car moves along the path, always keep a certain amount of the path generated ahead
        if self.path.should_regenerate_path(pose) or self.new_path_available:
            self.new_path_available = False
            integer_index = int(np.floor(self.last_point_index))
            points_len = len(self.path.pathpoints) - 1
            last_point = self.path.pathpoints[min(integer_index, points_len)]
            t = self.last_point_index % 1  # get the fraction along the line (0 - 1)

            self.path.generate_path_segment(pose)

            # the corresponding point on the new path should be the closest one, but in some
            # edge cases (overlapping) it may not be, but we shouldn't have to worry about this
            last_point_index, _ = self._get_closest_point(last_point)
            # technically... the t value might not line up perfectly, but it's close enough.
            self.last_point_index = float(last_point_index) + t

        path = self.path

        # if we reached the end of the path, reset so we can go back round
        # only switch to the beginning of the path if we are close enough to intersect it
        delta = pose[:2] - path.pathpoints[0]
        if (
            int(np.round(self.last_point_index)) >= len(path.pathpoints) - 1
            and np.hypot(delta[0], delta[1]) < self.base_lookahead_distance
        ):
            self.last_point_index = 0

        (self.closest_point_idx, self.closest_point) = self._get_closest_point(pose)
        # print(np.linalg.norm(self.closest_point - pose[:2]))
        # curvature = PurePursuitFollower.get_curvature_of_path(
        #     self.closest_point, path.pathpoints)

        # get the new lookahead point
        self._update_lp(pose)

        # lp_curvature = path.get_curvature_of_path(self.lookahead_point, path.pathpoints)
        curvature_to_lp = self._get_curvature_to_point(pose, self.lookahead_point)
        steer_angle = np.arctan(wheelbase * curvature_to_lp)  # effectively arctan(L/R)

        speed = path.velocities[int(np.floor(self.closest_point_idx))]

        self.lookahead_distance = self._adaptive_lookahead_distance(speed)

        self.mutex.release()
        return (speed, -steer_angle)

    def _adaptive_lookahead_distance(self, speed: float) -> float:

        # If we've gone way off, we need to turn the lookahead distance above what we initially set
        # do 2*d so that it picks a point a little ahead
        if self.closest_distance > self.base_lookahead_distance:
            return 2 * self.closest_distance

        lookahead_min = 1.25
        lookahead_max = 5
        lookahead_scale = 0.9
        return np.clip(lookahead_min, speed * lookahead_scale, lookahead_max)

    # gets the lookahead point for a given path, lookahead distance and robot position
    def _get_lookahead_point(self, pose: np.ndarray) -> np.ndarray:
        """Get the lookahead point by intersecting the lookahead circle with the path.

        Only calculate intersections ahead of the path from the previous intersection.

        Args:
            pose (np.ndarray): the current vehicle pose

        Returns:
            np.ndarray: the coordinates of the intersection point.
        """
        path = self.path
        center = pose[0:2]  # Center of the lookahead circle
        # Radius of the lookahead circle is look ahead distance
        r = self.lookahead_distance

        # loop through all points, from the last point we looked at, up to the second-to-last point
        for i in range(int(np.floor(self.last_point_index)), len(path.pathpoints) - 1):
            segment_start = path.pathpoints[i]
            segment_end = path.pathpoints[i + 1]

            d = segment_end - segment_start  # segment vector
            f = segment_start - center  # vector from car to segment start

            # Calculate the coeffecients of the quadratic to solve
            # ax^2 + bx + c
            a = np.dot(d, d)
            b = 2 * np.dot(f, d)
            c = np.dot(f, f) - (r * r)

            poly = np.polynomial.Polynomial([c, b, a])
            roots = poly.roots()
            # take only the real parts. we count a number with a very small imaginary component as
            # real (thanks to numerical error)
            roots = roots.real[abs(roots.imag) < 1e-5]
            # if no root exists then we will set it as -1 to be discarded later
            t1 = roots[0] if len(roots) > 0 else -1
            t2 = roots[1] if len(roots) > 1 else -1

            t = -1.0
            if t1 >= 0 and t1 <= 1:
                t = t1
            if t2 >= 0 and t2 <= 1 and t2 >= t:
                t = t2

            if t >= 0 and (i + t) >= self.last_point_index:
                self.last_point_index = i + t
                intersection_point = (t * d) + segment_start
                return intersection_point

        # no intersections with the path, so return the last point we did intersect
        if int(np.ceil(self.last_point_index)) >= len(path.pathpoints):
            return path.pathpoints[-1]
        else:
            segment_start = path.pathpoints[int(np.floor(self.last_point_index))]
            segment_end = path.pathpoints[int(np.ceil(self.last_point_index))]

            t = self.last_point_index % 1  # get just the fractional part
            d = segment_end - segment_start

            intersection_point = (t * d) + segment_start
            return intersection_point

    def _get_closest_point(self, pose: np.ndarray) -> Tuple[int, np.ndarray]:
        """Calculate the closest point on the path to a position.

        Args:
            pose (np.ndarray): the pose to calculate the closest point for

        Returns:
            Tuple[int, np.ndarray]: A tuple of the index of the closest path point, and the point
        """
        path = self.path

        closest_point = path.pathpoints[0]
        closest_idx = 0
        closest_distance_sqd = np.inf
        bot_pos = pose[0:2]

        for (i, p) in enumerate(path.pathpoints):
            delta = p - bot_pos
            distance_sqd = np.dot(delta, delta)
            if distance_sqd < closest_distance_sqd:
                closest_point = p
                closest_distance_sqd = distance_sqd
                closest_idx = i

        self.closest_distance = np.sqrt(closest_distance_sqd)
        return (closest_idx, closest_point)

    def _get_curvature_to_point(
        self, robot_pose: np.ndarray, point: np.ndarray
    ) -> float:
        """Calculate the signed curvature of the arc connecting a pose and the point."""
        # curvature formula: 2x/L^2

        robot_x, robot_y, robot_theta = robot_pose[0], robot_pose[1], robot_pose[2]

        # the "robot line", the line going through the robot, in the direction the robot is facing
        # ax+by+c=0
        a = -np.tan(robot_theta)
        b = 1
        c = (np.tan(robot_theta) * robot_x) - robot_y

        # find the distance between the point and the "robot line"
        # point line distance formula
        x = np.abs(a * point[0] + b * point[1] + c) / np.hypot(a, b)
        ld = self.lookahead_distance
        curvature = (2 * x) / (ld * ld)

        # since the point line distance formula doesn't give us which side of the line
        # the point is on, we need to calculate that ourself
        side = np.sign(
            (np.sin(robot_theta) * (point[0] - robot_x))
            - (np.cos(robot_theta) * (point[1] - robot_y))
        )

        # return a signed curvature (how much, and in which direction to turn)
        return side * curvature
