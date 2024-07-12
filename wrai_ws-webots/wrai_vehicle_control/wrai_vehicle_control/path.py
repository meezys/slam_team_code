import numpy as np
import numba
from numba.experimental import jitclass


@jitclass(
    [
        ("spacing", numba.float32),
        ("a", numba.float32),
        ("b", numba.float32),
        ("tolerance", numba.float32),
        ("pathpoints", numba.float32[:, :]),
        ("velocities", numba.float32[:]),
        ("waypoints", numba.float32[:, :]),
        ("max_decel", numba.float32),
        ("min_speed", numba.float32),
        ("max_speed", numba.float32),
        ("max_centripetal_accel", numba.float32),
        ("last_waypoint", numba.int32),
        ("segment_length", numba.int32),
    ]
)
class Path:
    """Generates a smooth path and a velocity profile given a set of waypoints."""

    def __init__(
        self,
        waypoints: np.ndarray,
        max_decel: float,
        spacing: float,
        b: float,
        tolerance: float,
        min_speed: float,
        max_speed: float,
        max_centripetal_accel: float,
    ) -> None:
        self.waypoints = waypoints
        self.spacing = spacing
        self.b = b
        self.a = 1 - self.b
        self.tolerance = tolerance

        self.max_decel = max_decel
        self.min_speed = min_speed
        self.max_speed = max_speed
        self.max_centripetal_accel = max_centripetal_accel

        self.last_waypoint = 0
        self.segment_length = 10

        # self.generate_path_from_waypoints(waypoints)

    @staticmethod
    def inject_points(control_points: np.ndarray, spacing: float) -> np.ndarray:
        """Inject more points into a path to increase point density.

        Args:
            control_points (np.ndarray): The waypoints to inject more points inbetween
            spacing (float): The spacing of the new injected points

        Returns:
            np.ndarray: the modified path
        """

        size_estimate = 1
        for i in range(len(control_points) - 1):
            start = control_points[i]
            end = control_points[i + 1]

            vector = end - start
            magnitude = np.hypot(vector[0], vector[1])
            points_that_fit = np.floor(magnitude / spacing)
            size_estimate += points_that_fit

        path_points = np.zeros((int(size_estimate), 2), dtype=np.float32)
        k = 0
        for i in range(len(control_points) - 1):
            start = control_points[i]
            end = control_points[i + 1]

            vector = end - start
            magnitude = np.hypot(vector[0], vector[1])
            points_that_fit = np.floor(magnitude / spacing)

            vector = (vector / magnitude) * spacing
            for i in range(int(points_that_fit)):
                new_point = start + (vector * i)
                path_points[k] = new_point
                k += 1

        path_points[k] = control_points[-1]

        return path_points

    @staticmethod
    def smooth(points: np.ndarray, a: float, b: float, tolerance: float) -> np.ndarray:
        """Smooth a path to the given smoothness.

        Args:
            points (np.ndarray): the path to smooth.
            a (float): how much to keep the original data. Set to 1-b.
            b (float): How much to smooth. range of 0 to 1.
            tolerance (float): Error tolerance, decides when the smoothing ends.

        Returns:
            np.ndarray: The smoothed path.
        """

        change = tolerance
        new_path_points = np.copy(points)

        while change >= tolerance:
            change = 0
            for i in range(1, len(points) - 2):
                old_point = np.copy(points[i])
                new_point = np.copy(new_path_points[i])

                new_path_points[i] += a * (old_point - new_point) + b * (
                    new_path_points[i - 1] + new_path_points[i + 1] - (2.0 * new_point)
                )
                point_delta = new_point - new_path_points[i]
                change += np.hypot(point_delta[0], point_delta[1])
        return new_path_points

    def velocity_at_curvature(
        self,
        curvature: float,
        min_speed: float,
        max_speed: float,
        max_centripetal_accel: float,
    ) -> float:
        """Determine velocity for a given curvature.

        Args:
            curvature (float): The curvature.
            min_speed (float): The vehicle min speed.
            max_speed (float): The vehicle max speed.
            max_centripetal_accel (float): The max centripetal accel.

        Returns:
            float: The velocity for this curvature.
        """
        r = 1.0 / np.max(np.array([curvature, 0.01], dtype=np.float32))
        # use centripetal accel formula sqrt(|R| * a)
        v = np.sqrt(abs(r) * max_centripetal_accel)

        # set a minimum and max speed
        if v > max_speed:
            return max_speed
        elif v < min_speed:
            return min_speed
        else:
            return v

    def calculate_velocities(
        self,
        points: np.ndarray,
        max_decel: float,
        min_speed: float,
        max_speed: float,
        max_centripetal_accel: float,
    ) -> np.ndarray:
        """Generate the velocity profile for a path.

        Args:
            points (np.ndarray): the path.
            max_decel (float): Vehicle max braking.
            min_speed (float): Vehicle min speed.
            max_speed (float): Vehicle max speed.
            max_centripetal_accel (float): Max centripetal accel.

        Returns:
            np.ndarray: The velocity profile.
        """
        velocities = np.zeros(len(points), dtype=np.float32)
        cumulative_distances = []

        cumulative_distance = 0.0

        # do a first pass, calculating target velocities directly from curvature
        for (i, point) in enumerate(points):
            # sum up the distances
            if i != 0:
                delta = point - points[i - 1]
                cumulative_distance += np.hypot(delta[0], delta[1])

            cumulative_distances.append(cumulative_distance)

            curvature = self.get_curvature_of_path(point, points)

            v = self.velocity_at_curvature(
                curvature, min_speed, max_speed, max_centripetal_accel
            )

            velocities[i] = v

        # second pass to limit deceleration
        # go through the list backwards, starting at the second to last point

        for (i, point) in list(enumerate(points))[-2::-1]:
            vthis = velocities[i]
            vlast = velocities[i + 1]

            a = max_decel
            d = cumulative_distances[i + 1] - cumulative_distances[i]

            vmax = np.sqrt((vlast * vlast) + 2 * a * d)

            if vthis > vmax:
                velocities[i] = vmax

        # from matplotlib import pyplot as plt
        # plt.plot(list(range(len(curvatures))), curvatures)
        # plt.plot(list(range(len(curvatures))), velocities)
        # plt.show()
        return velocities

    def get_curvature_of_path(self, point: np.ndarray, pathpoints: np.ndarray) -> float:
        """Given a point on a path, find the curvature of the path at that point."""
        # We can't get the curvature of the first or last points,
        # since we need one point ahead and one behind
        if (point == pathpoints[0]).all() or (point == pathpoints[-1]).all():
            return 0.0

        q = np.zeros(2, dtype=np.float32)  # start of line
        r = np.zeros(2, dtype=np.float32)  # end of line

        for i in range(len(pathpoints) - 1):
            # we are going to check if our point lies on the line between q and r
            q = pathpoints[i]
            r = pathpoints[i + 1]

            # get length^2 of all these vectors
            qp = np.dot(q - point, q - point)
            rp = np.dot(r - point, r - point)
            qr = np.dot(q - r, q - r)

            # if the length of qp and rp sum to qr, then p is on the line
            # check its less than a small number to account for floating point errors
            if (qp + rp) - qr < 0.0001:
                # if our current point *is* a pathpoint,
                # then we need to go one more backwards and one more forwards
                if i <= 1 or i >= len(pathpoints) - 2:
                    return 0.0
                q = pathpoints[i - 1]
                r = pathpoints[i + 2]

                break

        x1, y1 = point
        x2, y2 = q
        x3, y3 = r

        # if these two numbers are equal, we will divide by zero
        # add a very small amount so we don't
        if x1 == x2:
            x1 += 0.0001

        # finally, use a formula to calculate the curvature
        k1 = 0.5 * ((x1 * x1) + (y1 * y1) - (x2 * x2) - (y2 * y2)) / (x1 - x2)
        k2 = (y1 - y2) / (x1 - x2)

        b = (
            0.5
            * (
                (x2 * x2)
                - 2 * x2 * k1
                + (y2 * y2)
                - (x3 * x3)
                + 2 * x3 * k1
                - (y3 * y3)
            )
            / ((x3 * k2 - y3 + y2 - x2 * k2) + 0.0001)
        )
        a = k1 - k2 * b
        r = np.hypot(x1 - a, y1 - b)
        curvature: float = 1.0 / r  # type: ignore

        if np.isnan(curvature):
            curvature = 0.0

        # calculate circle center for testing purposes
        # D = 2 * ((x1*(y2 - y3)) + (x2*(y3-y1)) + (x3*(y1-y2)))

        # self.cx = (1/D) * ((x1**2 + y1**2)*(y2-y3) + (x2**2 + y2**2)
        #    * (y3-y1) + (x3**2 + y3**2)*(y1-y2))
        # self.cy = (1/D) * ((x1**2 + y1**2)*(x3-x2) + (x2**2 + y2**2)
        #    * (x1-x3) + (x3**2 + y3**2)*(x2-x1))
        # self.path_curvature = curvature

        return curvature

    def update_waypoints(self, waypoints: np.ndarray) -> None:
        """Update the waypoints for this path.

        Args:
            waypoints (np.ndarray): The new waypoints for this path.
        """
        self.waypoints = waypoints
        # TODO: regenerate path

    def should_regenerate_path(self, pose: np.ndarray) -> bool:
        """Determine whether the current path that the path needs to be regenerated.

        The critera is whether the vehicle has gone far enough along the path to pass
        the second waypoint in the path. The second waypoint should become the first
        waypoint of the new path.

        Args:
            pose (np.ndarray): Current vehicle pose.

        Returns:
            bool: Whether to regenerate.
        """
        # alternative to this would be regenerating if the car is closer to the point 2 in front
        # than the current point
        car_pos = pose[0:2]  # x and y
        num_waypoints = len(self.waypoints)

        last_waypoint = self.waypoints[self.last_waypoint]
        next_waypoint = self.waypoints[(self.last_waypoint + 1) % num_waypoints]

        if (
            abs(last_waypoint[0] - next_waypoint[0]) < 0.01
            and abs(last_waypoint[1] - next_waypoint[1]) < 0.01
        ):
            return True  # break out early if points are equal

        d = next_waypoint - last_waypoint
        d_len = np.linalg.norm(d)
        d_norm = d / d_len

        car_start_dist = np.linalg.norm(car_pos - last_waypoint)
        projection = np.dot(car_pos - last_waypoint, d_norm)

        # If the car has gone past this line segment, update the waypoints
        # assumption: the car position won't jitter back and forth across boundaries
        return projection > d_len or (car_start_dist > d_len and projection > 0)
        # return projection > d_len
        # return np.dot(car_pos - next_waypoint, d) > 0.0

    def generate_path_segment(self, pose: np.ndarray):
        """Generate a new path of a given length.

        Calculates the new last_waypoint to start the new path from.
        Starts from the last_waypoint and loops around if the path is complete.

        Args:
            pose (np.ndarray): Current vehicle pose.
        """
        num_waypoints = len(self.waypoints)

        while self.should_regenerate_path(pose):
            # go on to the next waypoint, and wrap around too
            self.last_waypoint += 1
            self.last_waypoint = self.last_waypoint % num_waypoints

        # if number of waypoints is less than the desired path length
        segment_length = min(self.segment_length, num_waypoints)

        # add the next n waypoints, but be able to loop around
        waypoints = np.zeros((segment_length, 2))

        j = self.last_waypoint  # index into the original path
        true_length = 0
        for i in range(segment_length):
            # check if the path loops, if so then reset the counter so we
            # add points from the start of the path
            if np.array_equal(self.waypoints[j], self.waypoints[0]) and j != 0:
                j = 0
            elif j >= num_waypoints:
                break

            waypoints[i] = self.waypoints[j]
            j += 1
            true_length += 1

        # slice waypoints so any stray 0s at the end aren't included
        self.generate_path_from_waypoints(waypoints[0:true_length])

    def generate_path_from_waypoints(self, waypoints: np.ndarray):
        """Generate a path.

        The underlying function that takes in a set of waypoints and
        performs all the calculations.

        Args:
            waypoints (np.ndarray): The waypoints of the path.
        """
        self.pathpoints = self.inject_points(waypoints, self.spacing)
        self.pathpoints = self.smooth(self.pathpoints, self.a, self.b, self.tolerance)
        self.velocities = self.calculate_velocities(
            self.pathpoints,
            self.max_decel,
            self.min_speed,
            self.max_speed,
            self.max_centripetal_accel,
        )
