import numpy as np
from typing import List
from scipy.spatial import KDTree
from typing import Tuple

from .util import multi_normal, gauss_noise, constrain_angle_pm
from .landmark import Landmark
from .cone_type import ConeType


class Particle:
    """Contains a single localisation estimate, and a list of landmark estimates."""

    def __init__(
        self,
        start_state: np.ndarray,
        association_threshold: float,
        landmark_combination_threshold: float,
        obs_distance_thresh: float,
        obs_angle_thresh: float,
        type_obs_prob: float,
        motion_dist_noise: float,
        motion_angle_noise: float,
        obs_dist_noise: float,
        obs_angle_noise: float,
    ) -> None:
        """Initialise the particle."""
        self.pos_x = start_state[0]
        self.pos_y = start_state[1]
        self.heading = start_state[2]

        self.landmarks: List[Landmark] = []
        self.lm_tree: KDTree = None

        self.weight = 1.0
        # Model error term will relax the covariance matrix
        self.obs_noise = np.array([[obs_dist_noise, 0], [0, obs_angle_noise]])
        self.type_obs_prob = type_obs_prob

        self.association_threshold = association_threshold
        # self.association_distance_thresh = 3
        # self.association_angle_thresh = 0.07

        self.landmark_combination_threshold = landmark_combination_threshold
        self.obs_distance_thresh = obs_distance_thresh
        self.obs_angle_thresh = obs_angle_thresh

        # self.bearing_noise = 0.1
        # self.distance_noise = 0.1
        self.motion_noise = motion_dist_noise
        self.turning_noise = motion_angle_noise

    def pose(self) -> np.ndarray:
        """Return this particle's current pose.

        Returns:
            np.ndarray: The current pose
        """
        return np.array((self.pos_x, self.pos_y, self.heading))

    def set_pose(self, x: float, y: float, orien: float):
        """Set the pose of the particle.

        Args:
            x (float): x pos
            y (float): y pos
            orien (float): heading
        """

        self.pos_x = x
        self.pos_y = y
        self.heading = orien

    def update(self, obs: List[Tuple[np.ndarray, ConeType]]):
        """Update the weight of the particle and its EKFs based on observations."""

        landmark_seen = np.full((len(self.landmarks),), False)

        for polar_pos, cone_type in obs:
            prob = np.exp(-70)

            obs_distance, obs_angle = polar_pos

            if self.landmarks:
                # start = time.perf_counter_ns()
                # find the data association with ML
                (
                    distance,
                    prob,
                    landmark_idx,
                    assoc_obs,
                    assoc_jacobian,
                    assoc_adjcov,
                ) = self.find_data_association(polar_pos)

                lm_pos = self.landmarks[landmark_idx].pos()
                lm_angle = (
                    np.arctan2(lm_pos[1] - self.pos_y, lm_pos[0] - self.pos_x)
                    - self.heading
                )

                angle_diff = obs_angle - lm_angle
                # smallest angle between
                angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))

                # landmarks are distance thresholded
                if (
                    distance
                    > self.association_threshold
                    # or abs(angle_diff) > self.association_angle_thresh
                ):
                    self._create_landmark(polar_pos, cone_type)

                    # extend the landmarks seen by 1
                    landmark_seen = np.concatenate((landmark_seen, np.array([True])))
                else:
                    landmark_seen[landmark_idx] = True  # we've now seen this landmark
                    # update corresponding EKF with new observation

                    self.update_landmark(
                        np.transpose(np.array([polar_pos])),
                        landmark_idx,
                        assoc_obs,
                        assoc_jacobian,
                        assoc_adjcov,
                        self.type_obs_prob,
                        cone_type,
                    )
                # end = time.perf_counter_ns()
                # print(f"assoc took {(end-start)/10**6} ms")
            else:
                # no initial landmarks
                self._create_landmark(polar_pos, cone_type)
                # extend the landmarks seen by 1
                landmark_seen = np.concatenate((landmark_seen, np.array([True])))
            self.weight *= prob

        # If a landmark should have been observed and it wasn't, decrement its counter
        for i in range(len(landmark_seen)):
            if not landmark_seen[i] and self._should_see_landmark(self.landmarks[i]):
                # print(f"decrementing landmark {self.landmarks[i]}")
                self.landmarks[i].counter -= 1

        # remove any landmarks that have counters below 0
        # This is done so that landmarks that should have been observed, but have not for a while
        # are removed, since they are probably false positives
        new_landmarks = []
        for landmark in self.landmarks:
            if landmark.counter >= 0:
                new_landmarks.append(landmark)
        self.landmarks = new_landmarks

        # Combines landmarks that are within some threshold, through Gaussian multiplication so
        # their covariance is taken into account.
        # This is done because duplicate landmarks close to eachother are
        # just duplicates of the same cone

        self._generate_tree()
        # get all the pairs of points that have a distance within the threshold
        pairs = self.lm_tree.query_pairs(self.landmark_combination_threshold)
        # repeat until there are no more pairs
        while pairs:
            i, j = pairs.pop()

            # get the first landmark
            lm1 = self.landmarks[i]
            # remove and get the second landmark
            lm2 = self.landmarks.pop(j)
            # combine landmarks
            lm1.combine_with(lm2)

            # regen pairs
            self._generate_tree()
            pairs = self.lm_tree.query_pairs(self.landmark_combination_threshold)

    def move(self, motion: np.ndarray) -> None:
        """Motion model.

        Moves particle forward of distance d plus gaussian noise.
        Maybe a better model is noise in each direction? or do a proper bicycle model.
        """
        # d = motion[0]  # distance moved forward
        # t = motion[1]  # change in heading
        self.pos_x = self.pos_x + motion[0] + gauss_noise(0, self.motion_noise)
        self.pos_y = self.pos_y + motion[1] + gauss_noise(0, self.motion_noise)

        self.heading = (
            self.heading + (motion[2] + gauss_noise(0, self.turning_noise))
        ) % (2 * np.pi)

    def compute_jacobians(
        self, landmark: Landmark
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Compute jacobians, predicted observation, and measurement covariance.

        Args:
            landmark (Landmark): The landmark to predict for.

        Returns:
            Tuple[np.ndarray, np.ndarray, np.ndarray]: Predicted observation, jacobian, covariance
        """
        dx = landmark.pos_x - self.pos_x
        dy = landmark.pos_y - self.pos_y
        d2 = dx**2 + dy**2
        d = np.sqrt(d2)

        heading = constrain_angle_pm(self.heading)
        pred_angle = np.arctan2(dy, dx) - heading
        pred_angle = constrain_angle_pm(pred_angle)
        predicted_obs = np.array([[d], [pred_angle]])

        jacobian = np.array([[dx / d, dy / d], [-dy / d2, dx / d2]])

        # Measurement covariance
        adj_cov = (
            jacobian.dot(landmark.sigma).dot(np.transpose(jacobian)) + self.obs_noise
        )
        return predicted_obs, jacobian, adj_cov

    def guess_landmark(self, obs) -> Landmark:
        """Based on the particle position and observation, guess the location of the landmark."""
        distance, direction = obs
        lm_x = self.pos_x + distance * np.cos(self.heading + direction)
        lm_y = self.pos_y + distance * np.sin(self.heading + direction)
        return Landmark(lm_x, lm_y)

    def find_data_association(self, obs):
        """Use a threshold distance to find data association."""
        prob = 0
        dist = np.inf
        assoc_obs = np.zeros((2, 1))
        assoc_jacobian = np.zeros((2, 2))
        assoc_adjcov = np.zeros((2, 2))
        landmark_idx = -1
        lm_estimate = self.guess_landmark(obs)

        dist, landmark_idx = self.lm_tree.query(lm_estimate.pos())

        if landmark_idx >= len(self.landmarks) or landmark_idx < 0:
            least_distance_sqd = np.inf
            # standard brute force search
            for idx, landmark in enumerate(self.landmarks):
                delta = lm_estimate.pos() - landmark.pos()
                distance_sqd = np.dot(delta, delta)
                if distance_sqd < least_distance_sqd:
                    least_distance_sqd = distance_sqd
                    landmark_idx = idx

            dist = np.sqrt(least_distance_sqd)

        if landmark_idx >= len(self.landmarks) or landmark_idx < 0:
            print("Warning: Invalid state! the landmarks are probably invalid")
            return np.inf, 0, 0, assoc_obs, assoc_jacobian, assoc_adjcov

        landmark = self.landmarks[landmark_idx]
        predicted_obs, jacobian, adj_cov = self.compute_jacobians(landmark)
        p = multi_normal(np.transpose(np.array([obs])), predicted_obs, adj_cov)
        prob = p
        assoc_obs = predicted_obs
        assoc_jacobian = jacobian
        assoc_adjcov = adj_cov

        return dist, prob, landmark_idx, assoc_obs, assoc_jacobian, assoc_adjcov

    def _create_landmark(self, obs, cone_type):
        landmark = self.guess_landmark(obs)
        landmark.update_cone_type(cone_type, self.type_obs_prob)
        self.landmarks.append(landmark)
        self._generate_tree()

    def _generate_tree(self):
        self.lm_tree = KDTree([lm.pos() for lm in self.landmarks])

    def _should_see_landmark(self, landmark: Landmark):
        distance_thresh = self.obs_distance_thresh
        angle_thresh = self.obs_angle_thresh

        dx = landmark.pos_x - self.pos_x
        dy = landmark.pos_y - self.pos_y

        distance = np.hypot(dx, dy)
        angle = np.arctan2(dy, dx) - self.heading

        return distance < distance_thresh and abs(angle) < angle_thresh

    def update_landmark(
        self,
        obs: np.ndarray,
        landmark_idx: int,
        assoc_obs: np.ndarray,
        assoc_jacobian: np.ndarray,
        assoc_adjcov: np.ndarray,
        type_prob: float,
        cone_type: ConeType = ConeType.UNKNOWN,
    ):
        """Update a landmark with an EKF.

        Args:
            obs (np.ndarray): The new observation.
            landmark_idx (int): Which landmark to update.
            assoc_obs (np.ndarray): The predicted observation.
            assoc_jacobian (np.ndarray): The landmark jacobian.
            assoc_adjcov (np.ndarray): The landmark covariance.
            type_prob (float): The probability of the type observation.
            cone_type (ConeType, optional): The observed cone type. Defaults to ConeType.UNKNOWN.
        """
        landmark = self.landmarks[landmark_idx]
        k_gain = landmark.sigma.dot(np.transpose(assoc_jacobian)).dot(
            np.linalg.inv(assoc_adjcov)
        )
        new_mu = landmark.mu + k_gain.dot(obs - assoc_obs)
        new_sigma = (np.eye(2) - k_gain.dot(assoc_jacobian)).dot(landmark.sigma)
        landmark.update_pos(new_mu, new_sigma)

        landmark.update_cone_type(cone_type, type_prob)

    def __str__(self):
        return str((self.pos_x, self.pos_y, self.heading, self.weight))
