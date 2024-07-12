from __future__ import annotations
from typing import Tuple

import numpy as np
from .cone_type import ConeType


class Landmark(object):
    """Data structure of a landmark associated with a particle."""

    def __init__(self, x: float, y: float):
        self.pos_x = x
        self.pos_y = y
        self.mu = np.array([[self.pos_x], [self.pos_y]])
        self.sigma = np.eye(2) * 99
        # Incremented when this landmark is observed,
        # decremented when it is not observed when it should have
        self.counter = 1

        num_types = ConeType.num_types()

        # Uniform distribution of beliefs (the prior)
        self._cone_type = np.array([1 / num_types] * num_types)

    def pos(self) -> np.ndarray:
        """Return the position of this landmark.

        Returns:
            np.ndarray: The position of this landmark
        """
        return np.array([self.pos_x, self.pos_y])

    def update_cone_type(self, cone_type: ConeType, probability: float):
        """Update the type of the cone given a reading.

        Args:
            cone_type (ConeType): The observed type.
            probability (float): The probability the observation is true.

        Raises:
            ValueError: Probability should be between 0.5 and 1.0
        """
        # If we have no type reading then ignore
        if cone_type == ConeType.UNKNOWN:
            return

        if probability < 0.5 or probability >= 1.0:
            raise ValueError("Probability should be between 0.5 and 1.0")

        scale = probability / (1.0 - probability)

        # increase the probability of our given type
        self._cone_type[cone_type.value] *= scale
        # normalise
        self._cone_type /= sum(self._cone_type)

    def cone_type(self, threshold: float) -> ConeType:
        """Get the type of this cone.

        Args:
            threshold (float): The minimum probability threshold.

        Returns:
            ConeType: The cone type.
        """
        # get the index of the cone with the highest probability
        index = np.argmax(self._cone_type)
        probability = self._cone_type[index]

        if probability < threshold:
            return ConeType.UNKNOWN
        else:
            return ConeType(index)

    def update_pos(self, mu: np.ndarray, sig: np.ndarray):
        """Update the position of this cone given a measurement.

        Args:
            mu (np.ndarray): mean.
            sig (np.ndarray): standard deviation.
        """
        self.mu = mu
        self.sigma = sig
        self.pos_x = self.mu[0][0]
        self.pos_y = self.mu[1][0]
        self.counter += 1

    @staticmethod
    def gaussian_mult(
        m1: float, m2: float, s1: float, s2: float
    ) -> Tuple[float, float]:
        """Combine two Gaussian PDFs (normal distributions).

        Args:
            m1 (float): Mu 1
            m2 (float): Mu 2
            s1 (float): Sigma 1
            s2 (float): Sigma 2

        Returns:
            Tuple[float, float]: (mu, sigma)
        """

        mu = (s1 * m2 + s2 * m1) / (s1 + s2)
        sigma = (s1 * s2) / (s1 + s2)

        return mu, sigma

    def combine_with(self, other: Landmark):
        """Merge this landmark with another landmark.

        Combines the Gaussians with Gaussian multiplication.
        Combines the type distributions by summing then normalising.

        Args:
            other (Landmark): The other landmark.
        """
        # gaussian multiplication to combine the landmarks
        x, sigma_x = self.gaussian_mult(
            self.pos_x, other.pos_x, self.sigma[0][0], other.sigma[0][0]
        )
        y, sigma_y = self.gaussian_mult(
            self.pos_y, other.pos_y, self.sigma[1][1], other.sigma[1][1]
        )

        new_mu = np.array([[x], [y]])
        new_sigma = np.array([[sigma_x, 0], [0, sigma_y]])

        self.update_pos(new_mu, new_sigma)

        # Sum the distributions then normalize
        self._cone_type += other._cone_type
        self._cone_type /= sum(self._cone_type)

    def __str__(self):
        return f"Landmark({self.pos_x}, {self.pos_y}, {self.cone_type(0.75).name})"
