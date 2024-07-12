from enum import Enum


class ConeType(Enum):
    """The type of a cone."""

    BLUE = 0
    YELLOW = 1
    ORANGE = 2
    BIG_ORANGE = 3

    UNKNOWN = 255

    @staticmethod
    def num_types() -> int:
        """Return the number of cone types.

        Returns:
            int: The number of types
        """
        return 4
