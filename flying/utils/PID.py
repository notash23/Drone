from dataclasses import dataclass
from time import monotonic


@dataclass
class Position:
    """Class for position"""
    x: float
    y: float
    distance: float

    def to_tuple(self):
        return self.x, self.y, self.distance

    def __sub__(self, other):
        return Position(self.x - other.x, self.y - other.y, self.distance - other.distance)

    def __add__(self, other):
        return Position(self.x + other.x, self.y + other.y, self.distance + other.distance)

    def __mul__(self, other: float):
        return Position(self.x * other, self.y * other, self.distance * other)

    def __rmul__(self, other):
        return Position(self.x * other, self.y * other, self.distance * other)

    def __truediv__(self, other: float):
        return Position(self.x / other, self.y / other, self.distance / other)

    def clamp(self, other):
        return Position(min(max(self.x, - other.x), other.x), min(max(self.y, - other.y), other.y), min(max(self.distance, - other.distance), other.distance))

class PID:
    def __init__(self, k_p: float = 0, k_i: float = 0, k_d: float = 0, max_windup: Position = Position(10, 10, 10), robot_position: Position = Position(0.5, 0.5, 0.5)) -> None:
        """
        Initializes the PID. This class is used to find by how much the drone needs to adjust itself based on the error in its position.

        It will consist of 3 parts
         - Error compensation proportional to the error (Increase this to change the position faster)
         - Error compensation proportional to the sum of the error (Increase this for more overshooting)
         - Error compensation proportional to the change in the error (Increase this to be more sensitive
         to quick changes in position and to compensate for the overshooting)

        :param k_p: Proportional part of the PID. Default value is
        :param k_i: Integral part of the PID. Default value is
        :param k_d: Derivative part of the PID. Default value is
        :param robot_position: The position where the robot is preferred to be.
            It is a data class with parameters (x, y, z), where x, y are between 0 and 1.
            (0, 0, z) means top left and
            (1, 1, z) means bottom right.

            The z value is the area of the bounding box of the robot's detection.

            Default value is (0.5, 0.5, 0.5)
        """
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.max_windup = max_windup
        self.robot_position = robot_position

        self.error_sum = Position(0, 0, 0)
        self.prev_error = Position(0, 0, 0)
        self.prev_time = -1

    def get_rc_controls(self, position: Position, timestep: float = None) -> tuple[float, float, float]:
        """
        Uses the PID parameters to find the speed and rotation of the drone

        :param timestep: This is for simulation purposes. Use this parameter to use a simulated clock instead of a real time clock
        :param position: The current position of the robot
            It is a data class with parameters (x, y, z), where x, y are between 0 and 1.
            (0, 0, z) means top left and
            (1, 1, z) means bottom right.

            The z value is the area of the bounding box of the robot's detection.

        :return: A Position object with the yaw, the horizontal speed, and the vertical speed for the drone in that order
        """
        error = self.robot_position - position
        pid = (self._get_p(error) + self._get_i(error, timestep) + self._get_d(error, timestep)).to_tuple()
        self.prev_time = monotonic()
        return pid

    def reset(self) -> None:
        """
        This function resets and running total and the changes in the position's error.
        This is always called whenever the drone exits the 'follow' state.

        :return: None
        """
        self.error_sum = Position(0, 0, 0)
        self.prev_error = Position(0, 0, 0)
        self.prev_time = -1

    def _get_p(self, error: Position) -> Position:
        return self.k_p * error

    def _get_i(self, error: Position, timestep: float = None) -> Position:
        if self.prev_time < 0:
            return Position(0, 0, 0)
        if timestep is None:
            timestep = monotonic() - self.prev_time
        self.error_sum += error * timestep
        self.error_sum.clamp(self.max_windup)
        return self.k_i * self.error_sum

    def _get_d(self, error: Position, timestep: float = None) -> Position:
        if self.prev_time < 0:
            return Position(0, 0, 0)
        if timestep is None:
            timestep = monotonic() - self.prev_time
        error_change = (error - self.prev_error) / timestep
        self.prev_error = error
        return self.k_d * error_change
