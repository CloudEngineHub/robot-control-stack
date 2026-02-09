
from typing import Protocol

import numpy as np


class BaseTorqueSensor(Protocol):

    def dof(self) -> int:
        """Returns robots DoF"""

    def torque(self) -> np.ndarray:
        """returns current tau external"""
