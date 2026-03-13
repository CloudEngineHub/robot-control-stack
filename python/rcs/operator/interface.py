from abc import ABC
from dataclasses import dataclass
from enum import Enum, auto
import threading
from typing import Protocol
import numpy as np
import rcs
from rcs.envs.base import ArmWithGripper, ControlMode, GripperDictType, JointsDictType, RelativeTo, TQuatDictType, TRPYDictType
from rcs.utils import SimpleFrameRate
import gymnasium as gym


@dataclass(kw_only=True)
class BaseOperatorConfig:
    env_frequency: int = 30

class BaseOperator(ABC, threading.Thread):
    """Interface for a operator device"""

    control_mode: tuple[ControlMode, RelativeTo]

    def __init__(self, env: gym.Env, config: BaseOperatorConfig):
        super().__init__()
        self.config = config
        self.env = env
        self.reset_lock = threading.Lock()


    def run(self):
        # read out hardware, set states and process buttons
        raise NotImplementedError()


    # TODO: use action spaces, needs to return gripper and 
    # TODO: support multiple robots
    def get_action() -> dict[str, ArmWithGripper]:
        raise NotImplementedError()

    def stop(self):
        self._exit_requested = True
        self.join()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *_):
        self.stop()

    def environment_step_loop(self):
        rate_limiter = SimpleFrameRate(self.env, "env loop")
        while True:
            if self._exit_requested:
                break
            with self.reset_lock:
                actions = self.get_action()
                self.env.step(actions)
            rate_limiter()


