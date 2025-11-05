import logging
from time import sleep

import numpy as np
from rcs._core.common import RobotPlatform
from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.creators import SimEnvCreator
from rcs_ur5e.creators import RCSUR5eEnvCreator
from rcs_ur5e.hw import UR5eConfig

import rcs
from rcs import sim

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

ROBOT_IP = "192.168.25.201"
ROBOT_INSTANCE = RobotPlatform.SIMULATION  
# ROBOT_INSTANCE = RobotPlatform.HARDWARE


def main():

    if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
        robot_cfg = UR5eConfig()
        env_rel = RCSUR5eEnvCreator()(
            control_mode=ControlMode.JOINTS,
            robot_cfg=robot_cfg,
            ip=ROBOT_IP,
            camera_set=None,
            max_relative_movement=np.deg2rad(5),
            relative_to=RelativeTo.LAST_STEP,
        )
    else:
        robot_cfg = sim.SimRobotConfig()
        robot_cfg.actuators = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
        robot_cfg.joints = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        robot_cfg.robot_type = rcs.common.RobotType.UR5e
        robot_cfg.attachment_site = "attachment_site"
        robot_cfg.arm_collision_geoms = []
        robot_cfg.mjcf_scene_path = rcs.scenes["ur5e_empty_world"].mjb
        robot_cfg.kinematic_model_path = rcs.scenes["ur5e_empty_world"].mjcf_robot
        robot_cfg.base = "base"

        gripper_config = sim.SimGripperConfig()
        gripper_config.actuator = "fingers_actuator"
        gripper_config.joint = "right_driver_joint"
        gripper_config.collision_geoms = []
        gripper_config.collision_geoms_fingers = []

        env_rel = SimEnvCreator()(
            control_mode=ControlMode.JOINTS,
            collision_guard=False,
            robot_cfg=robot_cfg,
            gripper_cfg=gripper_config,
            max_relative_movement=np.deg2rad(5),
            relative_to=RelativeTo.LAST_STEP,
        )
        env_rel.get_wrapper_attr("sim").open_gui()

    for _ in range(100):
        obs, info = env_rel.reset()
        for _ in range(3):
            # sample random relative action and execute it
            act = env_rel.action_space.sample()
            obs, reward, terminated, truncated, info = env_rel.step(act)
            if truncated or terminated:
                logger.info("Truncated or terminated!")
                return
            sleep(0.4)


if __name__ == "__main__":
    main()
