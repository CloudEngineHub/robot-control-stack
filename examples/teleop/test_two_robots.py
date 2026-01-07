
import logging
import threading
from time import sleep

import numpy as np
import rcs
from rcs._core import common
from rcs._core.common import RPY, Pose, RobotPlatform
from rcs._core.sim import CameraType, SimCameraConfig, SimConfig
from rcs.camera.hw import HardwareCameraSet
from rcs.envs.base import (
    ControlMode,
    GripperDictType,
    LimitedTQuatRelDictType,
    RelativeActionSpace,
    RelativeTo,
)
from rcs.envs.creators import SimMultiEnvCreator
from rcs.envs.storage_wrapper import StorageWrapper
from rcs.envs.utils import default_sim_gripper_cfg, default_sim_robot_cfg
from rcs.utils import SimpleFrameRate
from rcs_fr3.creators import RCSFR3MultiEnvCreator
from rcs_fr3.utils import default_fr3_hw_gripper_cfg, default_fr3_hw_robot_cfg
# from rcs_realsense.utils import default_realsense
from simpub.core.simpub_server import SimPublisher
from simpub.parser.simdata import SimObject, SimScene
from simpub.sim.mj_publisher import MujocoPublisher
from simpub.xr_device.meta_quest3 import MetaQuest3


logger = logging.getLogger(__name__)


ROBOT2ID = {
    "left": "0",
    "right": "1",
}


ROBOT_INSTANCE = RobotPlatform.SIMULATION

def main():
    # FR3
    rcs.scenes["rcs_icra_scene"] = rcs.Scene(
        mjcf_scene="/home/tobi/coding/rcs_clones/prs/models/scenes/rcs_icra_scene/scene.xml",
        mjcf_robot=rcs.scenes["fr3_simple_pick_up"].mjcf_robot,
        robot_type=common.RobotType.FR3,
    )
    rcs.scenes["pick"] = rcs.Scene(
        mjcf_scene="/home/tobi/coding/rcs_clones/prs/models/scenes/rcs_icra_scene/scene.xml",
        mjcf_robot=rcs.scenes["fr3_simple_pick_up"].mjcf_robot,
        robot_type=common.RobotType.FR3,
    )

    # robot_cfg = default_sim_robot_cfg("fr3_empty_world")
    # robot_cfg = default_sim_robot_cfg("fr3_simple_pick_up")
    robot_cfg = default_sim_robot_cfg("rcs_icra_scene")
    # robot_cfg = default_sim_robot_cfg("pick")

    resolution = (256, 256)
    cameras = {
        cam: SimCameraConfig(
            identifier=cam,
            type=CameraType.fixed,
            resolution_height=resolution[1],
            resolution_width=resolution[0],
            frame_rate=0,
        )
        for cam in ["side", "wrist"]
    }

    sim_cfg = SimConfig()
    sim_cfg.async_control = False
    env_rel = SimMultiEnvCreator()(
        name2id=ROBOT2ID,
        robot_cfg=robot_cfg,
        control_mode=ControlMode.CARTESIAN_TQuat,
        gripper_cfg=default_sim_gripper_cfg(),
        # cameras=cameras,
        max_relative_movement=0.5,
        # relative_to=RelativeTo.CONFIGURED_ORIGIN,
        relative_to=RelativeTo.LAST_STEP,
        sim_cfg=sim_cfg,
    )

    # env_rel = StorageWrapper(
    #     env_rel, DATASET_PATH, INSTRUCTION, batch_size=32, max_rows_per_group=100, max_rows_per_file=1000
    # )
    # sim = env_rel.unwrapped.envs[ROBOT2ID.keys().__iter__().__next__()].sim  # type: ignore
    sim = env_rel.get_wrapper_attr("sim")

    sim.open_gui()

    env_rel.reset()
    sleep(3)

    for _ in range(100):
        for _ in range(10):
            # act = {"left": {"tquat": [0.0, 0, 0, 0, 0, 0, 1], "gripper": 0}, "right": {"tquat": [0.0, 0, 0, 0, 0, 0, 1], "gripper": 0}}
            # print(act)
            # move 1cm in x direction (forward) and close gripper
            act = {"left": {"tquat": [0.01, 0, 0, 0, 0, 0, 1], "gripper": 0}, "right": {"tquat": [-0.01, 0, 0, 0, 0, 0, 1], "gripper": 0}}
            obs, reward, terminated, truncated, info = env_rel.step(act)
            sleep(0.1)
        for _ in range(10):
            # move 1cm in negative x direction (backward) and open gripper
            act = {"left": {"tquat": [-0.01, 0, 0, 0, 0, 0, 1], "gripper": 1}, "right": {"tquat": [0.01, 0, 0, 0, 0, 0, 1], "gripper": 1}}
            obs, reward, terminated, truncated, info = env_rel.step(act)
            sleep(0.1)


if __name__ == "__main__":
    main()
