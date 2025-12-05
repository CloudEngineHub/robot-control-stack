import logging
import threading
from time import sleep
# from simpub.core.node_manager import init_xr_node_manager
# from simpub.xr_device.meta_quest3 import MetaQuest3

from simpub.core.node_manager import init_xr_node_manager
from simpub.xr_device.meta_quest3 import MetaQuest3

import numpy as np
from rcs._core.common import RPY, Pose, RobotPlatform
from rcs.camera.hw import HardwareCameraSet
from rcs.envs.base import (
    ControlMode,
    GripperDictType,
    LimitedTQuatRelDictType,
    RelativeActionSpace,
    RelativeTo,
)
from rcs.envs.creators import SimEnvCreator
from rcs.envs.utils import default_sim_gripper_cfg, default_sim_robot_cfg
from rcs.utils import SimpleFrameRate
from rcs_fr3.creators import RCSFR3MultiEnvCreator
from rcs_fr3.utils import default_fr3_hw_gripper_cfg, default_fr3_hw_robot_cfg
from rcs_realsense.utils import default_realsense
from rcs_toolbox.wrappers.recorder import StorageWrapperParquet

# from rcs_xarm7.creators import RCSXArm7EnvCreator

logger = logging.getLogger(__name__)

# in order to use usb connection install adb
# sudo apt install android-tools-adb
# download the iris apk from the following repo release: https://github.com/intuitive-robots/IRIS-Meta-Quest3
# install it on your quest with
# adb install IRIS-Meta-Quest3.apk


INCLUDE_ROTATION = True
ROBOT2IP = {
    "left": "192.168.101.1",
    "right": "192.168.102.1",
}


# ROBOT_INSTANCE = RobotPlatform.SIMULATION
ROBOT_INSTANCE = RobotPlatform.HARDWARE
RECORD_FPS = 30
# set camera dict to none disable cameras
# CAMERA_DICT = {
#     "side_right": "244222071045",
#     "bird_eye": "243522070364",
#     "arro": "243522070385",
# }
CAMERA_DICT = None
MQ3_ADDR = "192.168.0.134"



class QuestReader(threading.Thread):

    transform_to_robot = Pose(RPY(roll=np.deg2rad(90), yaw=np.deg2rad(-90)))

    def __init__(self, env: RelativeActionSpace):
        super().__init__()
        # self._reader = oculus_reader.OculusReader()

        net_manager = init_xr_node_manager(MQ3_ADDR)
        net_manager.start_discover_node_loop()
        self._reader = MetaQuest3("RCSNode")

        self._resource_lock = threading.Lock()
        self._env_lock = threading.Lock()
        self._env = env

        self.controller_names = ["left", "right"]
        self._trg_btn = {"left": "index_trigger", "right": "index_trigger"}
        self._grp_btn = {"left": "hand_trigger", "right": "hand_trigger"}
        self._clb_btn = {"left": "X", "right": "A"}
        self._home_btn = "B"

        self._prev_data = None
        self._exit_requested = False
        self._grp_pos = {key: 1.0 for key in self.controller_names}  # start with opened gripper
        self._last_controller_pose = {key: Pose() for key in self.controller_names}
        self._offset_pose = {key: Pose() for key in self.controller_names}

        for robot in ROBOT2IP:
            # self._env.unwrapped.get_wrapper_attr("set_origin_to_current")[robot]()
            # self._env.unwrapped.set_origin_to_current[robot]()
            # self._env.unwrapped.envs[robot].set_origin_to_current()
            self._env.envs[robot].set_origin_to_current()

        self._step_env = False
        self._set_frame = {key: Pose() for key in self.controller_names}

    def next_action(self) -> Pose:
        with self._resource_lock:
            transforms = {}
            for controller in self.controller_names:
                transform = Pose(
                    translation=(
                        self._last_controller_pose[controller].translation()
                        - self._offset_pose[controller].translation()
                    ),
                    quaternion=(
                        self._last_controller_pose[controller] * self._offset_pose[controller].inverse()
                    ).rotation_q(),
                )

                set_axes = Pose(quaternion=self._set_frame[controller].rotation_q())

                transform = (
                    QuestReader.transform_to_robot
                    * set_axes.inverse()
                    * transform
                    * set_axes
                    * QuestReader.transform_to_robot.inverse()
                )
                if not INCLUDE_ROTATION:
                    transform = Pose(translation=transform.translation())  # identity rotation
                transforms[controller] = transform
            return transforms, {key: self._grp_pos[key] for key in self.controller_names}

    def run(self):
        rate_limiter = SimpleFrameRate(90, "teleop readout")
        warning_raised = False
        while not self._exit_requested:
            # pos, buttons = self._reader.get_transformations_and_buttons()
            input_data = self._reader.get_controller_data()
            if not warning_raised and input_data:
                logger.warning("[Quest Reader] packets empty")
                warning_raised = True
                sleep(0.5)
                continue
            elif input_data:
                sleep(0.5)
                continue
            elif warning_raised:
                logger.warning("[Quest Reader] packets arriving again")
                warning_raised = False


            for controller in self.controller_names:
                last_controller_pose = Pose(
                    translation=np.array(input_data[controller]["pos"]), quaternion=np.array(input_data[controller]["rot"])
                )

                # if input_data[self._clb_btn[controller]] and (
                #     self._prev_data is None or not self._prev_data[self._clb_btn[controller]]
                # ):
                    # print("clb button pressed")
                    # with self._resource_lock:
                    #     self._set_frame[controller] = last_controller_pose

                if input_data[controller][self._trg_btn[controller]] and (
                    self._prev_data is None or not self._prev_data[controller][self._trg_btn[controller]]
                ):
                    # trigger just pressed (first data sample with button pressed)

                    with self._resource_lock:
                        self._offset_pose[controller] = last_controller_pose
                        self._last_controller_pose[controller] = last_controller_pose

                elif not input_data[controller][self._trg_btn[controller]] and (
                    self._prev_data is None or self._prev_data[controller][self._trg_btn[controller]]
                ):
                    # released
                    transform = Pose(
                        translation=(
                            self._last_controller_pose[controller].translation()
                            - self._offset_pose[controller].translation()
                        ),
                        quaternion=(
                            self._last_controller_pose[controller] * self._offset_pose[controller].inverse()
                        ).rotation_q(),
                    )
                    print(np.linalg.norm(transform.translation()))
                    print(np.rad2deg(transform.total_angle()))
                    with self._resource_lock:
                        self._last_controller_pose[controller] = Pose()
                        self._offset_pose[controller] = Pose()
                    with self._env_lock:
                        self._env.envs[controller].set_origin_to_current()

                elif input_data[controller][self._trg_btn[controller]]:
                    # button is pressed
                    with self._resource_lock:
                        self._last_controller_pose[controller] = last_controller_pose
                else:
                    # trg button is not pressed
                    # TODO: deactivated for now
                    # if data["buttons"][self._home_btn] and (
                    #     self._prev_data is None or not self._prev_data["buttons"][self._home_btn]
                    # ):
                    #     # home button just pressed
                    #     with self._env_lock:
                    #         self._env.unwrapped.robot.move_home()
                    #         self._env.set_origin_to_current()
                    pass

                if input_data[controller][self._grp_btn[controller]] and (
                    self._prev_data is None or not self._prev_data[controller][self._grp_btn[controller]]
                ):
                    # just pressed
                    self._grp_pos[controller] = 0
                if not input_data[controller][self._grp_btn[controller]] and (
                    self._prev_data is None or self._prev_data[controller][self._grp_btn[controller]]
                ):
                    # just released
                    self._grp_pos[controller] = 1

            self._prev_data = input_data
            rate_limiter()

    def stop(self):
        self._exit_requested = True
        self.join()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *_):
        self.stop()

    def stop_env_loop(self):
        self._step_env = False

    def environment_step_loop(self):
        rate_limiter = SimpleFrameRate(RECORD_FPS, "env loop")
        self._step_env = True
        while self._step_env:
            if self._exit_requested:
                self._step_env = False
                break
            transforms, grippers = self.next_action()
            actions = {}
            for robot, transform in transforms.items():
                action = dict(
                    LimitedTQuatRelDictType(tquat=np.concatenate([transform.translation(), transform.rotation_q()]))
                )

                action.update(GripperDictType(gripper=grippers[robot]))
                actions[robot] = action

            with self._env_lock:
                self._env.step(actions)
            rate_limiter()


def main():
    if ROBOT_INSTANCE == RobotPlatform.HARDWARE:

        camera_set = HardwareCameraSet([default_realsense(CAMERA_DICT)]) if CAMERA_DICT is not None else None
        env_rel = RCSFR3MultiEnvCreator()(
            name2ip=ROBOT2IP,
            camera_set=camera_set,
            robot_cfg=default_fr3_hw_robot_cfg(async_control=True),
            control_mode=ControlMode.CARTESIAN_TQuat,
            gripper_cfg=default_fr3_hw_gripper_cfg(async_control=True),
            max_relative_movement=(0.5, np.deg2rad(90)),
            relative_to=RelativeTo.CONFIGURED_ORIGIN,
        )
    else:
        # FR3
        robot_cfg = default_sim_robot_cfg("fr3_empty_world")

        env_rel = SimEnvCreator()(
            robot_cfg=robot_cfg,
            control_mode=ControlMode.CARTESIAN_TQuat,
            collision_guard=False,
            gripper_cfg=default_sim_gripper_cfg(),
            # cameras=default_mujoco_cameraset_cfg(),
            max_relative_movement=0.5,
            relative_to=RelativeTo.CONFIGURED_ORIGIN,
        )
        env_rel.get_wrapper_attr("sim").open_gui()

    env_rel.reset()

    with env_rel, QuestReader(env_rel) as action_server:
        action_server.environment_step_loop()


if __name__ == "__main__":
    main()
