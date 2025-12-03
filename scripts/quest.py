import logging
import threading
from time import sleep

import numpy as np
import oculus_reader
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
from rcs_fr3.creators import RCSFR3EnvCreator
from rcs_fr3.utils import default_fr3_hw_gripper_cfg, default_fr3_hw_robot_cfg
from rcs_realsense.utils import default_realsense

# from rcs_xarm7.creators import RCSXArm7EnvCreator

logger = logging.getLogger(__name__)

# in order to use usb connection install adb
# sudo apt install android-tools-adb
# and run the following forwarding
# adb reverse tcp:5037 tcp:5037


INCLUDE_ROTATION = True
ROBOT_IP = "192.168.101.1"

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


class QuestReader(threading.Thread):

    transform_to_robot = Pose(RPY(roll=np.deg2rad(90), yaw=np.deg2rad(-90)))

    def __init__(
        self, env: RelativeActionSpace, trg_btn="RTr", grp_btn="RG", controller="r", clb_btn="A", home_btn="B"
    ):
        super().__init__()
        self._reader = oculus_reader.OculusReader()

        self._resource_lock = threading.Lock()
        self._env_lock = threading.Lock()
        self._env = env
        self._trg_btn = trg_btn
        self._grp_btn = grp_btn
        self._home_btn = home_btn
        self._clb_btn = clb_btn
        self._controller = controller
        self._grp_pos = 1
        self._prev_data = None
        self._exit_requested = False
        self._last_controller_pose = Pose()
        self._offset_pose = Pose()
        self._env.set_origin_to_current()
        self._step_env = False
        self._set_frame = Pose()

    def next_action(self) -> Pose:
        with self._resource_lock:
            transform = Pose(
                translation=(self._last_controller_pose.translation() - self._offset_pose.translation()),
                quaternion=(self._last_controller_pose * self._offset_pose.inverse()).rotation_q(),
            )

            set_axes = Pose(quaternion=self._set_frame.rotation_q())

            transform = (
                QuestReader.transform_to_robot
                * set_axes.inverse()
                * transform
                * set_axes
                * QuestReader.transform_to_robot.inverse()
            )
            if not INCLUDE_ROTATION:
                transform = Pose(translation=transform.translation())  # identity rotation
            return transform, self._grp_pos

    def run(self):
        rate_limiter = SimpleFrameRate(90, "teleop readout")
        warning_raised = False
        while not self._exit_requested:
            pos, buttons = self._reader.get_transformations_and_buttons()
            if not warning_raised and len(pos) == 0:
                logger.warning("[Quest Reader] packets empty")
                warning_raised = True
                sleep(0.5)
                continue
            elif len(pos) == 0:
                sleep(0.5)
                continue
            elif warning_raised:
                logger.warning("[Quest Reader] packets arriving again")
                warning_raised = False

            data = {"pos": pos, "buttons": buttons}

            last_controller_pose = Pose(
                pose_matrix=np.array(data["pos"][self._controller]),
            )

            if data["buttons"][self._clb_btn] and (
                self._prev_data is None or not self._prev_data["buttons"][self._clb_btn]
            ):
                print("clb button pressed")
                with self._resource_lock:
                    self._set_frame = last_controller_pose

            if data["buttons"][self._trg_btn] and (
                self._prev_data is None or not self._prev_data["buttons"][self._trg_btn]
            ):
                # trigger just pressed (first data sample with button pressed)

                with self._resource_lock:
                    self._offset_pose = last_controller_pose
                    self._last_controller_pose = last_controller_pose

            elif not data["buttons"][self._trg_btn] and (
                self._prev_data is None or self._prev_data["buttons"][self._trg_btn]
            ):
                # released
                transform = Pose(
                    translation=(self._last_controller_pose.translation() - self._offset_pose.translation()),
                    quaternion=(self._last_controller_pose * self._offset_pose.inverse()).rotation_q(),
                )
                print(np.linalg.norm(transform.translation()))
                print(np.rad2deg(transform.total_angle()))
                with self._resource_lock:
                    self._last_controller_pose = Pose()
                    self._offset_pose = Pose()
                with self._env_lock:
                    self._env.set_origin_to_current()

            elif data["buttons"][self._trg_btn]:
                # button is pressed
                with self._resource_lock:
                    self._last_controller_pose = last_controller_pose
            else:
                # trg button is not pressed
                if data["buttons"][self._home_btn] and (
                    self._prev_data is None or not self._prev_data["buttons"][self._home_btn]
                ):
                    # home button just pressed
                    with self._env_lock:
                        self._env.unwrapped.robot.move_home()
                        self._env.set_origin_to_current()

            if data["buttons"][self._grp_btn] and (
                self._prev_data is None or not self._prev_data["buttons"][self._grp_btn]
            ):
                # just pressed
                self._grp_pos = 0
            if not data["buttons"][self._grp_btn] and (
                self._prev_data is None or self._prev_data["buttons"][self._grp_btn]
            ):
                # just released
                self._grp_pos = 1

            self._prev_data = data
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
        self._env.set_origin_to_current()
        rate_limiter = SimpleFrameRate(RECORD_FPS, "env loop")
        self._step_env = True
        while self._step_env:
            if self._exit_requested:
                self._step_env = False
                break
            transform, gripper = self.next_action()
            action = dict(
                LimitedTQuatRelDictType(tquat=np.concatenate([transform.translation(), transform.rotation_q()]))
            )

            action.update(GripperDictType(gripper=gripper))

            with self._env_lock:
                self._env.step(action)
            rate_limiter()


def main():
    if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
        # xarm7
        # env_rel = RCSXArm7EnvCreator()(
        #     control_mode=ControlMode.CARTESIAN_TQuat,
        #     ip=ROBOT_IP,
        #     relative_to=RelativeTo.CONFIGURED_ORIGIN,
        #     max_relative_movement=0.1,
        # )
        camera_set = HardwareCameraSet([default_realsense(CAMERA_DICT)]) if CAMERA_DICT is not None else None
        env_rel = RCSFR3EnvCreator()(
            ip=ROBOT_IP,
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
        # xarm7
        # robot_cfg = rcs.sim.SimRobotConfig()
        # robot_cfg.actuators = [
        #     "act1",
        #     "act2",
        #     "act3",
        #     "act4",
        #     "act5",
        #     "act6",
        #     "act7",
        # ]
        # robot_cfg.joints = [
        #     "joint1",
        #     "joint2",
        #     "joint3",
        #     "joint4",
        #     "joint5",
        #     "joint6",
        #     "joint7",
        # ]
        # robot_cfg.base = "base"
        # robot_cfg.robot_type = rcs.common.RobotType.XArm7
        # robot_cfg.attachment_site = "attachment_site"
        # robot_cfg.arm_collision_geoms = []
        # robot_cfg.tcp_offset = rcs.common.Pose()
        # robot_cfg.mjcf_scene_path = rcs.scenes["xarm7_empty_world"].mjb
        # robot_cfg.kinematic_model_path = rcs.scenes["xarm7_empty_world"].mjcf_robot

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
