import typing
import numpy as np
import rcs
from rcs._core import common
from rcs.camera.hw import CalibrationStrategy, HardwareCameraSet
from rcs.envs.utils import default_digit
from rcs_fr3._core import hw
from rcs_realsense.calibration import FR3BaseArucoCalibration
from rcs_realsense.camera import RealSenseCameraSet

CAMERA_DICT = {
    "left_wrist": "230422272017",
    "right_wrist": "230422271040",
    "side": "243522070385",
    "bird_eye": "243522070364",
}
ROBOT_IP = "192.168.101.1"



def default_realsense(name2id: dict[str, str] | None) -> RealSenseCameraSet | None:
    if name2id is None:
        return None
    cameras = {
        name: common.BaseCameraConfig(identifier=id, resolution_width=1280, resolution_height=720, frame_rate=30)
        for name, id in name2id.items()
    }
    calibration_strategy = {
        name: typing.cast(
            CalibrationStrategy, FR3BaseArucoCalibration(name, force=True, expire=3600 * 8, show_live_window=True)
        )
        for name in name2id
    }
    return RealSenseCameraSet(cameras=cameras, calibration_strategy=calibration_strategy)


camera_set = HardwareCameraSet([default_realsense(CAMERA_DICT)])
camera_set.start()
camera_set.wait_for_frames()
camera_set.calibrate()

ik = rcs.common.Pin(
        rcs.scenes["fr3_empty_world"].mjcf_robot,
        "attachment_site_0",
        urdf=False,
    )
robot = hw.Franka(ROBOT_IP, ik)
robot_cfg = hw.FrankaConfig()
# robot_cfg.tcp_offset = rcs.common.Pose(rcs.common.FrankaHandTCPOffset())
robot_cfg.tcp_offset = rcs.common.Pose(pose_matrix=np.array([[0.707, 0.707, 0, 0], [-0.707, 0.707, 0, 0], [0, 0, 1,
    0.15], [0, 0, 0, 1]]))
robot_cfg.ik_solver = hw.IKSolver.rcs_ik
robot_cfg.speed_factor = 0.05
robot.set_config(robot_cfg)  # type: ignore

print(robot.get_cartesian_position())
print(robot.get_joint_position())