# Robot Control Stack

**Robot Control Stack (RCS)** is a flexible Gymnasium wrapper-based robot control interface made for robot learning and specifically Vision-Language-Action (VLA) models.
It unifies MuJoCo simulation and real world robot control with four supported robots: FR3/Panda, xArm7, UR5e and SO101.
It ships with several pre-build apps including data collection via teleoperation and remote model inference via [vlagents](https://github.com/RobotControlStack/vlagents).


<video 
  src="https://github.com/user-attachments/assets/21ac29af-373b-46aa-8a08-ae0ae8c0e235" 
  autoplay 
  muted 
  loop 
  playsinline
  style="max-width: 100%;">
</video>

## Wrapper-Based Architecture

<img src="docs/_static/rcs_architecture_small.svg" alt="rcs architecture diagram" width="100%">

## Example
```python
from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.creators import SimEnvCreator
from rcs.envs.utils import (
    default_mujoco_cameraset_cfg,
    default_sim_gripper_cfg,
    default_sim_robot_cfg,
)
env_rel = SimEnvCreator()(
    control_mode=ControlMode.CARTESIAN_TQuat,  # (trans(x, y, z), quat(x, y, z, w))
    robot_cfg=default_sim_robot_cfg(scene="fr3_empty_world"),
    gripper_cfg=default_sim_gripper_cfg(),
    cameras=default_mujoco_cameraset_cfg(),
    max_relative_movement=0.1,  # max 10cm in each direction
    relative_to=RelativeTo.LAST_STEP,
)
env_rel.get_wrapper_attr("sim").open_gui()
env_rel.reset()

# access low level robot api to get current cartesian position
print(env_rel.unwrapped.robot.get_cartesian_position())

for _ in range(10):
    # move 1cm in x direction (forward) and close gripper
    act = {"tquat": [0.01, 0, 0, 0, 0, 0, 1], "gripper": 0}
    obs, reward, terminated, truncated, info = env_rel.step(act)
    print(obs)
```
See more examples in the [examples](examples/) folder.

## Installation

We build and test RCS on the latest Debian and on the latest Ubuntu LTS.

1.  **System Dependencies**:
    ```shell
    sudo apt install $(cat debian_deps.txt)
    ```

2.  **Python Environment**:
    ```shell
    conda create -n rcs python=3.11
    conda activate rcs
    pip install -r requirements.txt
    ```

3.  **Install RCS**:
    ```shell
    pip install -ve . --no-build-isolation
    ```

## Hardware Extensions

RCS supports various hardware extensions (e.g., FR3, xArm7, RealSense). These are located in the `extensions` directory.

To install an extension:

```shell
pip install -ve extensions/rcs_fr3 --no-build-isolation
```

For a full list of extensions and detailed documentation, visit [robotcontrolstack.org/extensions](https://robotcontrolstack.org/extensions).

## Documentation


For full documentation, including installation, usage, and API reference, please visit:

**[robotcontrolstack.org](https://robotcontrolstack.org)**

## Citation

If you find RCS useful for your academic work, please consider citing it:

```bibtex
@misc{juelg2025robotcontrolstack,
  title={{Robot Control Stack}: {A} Lean Ecosystem for Robot Learning at Scale}, 
  author={Tobias J{\"u}lg and Pierre Krack and Seongjin Bien and Yannik Blei and Khaled Gamal and Ken Nakahara and Johannes Hechtl and Roberto Calandra and Wolfram Burgard and Florian Walter},
  year={2025},
  howpublished = {\url{https://arxiv.org/abs/2509.14932}}
}
```

For more scientific info, visit the [paper website](https://robotcontrolstack.github.io/).
