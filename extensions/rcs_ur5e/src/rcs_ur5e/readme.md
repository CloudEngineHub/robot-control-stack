# UR5e Extension for Robot Control Stack 🤖
This repository contains a basic Python-based controller plugin for the UR5 robot. It allows for direct manipulation of the UR5.

## Warnings & Safety Notices
- Non-Compliant Operation: This controller does not implement any force-based control (e.g., force mode or impedance control). The robot will not be compliant to external forces. The robot will go into Emergency-Stop when it detects a collision.

- Rough Control: This is a basic joint-level or Cartesian controller implementation. Movements may be slightly rough or non-optimal compared to highly tuned industrial controllers. 

- Home Position Verification: Always check and verify the robot's starting (home) position configured in the plugin or the control stack before starting any program. An incorrect home position could lead to collisions.

- Gradual Velocity Increase: Start all operations using a very slow velocity setting on the pendant or control panel(i.e., 10%). Gradually increase the velocity only after confirming that the basic movements are safe, stable, and correct. Never immediately jump to high speeds.

## Installation
To install hardware extension use
```shell
pip install -ve rcs_ur5e
```

## Usage
```python
import numpy as np
from rcs.envs.base import ControlMode, RelativeTo
from rcs_ur5e.creators import RCSUR5eEnvCreator
from rcs_ur5e.hw import UR5eConfig

ROBOT_IP = "192.168.1.15"
robot_cfg = UR5eConfig()
robot_cfg.async_control = False

env_rel = RCSUR5eEnvCreator()(
    robot_cfg=robot_cfg,
    control_mode=ControlMode.CARTESIAN_TRPY,
    ip=ROBOT_IP,
    camera_set=None,
    max_relative_movement=0.2,
    relative_to=RelativeTo.LAST_STEP,
)

obs, info = env_rel.reset()

act = {"xyzrpy": [0.0, 0, 0.0, 0, 0, np.deg2rad(45)], "gripper": 0}
obs, reward, terminated, truncated, info = env_rel.step(act)
```

For more examples see the [examples](../../examples/) folder.
You can switch to hardware by setting the following flag:
```python
ROBOT_INSTANCE = RobotPlatform.HARDWARE
# ROBOT_INSTANCE = RobotPlatform.SIMULATION
```

You can initially test the connection to the robot and basic movements by running the scripts in the `scripts` folder.




TODO(j.hechtl):
- test usage script
- test examples 
- add usage section to documentation page
- make ready for tools repo merge
- simulation gripper?