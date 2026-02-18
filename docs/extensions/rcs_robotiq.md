# RCS Robotiq Extension

This extension provides support for Robotiq 2F-85 Gripper in RCS.

## Installation

```shell
pip install -ve extensions/rcs_robotiq
```

Get the serial number of the gripper with this command:
```shell
udevadm info -a -n /dev/ttyUSB0 | grep serial
```

Provide the necessary permission:
```shell
chmod 777 /dev/ttyUSB0
```

## Usage
```python
from rcs_robotiq import RobotiQGripper

gripper = RobotiQGripper('<YOUR_SERIAL_NUMBER>')
gripper.reset()
gripper.shut()
print(gripper.get_normalized_width())
```
