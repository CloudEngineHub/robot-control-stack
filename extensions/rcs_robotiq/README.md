# RCS Robotiq Gripper Hardware Extension
Extension to use the Robotiq 2F-85 Gripper with rcs.

## Installation
```shell
pip install -ve .
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

