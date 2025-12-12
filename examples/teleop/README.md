# Teleoperation with Meta Quest
Teleoperate your robot (optinally dual arm) with the Meta Quest

## How does it work?
In the script [`quest_iris_dual_arm.py`](quest_iris_dual_arm.py) we use the [IRIS platform](https://intuitive-robots.github.io/iris-project-page/index.html) to get controller poses from the meta quest.
With the relative space wrapper and the relative to configured origin setting theses poses are then apply to the robot in a delta fashion whenever the trigger button is pressed.
The buttons are used to start and stop data recording with the [`StorageWrapper`](robot-control-stack/python/rcs/envs/storage_wrapper.py).

## Installation
[Install RCS](https://robotcontrolstack.org/getting_started/index.html) and the [FR3 extension](https://robotcontrolstack.org/extensions/rcs_fr3.html) (the script is writte for the FR3 as example but can be easily adapted for other robots).
Install the IRIS APK on your quest following [these instructions](https://github.com/intuitive-robots/IRIS-Meta-Quest3).
Finally, install [SimPub](https://github.com/intuitive-robots/SimPublisher) the IRIS python client by

```shell
pip install -r requirements.txt
```

