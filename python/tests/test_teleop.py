import pytest
from rcs._core.common import (
    Teleop,
    TeleopControlMode,
    TeleopDeviceState,
    Pose,
    LogicalButton,
)


class PyTeleop(Teleop):
    def get_state(self):
        # Return some dummy state
        state = TeleopDeviceState()
        state.cartesian_pose = Pose()
        state.button_states = {
            LogicalButton.GRASP: 0.8,
            LogicalButton.TRIGGER: 1.0,
        }
        state.raw_axis_states = {"some_axis": -0.5}
        return {"left": state}

    def get_control_mode(self):
        return TeleopControlMode.CARTESIAN_TQUAT


class TestTeleop:
    def test_teleop_implementation(self):
        """
        Tests if a Python class can inherit from the C++ Teleop interface
        and if its methods can be called.
        """
        try:
            teleop_device = PyTeleop()
            state_map = teleop_device.get_state()
            control_mode = teleop_device.get_control_mode()

            assert "left" in state_map
            state = state_map["left"]
            assert isinstance(state, TeleopDeviceState)
            assert state.button_states[LogicalButton.GRASP] == 0.8
            assert state.button_states[LogicalButton.TRIGGER] == 1.0
            assert state.raw_axis_states["some_axis"] == -0.5
            assert control_mode == TeleopControlMode.CARTESIAN_TQUAT

        except Exception as e:
            pytest.fail(f"Teleop interface test failed: {e}")