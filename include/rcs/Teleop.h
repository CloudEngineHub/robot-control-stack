#ifndef RCS_TELEOP_H
#define RCS_TELEOP_H

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "Pose.h"
#include "utils.h"

namespace rcs {
namespace common {

// Enum for control mode
enum class TeleopControlMode {
  CARTESIAN_TQUAT,
  CARTESIAN_XYZRPY,
  JOINT_POSITIONS
};

// Enum for standardized button actions
enum class LogicalButton {
  RECENTER,
  GRASP,
  PRIMARY_ACTION,
  SECONDARY_ACTION,
  TRIGGER
};

// A struct to hold the state of a single teleoperation device/controller
struct TeleopDeviceState {
  // Cartesian pose (if applicable)
  std::optional<Pose> cartesian_pose;
  // Joint positions (if applicable)
  std::optional<VectorXd> joint_positions;
  // Map of logical button states (0.0 to 1.0)
  std::map<LogicalButton, double> button_states;
  // Optional map of other raw axis states from the device
  std::optional<std::map<std::string, double>> raw_axis_states;
};

class Teleop {
 public:
  virtual ~Teleop() {}

  // Get the state of all teleoperation devices
  virtual std::map<std::string, TeleopDeviceState> get_state() = 0;

  // Get the control mode
  virtual TeleopControlMode get_control_mode() const = 0;
};

}  // namespace common
}  // namespace rcs

#endif  // RCS_TELEOP_H