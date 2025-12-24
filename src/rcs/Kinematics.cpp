#include "rcs/Kinematics.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/mjcf.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace rcs {
namespace common {

Pin::Pin(const std::string& path, const std::string& frame_id, bool urdf = true)
    : model() {
  if (urdf) {
    pinocchio::urdf::buildModel(path, this->model);
  } else {
    pinocchio::mjcf::buildModel(path, this->model);
  }
  this->data = pinocchio::Data(this->model);
  this->FRAME_ID = model.getFrameId(frame_id);
  if (FRAME_ID == -1) {
    throw std::runtime_error(
        frame_id + " frame id could not be found in the provided URDF");
  }
}

std::optional<VectorXd> Pin::inverse(const Pose& pose, const VectorXd& q0,
                                     const Pose& tcp_offset
                                    ) 
{
  // TODO: this returns wrong shape
  double null_gain = 1.0;// Stiffness of the pull
  const Vector7d q_home =
    (Vector7d() << 0.0, -M_PI_4, 0.0, -3 * M_PI_4, 0.0, M_PI_2, 0.0).finished();
  rcs::common::Pose new_pose = pose * tcp_offset.inverse();
  VectorXd q(model.nq);
  q.setZero();
  q.head(q0.size()) = q0; // Initialize with current guess

  const pinocchio::SE3 oMdes(new_pose.rotation_m(), new_pose.translation());
  pinocchio::Data::Matrix6x J(6, model.nv);
  J.setZero();

  bool success = false;
  Vector6d err;
  Eigen::VectorXd v(model.nv);
  Eigen::VectorXd v_null_bias(model.nv); 

  // Reusable solver to avoid re-computing Cholesky decomposition
  Eigen::LDLT<pinocchio::Data::Matrix6> solver;

  for (int i = 0;; i++) {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    // 1. Check Primary Task Error (Cartesian)
    const pinocchio::SE3 iMd = data.oMf[this->FRAME_ID].actInv(oMdes);
    err = pinocchio::log6(iMd).toVector();

    if (err.norm() < this->eps) {
      success = true;
      break;
    }
    if (i >= this->IT_MAX) {
      success = false;
      break;
    }

    // 2. Compute Jacobian
    pinocchio::computeFrameJacobian(model, data, q, this->FRAME_ID, J);
    pinocchio::Data::Matrix6 Jlog;
    pinocchio::Jlog6(iMd.inverse(), Jlog);
    J = -Jlog * J;

    // 3. Compute Damped Pseudo-Inverse Components
    pinocchio::Data::Matrix6 JJt;
    JJt.noalias() = J * J.transpose();
    JJt.diagonal().array() += this->damp;
    solver.compute(JJt);

    // 4. Calculate Primary Task Velocity
    // v_primary = J# * err
    v.noalias() = -J.transpose() * solver.solve(err);

    // 5. Calculate Nullspace Bias (The "Home Pose" Pull)
    // We treat the distance to home as a desired velocity vector.
    // v_bias = gain * (q_home - q)
    v_null_bias = null_gain * pinocchio::difference(model, q, q_home);

    // 6. Project Bias into Nullspace
    // Formula: v_total = v_primary + (I - J# J) * v_bias
    // Optimization: v_total = v_primary + v_bias - J# (J * v_bias)
    
    Eigen::VectorXd J_v_null = J * v_null_bias;
    v.noalias() += v_null_bias - (J.transpose() * solver.solve(J_v_null));

    // 7. Integrate
    q = pinocchio::integrate(model, q, v * this->DT);
  }

  if (success) {
    return q;
  } else {
    return std::nullopt;
  }
}

Pose Pin::forward(const VectorXd& q0, const Pose& tcp_offset) {
  // pose is assumed to be in the robots coordinate frame
  VectorXd q(model.nq);
  q.setZero();
  q.head(q0.size()) = q0;
  pinocchio::framesForwardKinematics(model, data, q);
  rcs::common::Pose pose(data.oMf[this->FRAME_ID].rotation(),
                         data.oMf[this->FRAME_ID].translation());

  // apply the tcp offset
  return pose * tcp_offset.inverse();
}

}  // namespace common
}  // namespace rcs