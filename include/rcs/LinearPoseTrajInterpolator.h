#ifndef RCS_LINEAR_POSE_TRAJ_INTERPOLATOR_H
#define RCS_LINEAR_POSE_TRAJ_INTERPOLATOR_H
#include <Eigen/Dense>

namespace rcs {
namespace common {

class LinearPoseTrajInterpolator {
 private:
  Eigen::Vector3d p_start_;
  Eigen::Vector3d p_goal_;
  Eigen::Vector3d last_p_t_;

  Eigen::Quaterniond q_start_;
  Eigen::Quaterniond q_goal_;
  Eigen::Quaterniond last_q_t_;

  double dt_;
  double last_time_;
  double max_time_;
  double start_time_;
  bool start_;
  bool initialized_;

 public:
  inline LinearPoseTrajInterpolator()
      : p_start_(Eigen::Vector3d::Zero()),
        p_goal_(Eigen::Vector3d::Zero()),
        last_p_t_(Eigen::Vector3d::Zero()),
        q_start_(Eigen::Quaterniond::Identity()),
        q_goal_(Eigen::Quaterniond::Identity()),
        last_q_t_(Eigen::Quaterniond::Identity()),
        dt_(0.),
        last_time_(0.),
        max_time_(1.),
        start_time_(0.),
        start_(false),
        initialized_(false){};

  inline ~LinearPoseTrajInterpolator(){};

  inline void reset(const double &time_sec, const Eigen::Vector3d &p_start,
                    const Eigen::Quaterniond &q_start,
                    const Eigen::Vector3d &p_goal,
                    const Eigen::Quaterniond &q_goal, const int &policy_rate,
                    const int &rate,
                    const double &traj_interpolator_time_fraction) {
    if (policy_rate > 0) {
      max_time_ = 1. / static_cast<double>(policy_rate) *
                  traj_interpolator_time_fraction;
    }
    dt_ = 1. / static_cast<double>(rate);
    last_time_ = time_sec;
    start_time_ = time_sec;

    start_ = false;

    if (!initialized_) {
      p_start_ = p_start;
      q_start_ = q_start;
      initialized_ = true;
    } else {
      p_start_ = last_p_t_;
      q_start_ = last_q_t_;
    }

    p_goal_ = p_goal;
    q_goal_ = q_goal;

    // Flip the sign if the dot product of quaternions is negative
    if (q_start_.coeffs().dot(q_goal_.coeffs()) < 0.0) {
      q_goal_.coeffs() << -q_goal_.coeffs();
    }
  };

  inline void next_step(const double &time_sec, Eigen::Vector3d &p_t,
                        Eigen::Quaterniond &q_t) {
    if (!start_) {
      start_time_ = time_sec;
      last_p_t_ = p_start_;
      last_q_t_ = q_start_;
      start_ = true;
    }

    if (last_time_ + dt_ <= time_sec) {
      double t =
          std::min(std::max((time_sec - start_time_) / max_time_, 0.), 1.);
      last_p_t_ = p_start_ + t * (p_goal_ - p_start_);
      last_q_t_ = q_start_.slerp(t, q_goal_);
      last_time_ = time_sec;
    }
    p_t = last_p_t_;
    q_t = last_q_t_;
  };
};
class LinearJointPositionTrajInterpolator {
 private:
  using Vector7d = Eigen::Matrix<double, 7, 1>;

  Vector7d q_start_;
  Vector7d q_goal_;
  Vector7d last_q_t_;

  double dt_;
  double last_time_;
  double max_time_;
  double start_time_;
  bool start_;
  bool initialized_;

 public:
  inline LinearJointPositionTrajInterpolator()
      : q_start_(Vector7d::Zero()),
        q_goal_(Vector7d::Zero()),
        last_q_t_(Vector7d::Zero()),
        dt_(0.),
        last_time_(0.),
        max_time_(1.),
        start_time_(0.),
        start_(false),
        initialized_(false){};

  inline ~LinearJointPositionTrajInterpolator(){};

  inline void reset(const double &time_sec,
                    const Eigen::Matrix<double, 7, 1> &q_start,
                    const Eigen::Matrix<double, 7, 1> &q_goal,
                    const int &policy_rate, const int &rate,
                    const double &traj_interpolator_time_fraction) {
    if (policy_rate > 0) {
      max_time_ = 1. / static_cast<double>(policy_rate) *
                  traj_interpolator_time_fraction;
    }
    dt_ = 1. / static_cast<double>(rate);
    last_time_ = time_sec;

    start_time_ = time_sec;

    start_ = false;

    if (!initialized_) {
      q_start_ = q_start;
      initialized_ = true;
    } else {
      q_start_ = last_q_t_;
    }
    q_goal_ = q_goal;
  };

  inline void next_step(const double &time_sec, Vector7d &q_t) {
    if (!start_) {
      start_time_ = time_sec;
      last_q_t_ = q_start_;
      start_ = true;
    }
    // std::cout << q_start_.transpose() << " | " << q_goal_.transpose() <<
    // std::endl;
    if (last_time_ + dt_ <= time_sec) {
      double t =
          std::min(std::max((time_sec - start_time_) / max_time_, 0.), 1.);
      last_q_t_ = q_start_ + t * (q_goal_ - q_start_);
      last_time_ = time_sec;
    }
    q_t = last_q_t_;
  };
};
}  // namespace common
}  // namespace rcs
#endif  // RCS_LINEAR_POSE_TRAJ_INTERPOLATOR_H
