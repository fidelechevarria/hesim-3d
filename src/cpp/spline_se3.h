#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>

namespace hesim3d {

struct TrajectorySample {
  double timestamp_sec{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d linear_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d linear_acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity_body{Eigen::Vector3d::Zero()};
  Eigen::Vector3d imu_acceleration{Eigen::Vector3d::Zero()};
};

struct Keyframe {
  double timestamp_sec{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

class SE3Spline {
 public:
  SE3Spline() = default;
  explicit SE3Spline(double dt_sec);

  void set_control_interval(double dt_sec);
  void set_gravity(const Eigen::Vector3d& g);

  // Add keyframes or control points
  void set_keyframes(const std::vector<Keyframe>& keyframes);
  void build_from_waypoints(const std::vector<Eigen::Vector3d>& positions,
                            const std::vector<Eigen::Quaterniond>& orientations,
                            double total_duration_sec);

  // Evaluation
  TrajectorySample evaluate(double t_sec) const;
  std::vector<TrajectorySample> evaluate_batch(const std::vector<double>& timestamps) const;

  // Presets
  static SE3Spline create_circle_lookat(const Eigen::Vector3d& center, double radius, double height,
                                        double duration_sec, double dt_ctrl = 0.1);

  static SE3Spline create_eight_loop(const Eigen::Vector3d& center, const Eigen::Vector3d& extent,
                                     double duration_sec, double speed_factor = 1.0,
                                     double dt_ctrl = 0.05);

  double min_time() const { return t_min_; }
  double max_time() const { return t_max_; }
  size_t num_control_points() const { return positions_.size(); }

  const std::vector<Eigen::Vector3d>& control_positions() const { return positions_; }
  const std::vector<Eigen::Quaterniond>& control_orientations() const { return orientations_; }

 private:
  double dt_{0.1};
  double t_min_{0.0};
  double t_max_{0.0};
  Eigen::Vector3d gravity_{0.0, 0.0, -9.81};

  std::vector<Eigen::Vector3d> positions_;
  std::vector<Eigen::Quaterniond> orientations_;

  // SO(3) Lie Algebra helper methods
  static Eigen::Vector3d log_so3(const Eigen::Matrix3d& R);
  static Eigen::Matrix3d exp_so3(const Eigen::Vector3d& omega);
  static Eigen::Matrix3d right_jacobian_so3(const Eigen::Vector3d& omega);
};

}  // namespace hesim3d
