#include "spline_se3.h"
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <numbers>

namespace hesim3d {

SE3Spline::SE3Spline(double dt_sec) : dt_(dt_sec) {}

void SE3Spline::set_control_interval(double dt_sec) {
    if (dt_sec <= 0.0) {
        throw std::invalid_argument("Control interval dt must be positive.");
    }
    dt_ = dt_sec;
}

void SE3Spline::set_gravity(const Eigen::Vector3d& g) {
    gravity_ = g;
}

void SE3Spline::set_keyframes(const std::vector<Keyframe>& keyframes) {
    if (keyframes.size() < 4) {
        throw std::invalid_argument("SE3Spline requires at least 4 keyframes.");
    }
    positions_.clear();
    orientations_.clear();
    positions_.reserve(keyframes.size());
    orientations_.reserve(keyframes.size());

    for (const auto& kf : keyframes) {
        positions_.push_back(kf.position);
        orientations_.push_back(kf.orientation.normalized());
    }

    t_min_ = 0.0;
    t_max_ = (positions_.size() - 3) * dt_;
}

void SE3Spline::build_from_waypoints(const std::vector<Eigen::Vector3d>& positions,
                                     const std::vector<Eigen::Quaterniond>& orientations,
                                     double total_duration_sec) {
    if (positions.size() < 2 || positions.size() != orientations.size()) {
        throw std::invalid_argument("Waypoints must have at least 2 matching positions and orientations.");
    }

    // Pad with boundary points if fewer than 4 points
    positions_.clear();
    orientations_.clear();

    positions_.push_back(positions.front());
    orientations_.push_back(orientations.front().normalized());

    for (size_t i = 0; i < positions.size(); ++i) {
        positions_.push_back(positions[i]);
        orientations_.push_back(orientations[i].normalized());
    }

    positions_.push_back(positions.back());
    orientations_.push_back(orientations.back().normalized());

    size_t num_segments = positions_.size() - 3;
    dt_ = total_duration_sec / static_cast<double>(num_segments);
    t_min_ = 0.0;
    t_max_ = total_duration_sec;
}

Eigen::Vector3d SE3Spline::log_so3(const Eigen::Matrix3d& R) {
    Eigen::AngleAxisd aa(R);
    return aa.axis() * aa.angle();
}

Eigen::Matrix3d SE3Spline::exp_so3(const Eigen::Vector3d& omega) {
    double theta = omega.norm();
    if (theta < 1e-10) {
        return Eigen::Matrix3d::Identity();
    }
    return Eigen::AngleAxisd(theta, omega / theta).toRotationMatrix();
}

Eigen::Matrix3d SE3Spline::right_jacobian_so3(const Eigen::Vector3d& omega) {
    double theta = omega.norm();
    if (theta < 1e-6) {
        return Eigen::Matrix3d::Identity() - 0.5 * (Eigen::Matrix3d() << 0, -omega.z(), omega.y(),
                                                                          omega.z(), 0, -omega.x(),
                                                                         -omega.y(), omega.x(), 0).finished();
    }
    Eigen::Vector3d a = omega / theta;
    Eigen::Matrix3d a_hat;
    a_hat << 0, -a.z(), a.y(),
             a.z(), 0, -a.x(),
            -a.y(), a.x(), 0;

    double s = std::sin(theta);
    double c = std::cos(theta);

    return (s / theta) * Eigen::Matrix3d::Identity() +
           (1.0 - s / theta) * (a * a.transpose()) -
           ((1.0 - c) / theta) * a_hat;
}

TrajectorySample SE3Spline::evaluate(double t_sec) const {
    TrajectorySample sample;
    sample.timestamp_sec = t_sec;
    sample.position.setZero();
    sample.orientation.setIdentity();
    sample.linear_velocity.setZero();
    sample.linear_acceleration.setZero();
    sample.angular_velocity_body.setZero();
    sample.imu_acceleration = -gravity_;

    if (positions_.size() < 4) {
        return sample;
    }

    double t_clamped = std::clamp(t_sec, t_min_, t_max_ - 1e-9);
    double seg_f = t_clamped / dt_;
    size_t i = static_cast<size_t>(std::floor(seg_f));
    if (i + 3 >= positions_.size()) {
        i = positions_.size() - 4;
    }
    double u = (t_clamped - i * dt_) / dt_;
    double u2 = u * u;
    double u3 = u2 * u;

    // Standard cubic B-spline blending matrix M / 6
    // B(u) = 1/6 * [ 1 - 3u + 3u^2 - u^3,  4 - 6u^2 + 3u^3,  1 + 3u + 3u^2 - 3u^3,  u^3 ]
    double b0 = (1.0 - 3.0 * u + 3.0 * u2 - u3) / 6.0;
    double b1 = (4.0 - 6.0 * u2 + 3.0 * u3) / 6.0;
    double b2 = (1.0 + 3.0 * u + 3.0 * u2 - 3.0 * u3) / 6.0;
    double b3 = u3 / 6.0;

    // First derivatives wrt u
    double db0 = (-3.0 + 6.0 * u - 3.0 * u2) / (6.0 * dt_);
    double db1 = (-12.0 * u + 9.0 * u2) / (6.0 * dt_);
    double db2 = (3.0 + 6.0 * u - 9.0 * u2) / (6.0 * dt_);
    double db3 = (3.0 * u2) / (6.0 * dt_);

    // Second derivatives wrt u
    double d2b0 = (6.0 - 6.0 * u) / (6.0 * dt_ * dt_);
    double d2b1 = (-12.0 + 18.0 * u) / (6.0 * dt_ * dt_);
    double d2b2 = (6.0 - 18.0 * u) / (6.0 * dt_ * dt_);
    double d2b3 = (6.0 * u) / (6.0 * dt_ * dt_);

    // Translation and derivatives
    sample.position = b0 * positions_[i] + b1 * positions_[i + 1] + b2 * positions_[i + 2] + b3 * positions_[i + 3];
    sample.linear_velocity = db0 * positions_[i] + db1 * positions_[i + 1] + db2 * positions_[i + 2] + db3 * positions_[i + 3];
    sample.linear_acceleration = d2b0 * positions_[i] + d2b1 * positions_[i + 1] + d2b2 * positions_[i + 2] + d2b3 * positions_[i + 3];

    // Cumulative basis functions for SO(3)
    double beta1 = b1 + b2 + b3;
    double beta2 = b2 + b3;
    double beta3 = b3;

    double dbeta1 = db1 + db2 + db3;
    double dbeta2 = db2 + db3;
    double dbeta3 = db3;

    Eigen::Matrix3d R0 = orientations_[i].toRotationMatrix();
    Eigen::Matrix3d R1 = orientations_[i + 1].toRotationMatrix();
    Eigen::Matrix3d R2 = orientations_[i + 2].toRotationMatrix();
    Eigen::Matrix3d R3 = orientations_[i + 3].toRotationMatrix();

    Eigen::Vector3d omega1 = log_so3(R0.transpose() * R1);
    Eigen::Vector3d omega2 = log_so3(R1.transpose() * R2);
    Eigen::Vector3d omega3 = log_so3(R2.transpose() * R3);

    Eigen::Matrix3d exp1 = exp_so3(beta1 * omega1);
    Eigen::Matrix3d exp2 = exp_so3(beta2 * omega2);
    Eigen::Matrix3d exp3 = exp_so3(beta3 * omega3);

    Eigen::Matrix3d R_wb = R0 * exp1 * exp2 * exp3;
    sample.orientation = Eigen::Quaterniond(R_wb).normalized();

    // Body angular velocity omega_b = R_wb^T * dR_wb/dt
    Eigen::Vector3d w1 = dbeta1 * omega1;
    Eigen::Vector3d w2 = dbeta2 * omega2;
    Eigen::Vector3d w3 = dbeta3 * omega3;

    Eigen::Vector3d omega_body = exp3.transpose() * exp2.transpose() * w1 +
                                 exp3.transpose() * w2 +
                                 w3;
    sample.angular_velocity_body = omega_body;

    // Ground-truth IMU measurement (specific force in body frame)
    sample.imu_acceleration = sample.orientation.conjugate() * (sample.linear_acceleration - gravity_);

    return sample;
}

std::vector<TrajectorySample> SE3Spline::evaluate_batch(const std::vector<double>& timestamps) const {
    std::vector<TrajectorySample> samples;
    samples.reserve(timestamps.size());
    for (double t : timestamps) {
        samples.push_back(evaluate(t));
    }
    return samples;
}

SE3Spline SE3Spline::create_circle_lookat(const Eigen::Vector3d& center,
                                          double radius,
                                          double height,
                                          double duration_sec,
                                          double dt_ctrl) {
    SE3Spline spline(dt_ctrl);
    int num_pts = static_cast<int>(std::ceil(duration_sec / dt_ctrl)) + 4;
    std::vector<Eigen::Vector3d> pos;
    std::vector<Eigen::Quaterniond> ori;
    pos.reserve(num_pts);
    ori.reserve(num_pts);

    for (int i = 0; i < num_pts; ++i) {
        double t = (i - 1) * dt_ctrl;
        double angle = (2.0 * std::numbers::pi * t) / duration_sec;
        double x = center.x() + radius * std::cos(angle);
        double y = center.y() + radius * std::sin(angle);
        double z = center.z() + height;

        Eigen::Vector3d p(x, y, z);
        pos.push_back(p);

        // Look at center
        Eigen::Vector3d forward = (center - p).normalized();
        Eigen::Vector3d up(0.0, 0.0, 1.0);
        Eigen::Vector3d right = forward.cross(up).normalized();
        Eigen::Vector3d actual_up = right.cross(forward).normalized();

        Eigen::Matrix3d R;
        R.col(0) = right;
        R.col(1) = actual_up;
        R.col(2) = -forward; // Camera looks down -Z in standard OpenGL/Filament convention

        ori.push_back(Eigen::Quaterniond(R).normalized());
    }

    spline.positions_ = pos;
    spline.orientations_ = ori;
    spline.t_min_ = 0.0;
    spline.t_max_ = duration_sec;
    return spline;
}

SE3Spline SE3Spline::create_eight_loop(const Eigen::Vector3d& center,
                                       const Eigen::Vector3d& extent,
                                       double duration_sec,
                                       double speed_factor,
                                       double dt_ctrl) {
    double eff_duration = duration_sec / speed_factor;
    SE3Spline spline(dt_ctrl);
    int num_pts = static_cast<int>(std::ceil(eff_duration / dt_ctrl)) + 4;
    std::vector<Eigen::Vector3d> pos;
    std::vector<Eigen::Quaterniond> ori;
    pos.reserve(num_pts);
    ori.reserve(num_pts);

    for (int i = 0; i < num_pts; ++i) {
        double t = (i - 1) * dt_ctrl;
        double tau = (2.0 * std::numbers::pi * t) / eff_duration;

        double x = center.x() + extent.x() * std::sin(tau);
        double y = center.y() + extent.y() * std::sin(tau) * std::cos(tau);
        double z = center.z() + extent.z() * std::sin(2.0 * tau);

        Eigen::Vector3d p(x, y, z);
        pos.push_back(p);

        // Compute velocity for orientation direction
        double dx = extent.x() * std::cos(tau);
        double dy = extent.y() * (std::cos(tau) * std::cos(tau) - std::sin(tau) * std::sin(tau));
        double dz = 2.0 * extent.z() * std::cos(2.0 * tau);
        Eigen::Vector3d vel(dx, dy, dz);
        Eigen::Vector3d forward = vel.normalized();

        Eigen::Vector3d up(0.0, 0.0, 1.0);
        if (std::abs(forward.dot(up)) > 0.95) {
            up = Eigen::Vector3d(0.0, 1.0, 0.0);
        }
        Eigen::Vector3d right = forward.cross(up).normalized();
        Eigen::Vector3d actual_up = right.cross(forward).normalized();

        Eigen::Matrix3d R;
        R.col(0) = right;
        R.col(1) = actual_up;
        R.col(2) = -forward;

        ori.push_back(Eigen::Quaterniond(R).normalized());
    }

    spline.positions_ = pos;
    spline.orientations_ = ori;
    spline.t_min_ = 0.0;
    spline.t_max_ = eff_duration;
    return spline;
}

} // namespace hesim3d
