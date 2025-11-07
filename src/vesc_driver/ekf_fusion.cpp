// -*- mode:c++; fill-column: 100; -*-

#include "ekf_fusion.h"
#include <cmath>
#include <iostream>

namespace vesc_driver
{

EKFFusion::EKFFusion(float wheelbase)
  : wheelbase_(wheelbase)
{
  // Initialize state to zero
  state_.setZero();
  
  // Initialize covariance matrix with moderate uncertainty
  P_.setIdentity();
  P_(0, 0) = 0.01;  // x position variance
  P_(1, 1) = 0.01;  // y position variance
  P_(2, 2) = 0.01;  // theta variance
  P_(3, 3) = 0.1;   // velocity variance
  P_(4, 4) = 0.1;   // angular velocity variance
  
  // Process noise covariance - tuned for vehicle dynamics
  Q_.setIdentity();
  Q_(0, 0) = 0.001;  // x position process noise
  Q_(1, 1) = 0.001;  // y position process noise
  Q_(2, 2) = 0.005;  // theta process noise
  Q_(3, 3) = 0.1;    // velocity process noise
  Q_(4, 4) = 0.1;    // angular velocity process noise
  
  // IMU measurement noise - gyroscope noise
  R_imu_ = 0.01;  // rad/s variance
}

void EKFFusion::predict(double dt, float rpm, float steering_angle,
                        float speed_to_erpm_gain, float speed_to_erpm_offset)
{
  if (dt <= 0.0 || dt > 1.0) {
    // Skip invalid time steps
    return;
  }
  
  // Convert RPM to linear velocity
  float lin_vel = (rpm - speed_to_erpm_offset) / speed_to_erpm_gain;
  
  // Clamp velocity to zero for minuscule values
  if (std::fabs(lin_vel) < 0.01) {
    lin_vel = 0.0;
  }
  
  // Calculate angular velocity from steering angle
  float rot_vel = 0.0;
  if (std::fabs(steering_angle) > 1e-6) {
    float turn_radius = wheelbase_ / std::tan(steering_angle);
    if (std::fabs(turn_radius) > 1e-6) {
      rot_vel = lin_vel / turn_radius;
    }
  }
  
  // Extract current state
  float x = state_(0);
  float y = state_(1);
  float theta = state_(2);
  
  // Predict new state using odometry motion model
  float cos_theta = std::cos(theta);
  float sin_theta = std::sin(theta);
  
  state_(0) = x + lin_vel * dt * cos_theta;  // x
  state_(1) = y + lin_vel * dt * sin_theta;  // y
  state_(2) = theta + rot_vel * dt;          // theta
  state_(3) = lin_vel;                        // v
  state_(4) = rot_vel;                        // omega
  
  // Normalize theta to [-pi, pi]
  while (state_(2) > M_PI) state_(2) -= 2.0 * M_PI;
  while (state_(2) < -M_PI) state_(2) += 2.0 * M_PI;
  
  // Compute Jacobian of motion model F
  Eigen::Matrix<float, 5, 5> F = Eigen::Matrix<float, 5, 5>::Identity();
  F(0, 2) = -lin_vel * dt * sin_theta;  // dx/dtheta
  F(0, 3) = dt * cos_theta;              // dx/dv
  F(1, 2) = lin_vel * dt * cos_theta;   // dy/dtheta
  F(1, 3) = dt * sin_theta;              // dy/dv
  F(2, 4) = dt;                          // dtheta/domega
  
  // Predict covariance: P = F * P * F^T + Q
  P_ = F * P_ * F.transpose() + Q_;
}

void EKFFusion::updateIMU(float angular_velocity_z, bool available)
{
  if (!available) {
    return;  // No IMU data available
  }
  
  // Measurement model: z = H * x + v
  // We're measuring omega (angular velocity) from IMU
  // H = [0, 0, 0, 0, 1] - selects the omega state
  
  Eigen::Matrix<float, 1, 5> H;
  H << 0, 0, 0, 0, 1;
  
  // Innovation (measurement residual)
  float y = angular_velocity_z - state_(4);
  
  // Innovation covariance
  float S = H * P_ * H.transpose() + R_imu_;
  
  // Kalman gain
  Eigen::Matrix<float, 5, 1> K = P_ * H.transpose() / S;
  
  // Update state estimate
  state_ = state_ + K * y;
  
  // Normalize theta
  while (state_(2) > M_PI) state_(2) -= 2.0 * M_PI;
  while (state_(2) < -M_PI) state_(2) += 2.0 * M_PI;
  
  // Update covariance estimate
  Eigen::Matrix<float, 5, 5> I = Eigen::Matrix<float, 5, 5>::Identity();
  P_ = (I - K * H) * P_;
}

void EKFFusion::getState(float& x, float& y, float& theta, float& v, float& omega) const
{
  x = state_(0);
  y = state_(1);
  theta = state_(2);
  v = state_(3);
  omega = state_(4);
}

void EKFFusion::reset()
{
  state_.setZero();
  P_.setIdentity();
  P_(0, 0) = 0.01;
  P_(1, 1) = 0.01;
  P_(2, 2) = 0.01;
  P_(3, 3) = 0.1;
  P_(4, 4) = 0.1;
}

} // namespace vesc_driver
