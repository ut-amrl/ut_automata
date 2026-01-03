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
  
  // Initialize covariance matrix with low uncertainty (starts at known position)
  P_.setIdentity();
  P_(0, 0) = 0.01;  // x position variance (m^2)
  P_(1, 1) = 0.01;  // y position variance (m^2)
  P_(2, 2) = 0.01;  // theta variance (rad^2)
  P_(3, 3) = 0.1;   // velocity variance (m/s)^2
  P_(4, 4) = 0.1;   // angular velocity variance (rad/s)^2
  
  // Process noise covariance - models uncertainty in commanded velocity
  Q_.setIdentity();
  Q_(0, 0) = 0.0005;  // x position process noise
  Q_(1, 1) = 0.0005;  // y position process noise
  Q_(2, 2) = 0.001;   // theta process noise
  Q_(3, 3) = 0.05;    // velocity process noise
  Q_(4, 4) = 0.05;    // angular velocity process noise
  
  // Wheel odometry measurement noise - low noise for tachometer-based odometry
  R_wheel_.setIdentity();
  R_wheel_(0, 0) = 0.001;  // dx measurement noise (m^2)
  R_wheel_(1, 1) = 0.001;  // dy measurement noise (m^2)
  
  // IMU measurement noise - gyroscope noise
  R_imu_ = 0.01;  // (rad/s)^2 variance
}

void EKFFusion::predict(double dt, float commanded_velocity, float steering_angle)
{
  if (dt <= 0.0 || dt > 1.0) {
    // Skip invalid time steps
    return;
  }
  
  // CRITICAL FIX: Force velocity to zero when commanded velocity is near zero
  // This prevents drift when stationary
  if (std::fabs(commanded_velocity) < 0.01) {
    commanded_velocity = 0.0;
  }
  
  // Calculate angular velocity from steering angle using bicycle model
  float angular_velocity = 0.0;
  if (std::fabs(steering_angle) > 1e-6 && std::fabs(commanded_velocity) > 1e-6) {
    float turn_radius = wheelbase_ / std::tan(steering_angle);
    if (std::fabs(turn_radius) > 1e-6) {
      angular_velocity = commanded_velocity / turn_radius;
    }
  }
  
  // Extract current state
  float x = state_(0);
  float y = state_(1);
  float theta = state_(2);
  // v and omega will be set directly from commanded values
  
  // Predict new state using kinematic bicycle model with commanded velocity
  float cos_theta = std::cos(theta);
  float sin_theta = std::sin(theta);
  
  // Use commanded velocity for prediction (drift prevention)
  state_(0) = x + commanded_velocity * dt * cos_theta;  // x
  state_(1) = y + commanded_velocity * dt * sin_theta;  // y
  state_(2) = theta + angular_velocity * dt;             // theta
  state_(3) = commanded_velocity;                        // v (set to commanded)
  state_(4) = angular_velocity;                          // omega (from steering)
  
  // Normalize theta to [-pi, pi]
  while (state_(2) > M_PI) state_(2) -= 2.0 * M_PI;
  while (state_(2) < -M_PI) state_(2) += 2.0 * M_PI;
  
  // Compute Jacobian of motion model F
  // x' = x + v*dt*cos(theta)
  // y' = y + v*dt*sin(theta)
  // theta' = theta + omega*dt
  // v' = v_cmd (direct assignment)
  // omega' = omega_cmd (direct assignment)
  Eigen::Matrix<float, 5, 5> F = Eigen::Matrix<float, 5, 5>::Identity();
  F(0, 2) = -commanded_velocity * dt * sin_theta;  // dx/dtheta
  F(0, 3) = dt * cos_theta;                        // dx/dv
  F(1, 2) = commanded_velocity * dt * cos_theta;   // dy/dtheta
  F(1, 3) = dt * sin_theta;                        // dy/dv
  F(2, 4) = dt;                                    // dtheta/domega
  
  // Predict covariance: P = F * P * F^T + Q
  P_ = F * P_ * F.transpose() + Q_;
}

void EKFFusion::updateWheelOdometry(double delta_tach, double dt, float tach_to_meters, float steering_angle)
{
  if (dt <= 0.0 || dt > 1.0) {
    return;  // Skip invalid time steps
  }
  
  // Convert tachometer delta to linear distance traveled
  float distance = static_cast<float>(delta_tach) * tach_to_meters;
  
  // Clamp to zero for minuscule values (noise filter)
  if (std::fabs(distance) < 0.0001) {
    distance = 0.0;
  }
  
  // Calculate expected delta_x and delta_y in odom frame
  // This assumes the vehicle moves in the direction of its heading
  float theta = state_(2);
  float expected_dx = distance * std::cos(theta);
  float expected_dy = distance * std::sin(theta);
  
  // Measurement vector z = [dx, dy]
  Eigen::Matrix<float, 2, 1> z;
  z << expected_dx, expected_dy;
  
  // Measurement model: we observe the change in position from wheel odometry
  // H is the Jacobian of h with respect to state [x, y, theta, v, omega]
  // The measurement depends on theta (direction of motion)
  Eigen::Matrix<float, 2, 5> H;
  H.setZero();
  H(0, 2) = -distance * std::sin(theta);  // ddx/dtheta
  H(1, 2) = distance * std::cos(theta);   // ddy/dtheta
  
  // Predicted measurement (should be zero since we've already moved in predict step)
  Eigen::Matrix<float, 2, 1> h_x;
  h_x << 0, 0;
  
  // Innovation (measurement residual)
  Eigen::Matrix<float, 2, 1> y = z - h_x;
  
  // Innovation covariance
  Eigen::Matrix<float, 2, 2> S = H * P_ * H.transpose() + R_wheel_;
  
  // Kalman gain
  Eigen::Matrix<float, 5, 2> K = P_ * H.transpose() * S.inverse();
  
  // Update state estimate
  state_ = state_ + K * y;
  
  // Normalize theta
  while (state_(2) > M_PI) state_(2) -= 2.0 * M_PI;
  while (state_(2) < -M_PI) state_(2) += 2.0 * M_PI;
  
  // Update covariance estimate
  Eigen::Matrix<float, 5, 5> I = Eigen::Matrix<float, 5, 5>::Identity();
  P_ = (I - K * H) * P_;
}

void EKFFusion::updateIMU(float angular_velocity_z, double dt)
{
  if (dt <= 0.0 || dt > 1.0) {
    return;  // Skip invalid time steps
  }
  
  // Measurement: angular velocity from IMU
  // H = [0, 0, 0, 0, 1] - selects the omega state
  Eigen::Matrix<float, 1, 5> H;
  H << 0, 0, 0, 0, 1;
  
  // Innovation (measurement residual)
  float innovation = angular_velocity_z - state_(4);
  
  // Innovation covariance
  float S = H * P_ * H.transpose() + R_imu_;
  
  // Kalman gain
  Eigen::Matrix<float, 5, 1> K = P_ * H.transpose() / S;
  
  // Update state estimate
  state_ = state_ + K * innovation;
  
  // Normalize theta
  while (state_(2) > M_PI) state_(2) -= 2.0 * M_PI;
  while (state_(2) < -M_PI) state_(2) += 2.0 * M_PI;
  
  // Update covariance estimate
  Eigen::Matrix<float, 5, 5> I = Eigen::Matrix<float, 5, 5>::Identity();
  P_ = (I - K * H) * P_;
  
  // Also integrate IMU measurement to update theta
  state_(2) = state_(2) + angular_velocity_z * dt;
  
  // Normalize theta again
  while (state_(2) > M_PI) state_(2) -= 2.0 * M_PI;
  while (state_(2) < -M_PI) state_(2) += 2.0 * M_PI;
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
