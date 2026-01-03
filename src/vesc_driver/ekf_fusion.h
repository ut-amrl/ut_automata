// -*- mode:c++; fill-column: 100; -*-

#ifndef VESC_DRIVER_EKF_FUSION_H_
#define VESC_DRIVER_EKF_FUSION_H_

// Suppress Eigen warnings about class-memaccess in NEON optimizations
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wclass-memaccess"
#include <Eigen/Dense>
#pragma GCC diagnostic pop

namespace vesc_driver
{

/**
 * @brief Extended Kalman Filter for fusing wheel odometry, commanded velocity, and IMU data
 * 
 * State vector: [x, y, theta, v, omega]
 * - x, y: position in odom frame (m)
 * - theta: orientation (yaw, rad)
 * - v: linear velocity (m/s)
 * - omega: angular velocity (rad/s)
 * 
 * Measurements:
 * - Wheel odometry (delta_x, delta_y from tachometer)
 * - IMU angular velocity
 * - Commanded velocity (for drift prevention)
 */
class EKFFusion
{
public:
  /**
   * @brief Constructor
   * @param wheelbase The wheelbase of the vehicle (m)
   */
  explicit EKFFusion(float wheelbase);

  /**
   * @brief Predict step using commanded velocity model
   * @param dt Time step (seconds)
   * @param commanded_velocity Commanded velocity from controller (m/s)
   * @param steering_angle Current steering angle (radians)
   */
  void predict(double dt, float commanded_velocity, float steering_angle);

  /**
   * @brief Update step using wheel odometry from tachometer
   * @param delta_tach Change in tachometer reading (encoder ticks)
   * @param dt Time step (seconds)
   * @param tach_to_meters Conversion from tachometer ticks to meters
   * @param steering_angle Current steering angle (radians)
   */
  void updateWheelOdometry(double delta_tach, double dt, float tach_to_meters, float steering_angle);

  /**
   * @brief Update step using IMU angular velocity measurement
   * @param angular_velocity_z Angular velocity from IMU (rad/s)
   * @param dt Time step (seconds)
   */
  void updateIMU(float angular_velocity_z, double dt);

  /**
   * @brief Get current state estimate
   * @param x Position x (m)
   * @param y Position y (m)
   * @param theta Orientation (rad)
   * @param v Linear velocity (m/s)
   * @param omega Angular velocity (rad/s)
   */
  void getState(float& x, float& y, float& theta, float& v, float& omega) const;

  /**
   * @brief Reset the filter to initial state
   */
  void reset();

private:
  // State vector [x, y, theta, v, omega]
  Eigen::Matrix<float, 5, 1> state_;
  
  // State covariance matrix
  Eigen::Matrix<float, 5, 5> P_;
  
  // Process noise covariance
  Eigen::Matrix<float, 5, 5> Q_;
  
  // Wheel odometry measurement noise covariance (2x2 for dx, dy)
  Eigen::Matrix<float, 2, 2> R_wheel_;
  
  // IMU measurement noise covariance (for angular velocity)
  float R_imu_;
  
  // Vehicle wheelbase
  float wheelbase_;
};

} // namespace vesc_driver

#endif // VESC_DRIVER_EKF_FUSION_H_
