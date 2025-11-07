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
 * @brief Extended Kalman Filter for fusing odometry with IMU data
 * 
 * State vector: [x, y, theta, v, omega]
 * - x, y: position in odom frame
 * - theta: orientation (yaw)
 * - v: linear velocity
 * - omega: angular velocity
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
   * @brief Predict step using odometry model
   * @param dt Time step (seconds)
   * @param rpm Motor RPM from VESC
   * @param steering_angle Current steering angle (radians)
   * @param speed_to_erpm_gain Conversion gain from speed to ERPM
   * @param speed_to_erpm_offset Conversion offset from speed to ERPM
   */
  void predict(double dt, float rpm, float steering_angle,
               float speed_to_erpm_gain, float speed_to_erpm_offset);

  /**
   * @brief Update step using IMU angular velocity measurement
   * @param angular_velocity_z Angular velocity from IMU (rad/s)
   * @param available Whether IMU data is available
   */
  void updateIMU(float angular_velocity_z, bool available);

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
  
  // IMU measurement noise covariance (for omega)
  float R_imu_;
  
  // Vehicle wheelbase
  float wheelbase_;
};

} // namespace vesc_driver

#endif // VESC_DRIVER_EKF_FUSION_H_
