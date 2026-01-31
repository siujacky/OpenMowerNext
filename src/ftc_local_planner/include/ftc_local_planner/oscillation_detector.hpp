// Copyright (c) 2016 TU Dortmund - Institute of Control Theory and Systems Engineering.
// All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
// ROS2 port of oscillation detector

#ifndef FTC_LOCAL_PLANNER__OSCILLATION_DETECTOR_HPP_
#define FTC_LOCAL_PLANNER__OSCILLATION_DETECTOR_HPP_

#include <deque>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace ftc_local_planner
{

/**
 * @class FailureDetector
 * @brief Detects if the robot got stuck or is oscillating
 *
 * Analyzes the last N commanded velocities to detect oscillation patterns.
 */
class FailureDetector
{
public:
  FailureDetector() = default;
  ~FailureDetector() = default;

  /**
   * @brief Set buffer length (measurement history)
   * @param length number of measurements to be kept
   */
  void setBufferLength(size_t length)
  {
    buffer_capacity_ = length;
    while (buffer_.size() > buffer_capacity_)
    {
      buffer_.pop_front();
    }
  }

  /**
   * @brief Add a new twist measurement to the internal buffer and compute a new decision
   * @param twist velocity information
   * @param v_max maximum forward translational velocity
   * @param v_backwards_max maximum backward translational velocity
   * @param omega_max maximum angular velocity
   * @param v_eps Threshold for the average normalized linear velocity in (0,1)
   * @param omega_eps Threshold for the average normalized angular velocity in (0,1)
   */
  void update(const geometry_msgs::msg::TwistStamped& twist, double v_max, double v_backwards_max, double omega_max,
              double v_eps, double omega_eps)
  {
    if (buffer_capacity_ == 0)
      return;

    VelMeasurement measurement;
    measurement.v = twist.twist.linear.x;
    measurement.omega = twist.twist.angular.z;

    if (measurement.v > 0 && v_max > 0)
      measurement.v /= v_max;
    else if (measurement.v < 0 && v_backwards_max > 0)
      measurement.v /= v_backwards_max;

    if (omega_max > 0)
      measurement.omega /= omega_max;

    buffer_.push_back(measurement);
    while (buffer_.size() > buffer_capacity_)
    {
      buffer_.pop_front();
    }

    detect(v_eps, omega_eps);
  }

  /**
   * @brief Check if the robot got stuck
   * @return true if the robot got stuck, false otherwise
   */
  bool isOscillating() const
  {
    return oscillating_;
  }

  /**
   * @brief Clear the current internal state
   */
  void clear()
  {
    buffer_.clear();
    oscillating_ = false;
  }

protected:
  struct VelMeasurement
  {
    double v = 0;
    double omega = 0;
  };

  bool detect(double v_eps, double omega_eps)
  {
    oscillating_ = false;

    if (buffer_.size() < buffer_capacity_ / 2)
      return false;

    double n = static_cast<double>(buffer_.size());

    double v_mean = 0;
    double omega_mean = 0;
    int omega_zero_crossings = 0;

    for (size_t i = 0; i < buffer_.size(); ++i)
    {
      v_mean += buffer_[i].v;
      omega_mean += buffer_[i].omega;
      if (i > 0 && sign(buffer_[i].omega) != sign(buffer_[i - 1].omega))
        ++omega_zero_crossings;
    }
    v_mean /= n;
    omega_mean /= n;

    if (std::abs(v_mean) < v_eps && std::abs(omega_mean) < omega_eps && omega_zero_crossings > 1)
    {
      oscillating_ = true;
    }

    return oscillating_;
  }

private:
  static int sign(double x)
  {
    if (x > 0)
      return 1;
    else if (x < 0)
      return -1;
    else
      return 0;
  }

  std::deque<VelMeasurement> buffer_;
  size_t buffer_capacity_ = 0;
  bool oscillating_ = false;
};

}  // namespace ftc_local_planner

#endif  // FTC_LOCAL_PLANNER__OSCILLATION_DETECTOR_HPP_
