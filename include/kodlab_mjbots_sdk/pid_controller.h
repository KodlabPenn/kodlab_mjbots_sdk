/**
 * @file pid_controller.h
 * @author Chandravaran Kunjeti (kunjeti@seas.upenn.edu)
 * @brief Class implementation of PID
 * @date 9/30/2022
 *
 * @copyright 2022 The Trustees of the University of Pennsylvania.
 * All rights reserved.
 *
 */

#pragma once

#include <cmath>
#include <vector>

#define HIGH_SATURATION_INDEX_ 0
#define LOW_SATURATION_INDEX_ 1

namespace kodlab {

  /**
   * @brief PIDController class that allows you to use PID control.
   */
  class PIDController {
  public:
    /**
     * @brief Construct a Pid object
     * @param p_gain Proportional Gain
     * @param d_gain Derivative Gain
     * @param i_gain Integral Gain
     * @param time_step Time difference between 2 updates
     */
    PIDController(double p_gain, double d_gain, double i_gain, double time_step,
      double deadband = 1.0,
      std::vector<double> saturation = { INFINITY, -INFINITY })
      : kp_(p_gain),
      kd_(d_gain),
      ki_(i_gain),
      deadband_(deadband),
      saturation_(saturation) {
      time_step_ = time_step;
    }

    /**
     * @brief Function to set the PID gains
     * @param p_gain Proportional Gain
     * @param d_gain Derivative Gain
     * @param i_gain Integral Gain
     */
    void set_gains(double p_gain, double d_gain, double i_gain) {
      kp_ = p_gain;
      kd_ = d_gain;
      ki_ = i_gain;
    }

    /**
     * @brief Function to set the deadband
     * @param deadband Deadband that needs to be set
     */
    void set_deadband(double deadband) { deadband_ = deadband; }

    /**
     * @brief Function to set the Saturation
     * @param saturation Saturation
     */
    void set_saturation(std::vector<double> saturation) {
      saturation_ = saturation;
    }

    /**
     * @brief Function to update the PID output.
     * @param goal Final goal state for the controller.
     * @param current Current state of the controller.
     * @return Returns the Pid output
     */
    double Update(double goal, double current, double time_step = time_step_) {
      // Calculate the errors
      // Proportional error
      error_ = goal - current;

      // Integral error
      i_error_ += error_ * time_step;

      // Derivative error
      d_error_ = (error_ - d_error_) / time_step;

      // Calling UpdateWithError.
      UpdateWithError(error_, d_error_, i_error_);

      return output_;
    }

    /**
     * @brief Function to update the PID output given the respective errors
     * @param error_p Proportional Error
     * @param error_d Derivative Error
     * @param error_i Integral Error
     * @return Returns the Pid output
     */
    double UpdateWithError(double error_p, double error_d, double error_i) {
      // Finding the control output if errors are inputed
      output_ = kp_ * error_p + kd_ * error_d + ki_ * error_i;

      // Checking the upper limit of the saturation level
      output_ = (output_ > saturation_[HIGH_SATURATION_INDEX_])
        ? saturation_[HIGH_SATURATION_INDEX_]
        : output_;

      // Checking the lower limit of the saturation level
      output_ = (output_ < saturation_[LOW_SATURATION_INDEX_])
        ? saturation_[LOW_SATURATION_INDEX_]
        : output_;
      return output_;
    }

    /**
     * @brief Destructor
     */
    ~PIDController() {}

  protected:
    /**
     * @brief Time difference between 2 updates
     */
    static double time_step_;

    /**
     * @brief Proportional Gain
     */
    double kp_;

    /**
     * @brief Derivative Gain
     */
    double kd_;

    /**
     * @brief Integral Gain
     */
    double ki_;

    /**
     * @brief Stores the previous error for the d term
     */
    double d_error_ = 0;

    /**
     * @brief Sumes up the error for calculating the I term
     */
    double i_error_ = 0;

    /**
     * @brief Error at each step.
     */
    double error_ = 0;

    /**
     * @brief Deadband of the system
     */
    double deadband_;

    /**
     * @brief Max control output
     */
    std::vector<double> saturation_{ INFINITY, -INFINITY };

    /**
     * @brief Output of the function used internally
     */
    double output_ = 0;
  };

}  // namespace kodlab
