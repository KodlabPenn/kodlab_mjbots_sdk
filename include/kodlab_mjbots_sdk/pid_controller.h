/**
 * @file pid.h
 * @author Chandravaran Kunjeti (kunjeti@seas.upenn.edu)
 * @author Raghavesh Viswanath (vrag@seas.upenn.edu)
 * @brief Class implementation of PID
 * @date 5/18/2023
 * @copyright 2023 The Trustees of the University of Pennsylvania.
 * All rights reserved.
 *
 */

#pragma once

#include <math.h>
#include <array>
#include <vector>
#include <algorithm>

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
      std::array<double, 2> saturation = { -INFINITY, INFINITY })
      : kp_(p_gain),
        kd_(d_gain),
        ki_(i_gain),
        deadband_(deadband),
        saturation_(saturation),
        time_step_(time_step) {}

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
    void set_saturation(std::array<double, 2> saturation) {
      saturation_ = saturation;
    }

    /**
     * @brief Function to update the state setpoints.
     * @param setpoint State Target of the controller.
     * @param derivative_setpoint Derivative State Target of the controller.
     */
    void set_setpoint(double setpoint,double derivative_setpoint = 0.){
      setpoint_ = setpoint;
      derivative_setpoint_ = derivative_setpoint;
    }

    /**
     * @brief Function to update setpoints and PID output.
     * @param state Current state for the controller.
     * @param derivative_state Current derivative state of the controller.
     * @param setpoint Desired state of the controller
     * @param derivative_setpoint Desired derivative state of the controller
     * @param time_step   Time difference between 2 updates
     * @return Returns the Pid output
     */
    double Update(double state,double derivative_state,double setpoint,double derivative_setpoint,double time_step) {
      
      set_setpoint(setpoint,derivative_setpoint);
      
      // Calling Nominal Update function.
      return Update(state,derivative_state,time_step);
    }

    /**
     * @brief Function to update setpoints and PID output.
     * @param state Current state for the controller.
     * @param derivative_state Current derivative state of the controller.
     * @param time_step   Time difference between 2 updates
     * @return Returns the Pid output
     */
    double Update(double state,double derivative_state,double time_step) {
      // Calculate the errors
      // Proportional error
      error_ = setpoint_ - state;
     
      // Derivative error
      d_error_ =  derivative_setpoint_ - derivative_state;
      
      // Calling UpdateWithError.
      return UpdateWithError(error_, d_error_,time_step);
    }

    /**
     * @brief Function to update the PID output given the respective errors
     * @param state Current state for the controller.
     * @param derivative_state Current derivative state of the controller.  
     * @return Returns the Pid output
     */
    double Update(double state, double derivative_state) {

      return Update(state,derivative_state,time_step_);
    }

    /**
     * @brief Function to update the PID output given the respective errors
     * @param error_p Proportional Error
     * @param error_d Derivative Error
     * @return Returns the Pid output
     */
    double UpdateWithError(double error_p, double error_d, double time_step) {
      // Computing the integral error
      i_error_ += error_p*time_step; 

      // Finding the control output from the errors
      output_ = kp_ * error_p + kd_ * error_d + ki_ * i_error_;

      output_= std::clamp(output_,saturation_[kSaturationLowerLimitIndex],saturation_[kSaturationUpperLimitIndex]);
      
      return output_;
    }

    /**
     * @brief Destructor
     */
    ~PIDController() {}


  protected:
    
    /**
     * @brief State Setpoint 
     */
    double setpoint_;

    /**
     * @brief State derivative Setpoint
     */
    double derivative_setpoint_;
    /**
     * @brief Time difference between 2 updates
     */
    double time_step_;

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
     * @brief Sums up the error for calculating the I term
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
    std::array<double, 2> saturation_{ -INFINITY, INFINITY };

    /**
     * @brief Output of the function used internally
     */
    double output_ = 0;

  private:
    static constexpr int kSaturationLowerLimitIndex = 0;
    static constexpr int kSaturationUpperLimitIndex = 1;

  };

}  // namespace kodlab
 