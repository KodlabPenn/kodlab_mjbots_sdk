/**
 * @file pid.h
 * @author Chandravaran Kunjeti (kunjeti@seas.upenn.edu)
 * @brief Class implementation of PID
 * @date 9/30/2022
 *
 * @copyright 2022 The Trustees of the University of Pennsylvania.
 * All rights reserved.
 *
 */

#pragma once

#ifndef _PID_
#define _PID_

#include "Eigen/Geometry"

namespace kodlab {

/**
 * @brief Pid object
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
                  double deadband, double saturation)
        : kp_(0), kd_(0), ki_(0), time_step_(0.01), deadband_(0), saturation_(0) {
        kp_ = p_gain;
        kd_ = d_gain;
        ki_ = i_gain;
        time_step_ = time_step;
        deadband_ = deadband;
        saturation_ = saturation;
    }

    /**
     * @brief Function to set the PID gains
     * @param p_gain Proportional Gain
     * @param d_gain Derivative Gain
     * @param i_gain Integral Gain
     */
    void SetGains(double p_gain, double d_gain, double i_gain) {
        kp_ = p_gain;
        kd_ = d_gain;
        ki_ = i_gain;
    }

    /**
     * @brief Function to set the deadband
     * @param deadband Deadband that needs to be set
     */
    void SetDeadband(double deadband) {
        deadband_ = deadband;
    }

    /**
     * @brief Function to set the Saturation
     * @param saturation Saturation
     */
    void SetSaturation(double saturation) {
        saturation_ = saturation;
    }

    /**
     * @brief Function to update the PID output.
     * @param goal Final goal state for the controller.
     * @param current Current state of the controller.
     * @return Returns the Pid output
     */
    double Update(double goal, double current) {
        // Calculate the errors
        // Proportional error
        error_ = goal - current;

        // Integral error
        i_error_ += error_ * time_step_;

        // Derivative error
        derivative_ = (error_ - d_error_) / time_step_;

        // Control output
        output_ = kp_ * error_ + kd_ * derivative_ + ki_ * i_error_;

        // Saturation checking
        output_ = (output_ > saturation_) ? saturation_ : output_;

        // Storing the past error
        d_error_ = derivative_;

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
        output_ = (output_ > saturation_) ? saturation_ : output_;
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
    double time_step_;

    /**
     * @brief Proportional Gain
     */
    double kp_;

    /**
     * @brief Derivative Gain
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
     * @brief Current derivative error found
     */
    double derivative_;

    /**
     * @brief Sumes up the error for calculating the I term
     */
    double i_error_;

    /**
     * @brief Error at each step.
     */
    double error_;

    /**
     * @brief Deadband of the system
     */
    double deadband_;

    /**
     * @brief Max control output
     */
    double saturation_;

    /**
     * @brief Output of the function
     */
    double output_;
};

}  // namespace kodlab

#endif