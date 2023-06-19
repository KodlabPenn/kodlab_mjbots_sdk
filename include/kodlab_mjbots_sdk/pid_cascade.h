/**
 * @file pid_cascade.h
 * @author Raghavesh Viswanath (vrag@seas.upenn.edu)
 * @author J. Diego Caporale (jdcap@seas.upenn.edu)
 * @brief Class implementation of PID with a cascade architecture
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
#include <Eigen/Dense>
#include <tuple>

#include "pid_controller.h"

namespace kodlab {

/**
 * @brief Implements a type-agnostic PID controller with a cascade architecture
 * @details Sets up a scalar/Eigen::Vector based PID controller
 * that is type agnostic and has deadband, windup(accumulator) limit,
 * saturation/control-effort limit capabilities. The class template is meant
 * to be implicitly instantiated with Scalar and N deduced based on the
 * argument of the constructor.
 * @tparam Scalar data type for the controller (float, double, etc.)
 * @tparam  N[optional] dimension of the controller
 * @note Class encapsulates a PID cascade controller given gains 
 * \f$ k_p,k_i,k_d \f$ \n 
 * \f$ output \mathrel{=} k_d\cdot v_{error} \mathrel{+} k_i\cdot i_{error}\f$
 * @note  where : \f$ p_{error}\mathrel{=} setpoint \mathrel{-} state,\\ v_{cmd}
 * \mathrel{=} k_p\cdot p_{error} + derivative\_setpoint ,\; 
 * \\v_{error} \mathrel{=} v_{cmd} - derivative\_state ,\\ i_{error} 
 * \mathrel{{+}{=}} v_{error}\cdot time\_step \f$
 * @note Deadband limits are on  \f$ v_{error},\f$ \n \f$ if(\:deadband_{min}\:
 * \leq\:v_{error}\:\leq deadband_{max}\:): \; v_{error} \mathrel{=} 0\f$
 * @note Accumulator limits are applied on the integral term's effort, 
 * \f$ k_i * i_{error} \f$, clamping it
 * @note Saturation (control effort) limits are applied on the output effort,
 * \f$ output \f$, clamping it
 */
template <typename Scalar, int N=1>
class PIDCascade: public PIDController<Scalar, N> {

public:

  typedef Eigen::Matrix<Scalar,N,1> VectorNS;
  using PIDController<Scalar,N>::Update;
  
  /**
   * @brief Construct a vector cascade PID object
   * @param p_gain Proportional Gain
   * @param i_gain Derivative Gain
   * @param d_gain Integral Gain
   * @param time_step Timestep between updates in seconds [Default: 1ms]
   * @param deadband Range within setpoints where no correction 
   * input is applied [Default: {0, 0} (disabled)]
   * @param accumulator_limits Prevents runaway integrator/windup
   * [Default: {-inf, inf} (disabled)]
   * @param saturation_limits Controller output limits
   * [Default: {-inf, inf} (disabled)]
   * @param velocity_limits Velocity command limits
   * [Default: {-inf, inf} (disabled)]
   * \overload
   */
  PIDCascade( const VectorNS & p_gain,
              const VectorNS & i_gain,
              const VectorNS & d_gain, 
              double time_step = 0.001,
              math::Range<VectorNS> deadband = { VectorNS::Zero(), 
                                                 VectorNS::Zero() }, 
              math::Range<VectorNS> accumulator_limit = 
                  {-std::numeric_limits<Scalar>::max() * VectorNS::Ones(), 
                    std::numeric_limits<Scalar>::max() * VectorNS::Ones()},
              math::Range<VectorNS> saturation = 
                  {-std::numeric_limits<Scalar>::max() * VectorNS::Ones(), 
                    std::numeric_limits<Scalar>::max() * VectorNS::Ones()})
              
    : 
      PIDController<Scalar, N>(p_gain, i_gain, d_gain, time_step, deadband,
                              accumulator_limit, saturation){
    
    max_vel_ = std::numeric_limits<Scalar>::max()*VectorNS::Ones();
    }

  /**
   * @brief Construct a scalar cascade PID object
   * @param p_gain Proportional Gain
   * @param i_gain Derivative Gain
   * @param d_gain Integral Gain
   * @param time_step Timestep between updates in seconds [Default: 1ms]
   * @param deadband Range within setpoints where no correction 
   * input is applied [Default: {0, 0} (disabled)]
   * @param accumulator_limits Prevents runaway integrator/windup
   * [Default: {-inf, inf} (disabled)]
   * @param saturation_limits Controller output limits
   * [Default: {-inf, inf} (disabled)]
   * @param velocity_limits Velocity command limits
   * [Default: {-inf, inf} (disabled)]
   * \overload
   */
  PIDCascade( Scalar p_gain,
              Scalar i_gain,
              Scalar d_gain, 
              double time_step = 0.001,
              math::Range<VectorNS> deadband = { VectorNS::Zero(), 
                                                 VectorNS::Zero() }, 
              math::Range<VectorNS> accumulator_limit = 
                  {-std::numeric_limits<Scalar>::max() * VectorNS::Ones(), 
                    std::numeric_limits<Scalar>::max() * VectorNS::Ones()},
              math::Range<VectorNS> saturation = 
                  {-std::numeric_limits<Scalar>::max() * VectorNS::Ones(), 
                    std::numeric_limits<Scalar>::max() * VectorNS::Ones()})
    : 
      PIDController<Scalar, N>(p_gain, i_gain, d_gain, time_step, deadband,
                               accumulator_limit, saturation){
    
    max_vel_ = std::numeric_limits<Scalar>::max()*VectorNS::Ones();
    }

  /**
   * @brief Function to set maximum velocity magnitudes
   * @param max_vel velocity limits for the controller
   * \overload
   */
  void set_max_velocity(VectorNS max_vel) { max_vel_ = max_vel;}

  /**
   * @brief Function to set maximum velocity
   * @param max_vel velocity limit for the controller.
   * \overload
   */
  void set_max_velocity(Scalar max_vel) { max_vel_ = max_vel*VectorNS::Ones();}

  /**
   * @brief Function to get maximum velocity
   * @return max_vel velocity limit for the controller.
   */
  VectorNS get_max_velocity() { return max_vel_;}
  
  /**
   * @brief Function to update setpoints and cascade scalar PID output.
   * @param state Current state for the controller.
   * @param derivative_state Current derivative state of the controller.
   * @return Returns the Pid output
   * \overload
   */
  Scalar Update(Scalar state,Scalar derivative_state) {
    //checking if using for scalar PID
    static_assert(N==1, "Using Scalar function in vector PID");

    // converts scalar to vector form to calculate output
    return Update(state*VectorNS::Ones(), derivative_state*VectorNS::Ones())(0);
  }

  /**
   * @brief Function to update setpoints and cascade vector PID output.
   * @param state Current state for the controller.
   * @param derivative_state Current derivative state of the controller.
   * @return Returns the Pid output
   * \overload
   */
  VectorNS Update(const VectorNS & state, const VectorNS & derivative_state) {

    return Update(state, derivative_state, time_step_);
  }  

  /**
   * @brief Function to update setpoints and cascade PID output.
   * @param state Current state for the controller.
   * @param derivative_state Current derivative state of the controller.
   * @param time_step   Time difference between 2 updates
   * @return Returns the Pid output
   * \overload
   */
  VectorNS Update (const VectorNS & state,
                   const VectorNS & derivative_state,
                   double time_step) override  {
    
    // Proportional error
    error_ = setpoint_ - state;
    
    // V_cmd with max velocity limits
    v_cmd_ = kp_.array() *error_.array() + derivative_setpoint_.array();
    v_cmd_ = v_cmd_.cwiseMin(max_vel_).cwiseMax(-max_vel_);
    
    // Velocity error
    v_error_ = v_cmd_ - derivative_state;
    
    // Calling UpdateWithError.   
    return UpdateWithError(error_, v_error_, time_step);
  }

  /**
   * @brief Function to update the cascade PID output given the respective 
   * errors
   * @param error_p Proportional Error (unused)
   * @param error_d Derivative Error
   * @return Returns the PID output
   * \fn
   */
  VectorNS UpdateWithError( VectorNS & error_p, 
                            VectorNS & error_v, 
                           double time_step) override  {
    
    //checking if within deadband range
    for (int i=0; i < N; i++){
      if ( error_v.coeff(i) < deadband_.max().coeff(i) && 
           deadband_.min().coeff(i) < error_v.coeff(i)) {
        //implementing deadband control logic
        error_v(i) = 0;  
      }
      //if ki is 0, stop accumulating
      if (ki_(i) == 0) i_error_(i) = 0;

      //else keep accumulating
      else{
        i_error_(i) += error_v(i)*time_step; 
        //adding a limit on integral error to prevent windup
        i_error_ = i_error_.array().
                    min(accumulator_limit_.max().array()/ ki_.array()).
                    max(accumulator_limit_.min().array()/ ki_.array());
      }
    }

    //implementing cascade control logic
    output_ = kd_.array() * error_v.array() + ki_.array() * i_error_.array();

    output_ = output_.cwiseMin(saturation_.max()).cwiseMax(saturation_.min());
    return output_;
  }

protected:
  
  using PIDController<Scalar,N>::kp_;
  using PIDController<Scalar,N>::ki_;
  using PIDController<Scalar,N>::kd_;
  using PIDController<Scalar,N>::error_;
  using PIDController<Scalar,N>::i_error_;
  using PIDController<Scalar,N>::time_step_;
  using PIDController<Scalar,N>::deadband_;
  using PIDController<Scalar,N>::output_;
  using PIDController<Scalar,N>::accumulator_limit_;
  using PIDController<Scalar,N>::setpoint_;
  using PIDController<Scalar,N>::derivative_setpoint_;
  using PIDController<Scalar,N>::saturation_;
  
  /**
   * @brief Velocity error
   */
  VectorNS v_error_ ;

  /**
   * @brief Velocity command
   */
  VectorNS v_cmd_ ;

  /**
   * @brief Velocity command limits
   */
  VectorNS max_vel_ ;

};

} // namespace kodlab
 