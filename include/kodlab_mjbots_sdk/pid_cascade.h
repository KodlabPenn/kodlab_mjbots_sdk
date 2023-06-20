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
#include <utility>
#include <type_traits>

#include "kodlab_mjbots_sdk/pid_controller.h"
#include "kodlab_mjbots_sdk/math.h"

namespace kodlab {

/**
 * @brief Implements a type-agnostic PID controller with a cascade architecture
 * @details Sets up a scalar/Eigen::Vector based PID controller
 * that is type agnostic and has deadband, windup(accumulator) limit,
 * saturation/control-effort limit capabilities. The class template can be
 * implicitly instantiated with Scalar and N deduced based on the argument of 
 * the constructor.
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
 * @note Velocity limits are applied on the commanded velocity, \f$ v_{cmd}\f$,
 * clamping it
 * @note Accumulator limits are applied on the integral term's effort, 
 * \f$ k_i * i_{error} \f$, clamping it
 * @note Saturation (control effort) limits are applied on the output effort,
 * \f$ output \f$, clamping it
 */
template <typename Scalar, int N>
class PIDCascade: public PIDController<Scalar, N> {

public:

  typedef Eigen::Matrix<Scalar,N,1> VectorNS;
  typedef math::Range<VectorNS> RangeNS;
  using PIDController<Scalar,N>::Update;
  using PIDController<Scalar,N>::UpdateWithError;
  
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
              RangeNS deadband = { VectorNS::Zero(), VectorNS::Zero() }, 
              RangeNS accumulator_limits = 
                  {-std::numeric_limits<Scalar>::max() * VectorNS::Ones(), 
                    std::numeric_limits<Scalar>::max() * VectorNS::Ones()},
              RangeNS saturation_limits = 
                  {-std::numeric_limits<Scalar>::max() * VectorNS::Ones(), 
                    std::numeric_limits<Scalar>::max() * VectorNS::Ones()},
              RangeNS velocity_limits = 
                  {-std::numeric_limits<Scalar>::max() * VectorNS::Ones(), 
                    std::numeric_limits<Scalar>::max() * VectorNS::Ones()})
              : PIDController<Scalar, N>( p_gain, 
                                          i_gain,
                                          d_gain,
                                          time_step,
                                          deadband,
                                          accumulator_limits,
                                          saturation_limits),
                v_limits_(velocity_limits){}

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
              RangeNS deadband = { VectorNS::Zero(), VectorNS::Zero() }, 
              RangeNS accumulator_limits = 
                  {-std::numeric_limits<Scalar>::max() * VectorNS::Ones(), 
                    std::numeric_limits<Scalar>::max() * VectorNS::Ones()},
              RangeNS saturation_limits = 
                  {-std::numeric_limits<Scalar>::max() * VectorNS::Ones(), 
                    std::numeric_limits<Scalar>::max() * VectorNS::Ones()},
              RangeNS velocity_limits = 
                  {-std::numeric_limits<Scalar>::max() * VectorNS::Ones(), 
                    std::numeric_limits<Scalar>::max() * VectorNS::Ones()})
              : PIDController<Scalar, N>(p_gain, 
                                         i_gain,
                                         d_gain,
                                         time_step,
                                         deadband,
                                         accumulator_limits,
                                         saturation_limits),
                v_limits_(velocity_limits){}

  /**
   * @brief Function to set maximum velocity magnitudes
   * @param max_vels velocity limits for the controller
   * \overload
   */
  void set_velocity_limits(const VectorNS & max_vels) {
    v_limits_.set_min(-max_vels);
    v_limits_.set_max( max_vels);
  }

  /**
   * @brief Function to set min and max velocity limits
   * @param min_vels min velocity limits for the controller
   * @param max_vels max velocity limits for the controller
   * \overload
   */
  void set_velocity_limits(const VectorNS & min_vels, 
                           const VectorNS & max_vels) {
    v_limits_.set_min(min_vels);
    v_limits_.set_max(max_vels);
  }

  /**
   * @brief Function to set maximum velocity magnitude
   * @param max_vel velocity limit for the controller
   * \overload
   */
  void set_velocity_limits(Scalar max_vel) { 
    v_limits_.set_min(-max_vel * VectorNS::Ones());
    v_limits_.set_max( max_vel * VectorNS::Ones());
  }

  /**
   * @brief Function to set min and max velocity limits
   * @param min_vel min velocity limit for the controller
   * @param max_vel max velocity limit for the controller
   * \overload
   */
  void set_velocity_limits(Scalar min_vel, Scalar max_vel) { 
    v_limits_.set_min(min_vel * VectorNS::Ones());
    v_limits_.set_max(max_vel * VectorNS::Ones());
  }

  /**
   * @brief Function to get velocity limits
   * @return max velocities for the controller.
   */
  const math::Range<VectorNS>& get_velocity_max() const {
    return v_limits_.max();
  }

  /**
   * @brief Function to get velocity max limits
   * @return min velocities for the controller.
   */
  const math::Range<VectorNS>& get_velocity_min() const {
    return v_limits_.min();
  }

  /**
   * @brief Function to update setpoints and cascade PID output.
   * @param state Current state for the controller.
   * @param derivative_state Current derivative state of the controller.
   * @param time_step   Time difference between 2 updates
   * @return Returns the PID output
   * \overload
   */
  VectorNS Update (const VectorNS & state,
                   const VectorNS & derivative_state,
                   double time_step) override  {
    
    // Proportional error
    error_ = setpoint_ - state;
    
    // V_cmd with max velocity limits
    v_cmd_ = kp_.array() * error_.array() + derivative_setpoint_.array();
    v_cmd_ = v_cmd_.cwiseMin(v_limits_.max()).cwiseMax(v_limits_.min());
    
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
  VectorNS UpdateWithError(const VectorNS & error_p_in, 
                           const VectorNS & error_v_in, 
                           double time_step) override  {
    VectorNS error_v = error_v_in; //copy for zeroing out
    //checking if within deadband range
    for (int i=0; i < N; i++){
      if ( error_v.coeff(i) < deadband_.max().coeff(i) && 
           deadband_.min().coeff(i) < error_v.coeff(i)) {
        //implementing deadband control logic
        error_v(i) = 0;  
      }
      //if ki is 0, stop accumulating
      if (ki_(i) == 0){
        i_error_(i) = 0;
      }else{//else keep accumulating
        i_error_(i) += error_v(i) * time_step; 

        //adding a limit on integral error to prevent windup
        i_error_(i) = std::clamp(i_error_(i),
                                 accumulator_limits_.min()(i)/ki_(i),
                                 accumulator_limits_.max()(i)/ki_(i));
      }
    }

    //implementing cascade control logic
    output_ = kd_.array() * error_v.array() + ki_.array() * i_error_.array();

    output_ = output_.cwiseMin(saturation_limits_.max()).cwiseMax(saturation_limits_.min());
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
  using PIDController<Scalar,N>::accumulator_limits_;
  using PIDController<Scalar,N>::setpoint_;
  using PIDController<Scalar,N>::derivative_setpoint_;
  using PIDController<Scalar,N>::saturation_limits_;
  
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
  RangeNS v_limits_ ;

};

} // namespace kodlab
 