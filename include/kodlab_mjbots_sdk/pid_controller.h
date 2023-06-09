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
#include <Eigen/Dense>
#include <tuple>
#include <utility>

//need to add comments
template<typename VectorNS>
class Range{

   public:  
    Range(const VectorNS & min, const VectorNS & max): pair_(min,max) {}

    const VectorNS & min(){
      return pair_.first;
    }

    const  VectorNS & max(){
      return pair_.second;
    }
    
    void set_min(const VectorNS & min){
      pair_.first = min;
    }

    void set_max(const VectorNS & max){
      pair_.second = max;
    }

   private:
    std::pair<VectorNS, VectorNS> pair_;
};


namespace kodlab {

/**
 * @brief PIDController class that allows you to use scalar/vector PID control 
 * that is type agnostic and has deadband, windup(accumulator limit) ,
 * control-effort limit capabilities.
 * @tparam Scalar[optional] data type for the controller
 * @tparam  N[optional] dimension of the controller
 * @note Class encapsulates a PID controller given gains \f$ k_p,k_i,k_d \f$ \n 
 * output\f$ \mathrel{=} k_p\cdot error_p \mathrel{+} k_d\cdot error_d 
 * \mathrel{+} k_i\cdot error_i\f$
 * @note  where : \f$ error_p \mathrel{=} setpoint \mathrel{-} state,\\ 
 * error_d \mathrel{=} derivative\_setpoint - derivative\_state ,\\ error_i 
 * \mathrel{{+}{=}} error_p\cdot time\_step \f$
 * @note Deadband limits are on  \f$ error_p,\f$ \n \f$ if(\:deadband_{min}\:
 * \leq\:error_p\:\leq deadband_{max}\:): \;output \mathrel{=} 0,\; error_i 
 * \mathrel{=} 0\f$
 * @note Accumulator limits are applied on \f$ error_i \f$ which clamps it with 
 * the limits as the boundary
 * @note  Saturation(control effort) limits are applied on output which 
 * clamps it with the limits as the boundary
 */
template <typename Scalar, int N=1>
class PIDController {

 public:

  typedef Eigen::Matrix<Scalar,N,1> VectorNS;
  
  /**
   * @brief Construct a vector Pid object
   * @param p_gain Proportional Gain
   * @param i_gain Derivative Gain
   * @param d_gain Integral Gain
   * @param time_step Time difference between 2 updates
   * @param deadband Used to set range within setpoints where no correction 
   * input is applied
   * @param accumulator_limit Used to prevent runaway integrator/windup
   * @param saturation controller output limits
   * \overload
   */
  PIDController(const VectorNS & p_gain,
                const VectorNS & i_gain,
                const VectorNS & d_gain, 
                double time_step = 0.001,
                Range<VectorNS> deadband = {-0.05*VectorNS::Ones(), 
                                             0.05*VectorNS::Ones()}, 
                Range<VectorNS> accumulator_limit = 
                    {-std::numeric_limits<Scalar>::max()*VectorNS::Ones(), 
                      std::numeric_limits<Scalar>::max()*VectorNS::Ones()},
                Range<VectorNS> saturation = 
                    {-std::numeric_limits<Scalar>::max()*VectorNS::Ones(), 
                      std::numeric_limits<Scalar>::max()*VectorNS::Ones()})
    : 
      deadband_(deadband),
      accumulator_limit_(accumulator_limit),
      saturation_(saturation),
      time_step_(time_step),
      kp_(p_gain),
      ki_(i_gain),
      kd_(d_gain) {

    error_ = VectorNS::Zero();
    d_error_ = VectorNS::Zero();
    set_setpoints(VectorNS::Zero(), VectorNS::Zero());
  }
  
  /**
   * @brief Construct a scalar Pid object
   * @param p_gain Proportional Gain
   * @param i_gain Derivative Gain
   * @param d_gain Integral Gain
   * @param time_step Time difference between 2 updates
   * @param deadband Used to set range within setpoints where no correction 
   * input is applied
   * @param accumulator_limit Used to prevent runaway integrator/windup
   * @param saturation Used to set controller output limits
   * \overload
   */
  PIDController(Scalar p_gain,
                Scalar i_gain,
                Scalar d_gain, 
                double time_step = 0.001,
                Range<VectorNS> deadband = {-0.05*VectorNS::Ones(), 
                                             0.05*VectorNS::Ones()}, 
                Range<VectorNS> accumulator_limit = 
                    {-std::numeric_limits<Scalar>::max()*VectorNS::Ones(), 
                      std::numeric_limits<Scalar>::max()*VectorNS::Ones()},
                Range<VectorNS> saturation = 
                    {-std::numeric_limits<Scalar>::max()*VectorNS::Ones(), 
                      std::numeric_limits<Scalar>::max()*VectorNS::Ones()})
    : 
      deadband_(deadband),
      accumulator_limit_(accumulator_limit),
      saturation_(saturation),
      time_step_(time_step){

    set_gains(p_gain, i_gain, d_gain); 
    error_ = VectorNS::Zero();
    d_error_ = VectorNS::Zero();
    set_setpoints(VectorNS::Zero(), VectorNS::Zero());    
  }

  /**
   * @brief Function to set the vector PID gains 
   * @param p_gain Proportional Gain
   * @param d_gain Derivative Gain
   * @param i_gain Integral Gain
   * \overload
   */
  void set_gains(VectorNS p_gain, VectorNS i_gain, VectorNS d_gain) {
    kp_ = p_gain;
    ki_ = i_gain;
    kd_ = d_gain;
  }

  /**
   * @brief Function to set the scalar PID gains
   * @param p_gain Proportional Gain
   * @param d_gain Derivative Gain
   * @param i_gain Integral Gain
   * \overload
   */
  void set_gains(Scalar p_gain, Scalar i_gain, Scalar d_gain) {
    //checking if using for scalar PID
    static_assert(N == 1, "Using Scalar function in vector PID");
    kp_ = p_gain*VectorNS::Ones();
    ki_ = i_gain*VectorNS::Ones();
    kd_ = d_gain*VectorNS::Ones();
  }
  
  /**
   * @brief Function to get Kp gain
   */
  const VectorNS & get_kp_gain(){return kp_;}

  /**
   * @brief Function to get Ki gain
   */
  const VectorNS & get_ki_gain(){return ki_;}

  /**
   * @brief Function to get Kd gain 
   */
  const VectorNS & get_kd_gain(){return kd_;}
  
  /**
   * @brief Function to set the scalar deadband
   * @param limit Deadband that needs to be set
   * \overload 
   */
  void set_deadband(Scalar deadband) { 
    deadband_.set_min(-deadband*VectorNS::Ones());
    deadband_.set_max( deadband*VectorNS::Ones()); 
  }

  /**
   * @brief Function to set the scalar deadband
   * @param min Deadband lower bound
   * @param max Deadband upper bound
   * \overload 
   */
  void set_deadband(Scalar min, Scalar max) { 
    deadband_.set_min(min*VectorNS::Ones());
    deadband_.set_max(max*VectorNS::Ones());
  }

  /**
   * @brief Function to set the vector deadband
   * @param deadband Deadband that needs to be set
   * \overload 
   */
  void set_deadband(const VectorNS & deadband) { 
    deadband_.set_min(-deadband);
    deadband_.set_max(deadband);
  }

  /**
   * @brief Function to set the vector deadband
   * @param min Deadband lower bound
   * @param max Deadband upper bound
   * \overload
   */
  void set_deadband(const VectorNS & min, const VectorNS & max) { 
    deadband_.set_min(min);
    deadband_.set_max(max);
  }

  /**
   * @brief Function to get deadband maxima
   * @return Max deadband range
   */
  const VectorNS & get_deadband_max(){return deadband_.max();}

  /**
   * @brief Function to get deadband minima
   * @return Min deadband range
   */
  const VectorNS & get_deadband_min(){return deadband_.min();}

  /**
   * @brief Function to set the scalar accumulator limit
   * @param limit accumulator limit
   * \overload 
   */
  void set_accumulator(Scalar limit){ 
    accumulator_limit_.set_min(-limit*VectorNS::Ones()); 
    accumulator_limit_.set_max(limit*VectorNS::Ones());
  }

  /**
   * @brief Function to set the scalar accumulator limit
   * @param min Accumulator limit lower bound
   * @param max Accumulator limit upper bound
   * \overload 
   */
  void set_accumulator(Scalar min, Scalar max){ 
    accumulator_limit_.set_min( min*VectorNS::Ones()); 
    accumulator_limit_.set_max( max*VectorNS::Ones());
  }

  /**
   * @brief Function to set the vector accumulator limit
   * @param limit accumulator limit
   * \overload 
   */
  void set_accumulator(const VectorNS & limit){ 
    accumulator_limit_.set_min(-limit);
    accumulator_limit_.set_max(limit);
  }

  /**
   * @brief Function to set the vector accumulator limit
   * @param min Accumulator limit lower bound
   * @param max Accumulator limit upper bound
   * \overload
   */
  void set_accumulator(const VectorNS & min, const VectorNS & max){ 
    accumulator_limit_.set_min(min);
    accumulator_limit_.set_max(max);
  }

  /**
   * @brief Function to get maximum accumulator limit
   * @return Max accumulator limit
   */
  const VectorNS & get_accumulator_max(){return accumulator_limit_.max();}

  /**
   * @brief Function to get minimum accumulator limit
   * @return Min accumulator limit
   */
  const VectorNS & get_accumulator_min(){return accumulator_limit_.min();}

  /**
   * @brief Function to set the vector saturation limits of the controller
   * @param saturation Saturation limit
   * \overload 
   */
  void set_saturation_limits(const VectorNS & saturation) {
    saturation_.set_min(-saturation);
    saturation_.set_max(saturation);
  }

  /**
   * @brief Function to set the vector saturation limits of the controller
   * @param min Saturation lower limit
   * @param max Saturation upper limit
   * \fn
   */
  void set_saturation_limits(const VectorNS & min, const VectorNS & max) {
    saturation_.set_min(min);
    saturation_.set_max(max);
  }

  /**
   * @brief Function to set the scalar saturation limits of the controller
   * @param saturation Saturation limit
   * \overload 
   */
  void set_saturation_limits(Scalar saturation) {
    
    saturation_.set_min(-saturation*VectorNS::Ones());
    saturation_.set_max(saturation*VectorNS::Ones());
  }

  /**
   * @brief Function to set the scalar saturation limits of the controller
   * @param min Saturation lower limit
   * @param max Saturation upper limit
   * \overload 
   */
  void set_saturation_limits(Scalar min, Scalar max) {
    
    saturation_.set_min(min*VectorNS::Ones());
    saturation_.set_max(max*VectorNS::Ones());
  }
  
  /**
   * @brief Function to get maximum control input
   * @return Max saturation limit
   */
  const VectorNS & get_saturation_max(){return saturation_.max();}

  /**
   * @brief Function to get minimum control input
   * @return Min saturation limit
   */
  const VectorNS & get_saturation_min(){return saturation_.min();}

  /**
   * @brief Function to get the dimensions
   * @return N
   */
  int get_N(){ return N;}

  /**
   * @brief Function to reset the integral error to zero
   */
  void reset_i_error() { i_error_ = VectorNS::Zero();}

  /**
   * @brief Function to update the state setpoints
   * @param setpoint State Target of the controller.
   * @param derivative_setpoint Derivative State Target of the controller.
   * \overload
   */
  void set_setpoints(const VectorNS & setpoint, 
                     const VectorNS & derivative_setpoint = VectorNS::Zero()){
    setpoint_ = setpoint; 
    derivative_setpoint_ = derivative_setpoint;  
  }

  /**
   * @brief Function to update the state setpoints 
   * @param setpoint State Target of the controller.
   * @param derivative_setpoint Derivative State Target of the controller.
   * \overload
   */
  void set_setpoints(Scalar setpoint, Scalar derivative_setpoint = 0){
    //checking if using for scalar PID
    static_assert(N==1, "Using Scalar function in vector PID");
    set_setpoints(setpoint*VectorNS::Ones(), 
                  derivative_setpoint*VectorNS::Ones());
  }
  
  /**
   * @brief Function to get tate setpoint
   * @return State setpoint
   */
  const VectorNS & get_state_setpoint(){return setpoint_;}

  /**
   * @brief Function to get derivative state setpoint
   * @return derivative state setpoint
   */
  const VectorNS & get_dstate_setpoint(){return derivative_setpoint_;}

  /**
   * @brief Function to get state & derivative state setpoints
   * @return state & derivative state setpoints as std::pair
   */
  std::pair<VectorNS, VectorNS> get_setpoints(){
    
    return {setpoint_, derivative_setpoint_};
  }

  /**
   * @brief Function to update setpoints and vector PID output.
   * @param state Current state for the controller.
   * @param derivative_state Current derivative state of the controller.
   * @param setpoint Desired state of the controller
   * @param derivative_setpoint Desired derivative state of the controller
   * @param time_step   Time difference between 2 updates
   * @return Returns the Pid output
   * \overload
   */
  VectorNS Update(const VectorNS & state,
                  const VectorNS & derivative_state,
                  const VectorNS & setpoint,
                  const VectorNS & derivative_setpoint,
                  double time_step) {
    
    set_setpoint(setpoint, derivative_setpoint);
    return Update(state, derivative_state, time_step);
  }

  /**
   * @brief Function to update setpoints and scalar PID output.
   * @param state Current state for the controller.
   * @param derivative_state Current derivative state of the controller.
   * @param setpoint Desired state of the controller
   * @param derivative_setpoint Desired derivative state of the controller
   * @param time_step   Time difference between 2 updates
   * @return Returns the Pid output
   * \overload 
   */
  Scalar Update(Scalar state, 
                Scalar derivative_state,
                Scalar setpoint,
                Scalar derivative_setpoint,
                double time_step) {
    //checking if using for scalar PID
    static_assert(N==1, "Using Scalar function in vector PID");
    set_setpoint(setpoint, derivative_setpoint);
    
    // Calling Nominal Update function.
    return Update(state*VectorNS::Ones(), 
                  derivative_state*VectorNS::Ones(), 
                  time_step)(0);
  }

  /**
   * @brief Function to update PID output.
   * @param state Current state for the controller.
   * @param derivative_state Current derivative state of the controller.
   * @param time_step   Time difference between 2 updates
   * @return Returns the Pid output
   * \overload
   */
  virtual VectorNS Update(const VectorNS & state, 
                          const VectorNS & derivative_state, 
                          double time_step) {
    
    // Proportional error
    error_ = setpoint_ - state;
    // Derivative error
    d_error_ =  derivative_setpoint_ - derivative_state;
    // Calling UpdateWithError.
    return UpdateWithError(error_, d_error_, time_step);;
  }

  /**
   * @brief Function to update setpoints and PID output.
   * @param state Scalar state for the controller.
   * @param derivative_state Scalar derivative state of the controller.
   * @return Returns the Pid output
   * \overload 
   */
  Scalar Update(Scalar state, Scalar derivative_state) {
    //checking if using for scalar PID
    static_assert(N==1, "Using Scalar function in vector PID");

    // converts scalar to vector form to calculate output
    return Update(state*VectorNS::Ones(), derivative_state*VectorNS::Ones())(0);
  }
  /**
   * @brief Function to update the PID output given the state and derivative 
   * state
   * @param state Current state for the controller.
   * @param derivative_state Current derivative state of the controller.  
   * @return Returns the Pid output
   * \overload 
   */
  VectorNS Update(const VectorNS & state, const VectorNS & derivative_state) {

    return Update(state, derivative_state, time_step_);
  }

  /**
   * @brief Function to update the PID output given the respective errors
   * @param error_p Proportional Error
   * @param error_d Derivative Error
   * @return Returns the Pid output
   * \overload
   */
  virtual VectorNS UpdateWithError(const VectorNS & error_p, 
                                   const VectorNS &  error_d, 
                                   double time_step) {
    //accumulating integral error
    i_error_ += error_p*time_step;
    
    //adding a limit on integral error to prevent windup
    i_error_ = i_error_.cwiseMin(accumulator_limit_.max())
                       .cwiseMax(accumulator_limit_.min());

    // Finding the control output from the errors
    output_ = kp_.array() * error_p.array() + kd_.array() * error_d.array() + 
              ki_.array() * i_error_.array();

    //Checking if in Deadband range
    for(int i=0; i < N; i++){
      if ( error_p.coeff(i) <= deadband_.max().coeff(i) && 
           deadband_.min().coeff(i) <= error_p.coeff(i)) {
          output_(i) =0; 
          i_error_(i)=0;
      } 
    }
    output_ = output_.cwiseMin(saturation_.max())
                     .cwiseMax(saturation_.min());

    return output_;
  }

  /**
   * @brief Destructor
   */
  ~PIDController() {}


protected:
  
  /**
   * @brief State setpoint 
   */
  VectorNS setpoint_;

  /**
   * @brief State derivative setpoint
   */
  VectorNS derivative_setpoint_;

  /**
   * @brief Time difference between 2 updates 
   */
  double time_step_;

  /**
   * @brief Proportional gain
   */
  VectorNS kp_;

  /**
   * @brief Derivative gain
   */
  VectorNS kd_;

  /**
   * @brief Integral gain
   */
  VectorNS ki_;

  /**
   * @brief Derivative state/velocity error at each time-step
   */
  VectorNS d_error_ ;

  /**
   * @brief Accumulated state error over previous time-steps
   */
  VectorNS i_error_ ;

  /**
   * @brief State error at each time-step
   */
  VectorNS error_ ;

  /**
   * @brief Deadband of the system
   */
  Range<VectorNS> deadband_;

  /**
   * @brief Accumulator limits of i_error_
   */
  Range<VectorNS> accumulator_limit_;
  
  /**
   * @brief Control output limits
   */
  Range<VectorNS> saturation_;

  /**
   * @brief Output of the function used internally
   */
  VectorNS output_ ;

};

} // namespace kodlab
 
 