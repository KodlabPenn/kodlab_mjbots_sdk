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

namespace kodlab {

/**
 * @brief PIDController class that allows you to use PID control.
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
   */
  PIDController(const VectorNS & p_gain,
                const VectorNS & i_gain,
                const VectorNS & d_gain, 
                double time_step = 0.001,
                VectorNS deadband = 0.05*VectorNS::Ones(), 
                VectorNS accumulator_limit = 
                    std::numeric_limits<Scalar>::max()*VectorNS::Ones(),
                std::array<VectorNS, 2> saturation = 
                    { -std::numeric_limits<Scalar>::max()*VectorNS::Ones(), 
                       std::numeric_limits<Scalar>::max()*VectorNS::Ones() })
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
   * @brief Construct a vector Pid object
   * @param p_gain Proportional Gain
   * @param i_gain Derivative Gain
   * @param d_gain Integral Gain
   * @param time_step Time difference between 2 updates
   * @param deadband Used to set range within setpoints where no correction 
   * input is applied
   * @param accumulator_limit Used to prevent runaway integrator/windup
   * @param saturation Used to set controller output limits
   */
  PIDController(Scalar p_gain,
                Scalar i_gain,
                Scalar d_gain, 
                double time_step = 0.001,
                VectorNS deadband = 0.05*VectorNS::Ones(),
                VectorNS accumulator_limit = 
                    std::numeric_limits<Scalar>::max()*VectorNS::Ones(),
                std::array<VectorNS, 2> saturation = 
                    { -std::numeric_limits<VectorNS>::max()*VectorNS::Ones(), 
                       std::numeric_limits<VectorNS>::max()*VectorNS::Ones() })
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
   */
  void set_gains(VectorNS p_gain, VectorNS i_gain, VectorNS d_gain) {
    kp_ = p_gain;
    ki_ = i_gain;
    kd_ = d_gain;
  }
  /**
   * @brief Function to set the scalar  PID gains
   * @param p_gain Proportional Gain
   * @param d_gain Derivative Gain
   * @param i_gain Integral Gain
   */
  
  void set_gains(Scalar p_gain, Scalar i_gain, Scalar d_gain) {
    //checking if using for scalar PID
    static_assert(N==1, "Using Scalar function in vector PID");
    kp_ = p_gain*VectorNS::Ones();
    ki_ = i_gain*VectorNS::Ones();
    kd_ = d_gain*VectorNS::Ones();
  }
  
  /**
   * @brief Function to get Kp gain
   */
  VectorNS get_kp_gain(){return kp_;}

  /**
   * @brief Function to get Ki gain
   */
  VectorNS get_ki_gain(){return ki_;}

  /**
   * @brief Function to get Kd gain 
   */
  VectorNS get_kd_gain(){return kd_;}
  
  /**
   * @brief Function to set the scalar deadband
   * @param deadband Deadband that needs to be set
   */
  void set_deadband(Scalar deadband) { deadband_ = deadband*VectorNS::Ones(); }

  /**
   * @brief Function to set the vector deadband
   * @param deadband Deadband that needs to be set
   */
  void set_deadband(const VectorNS & deadband) { deadband_ = deadband; }

  /**
   * @brief Function to get deadband 
   */
  VectorNS get_deadband(){return deadband_;}


  /**
   * @brief Function to set the scalar accumulator limit
   * @param limit accumulator limit
   */
  void set_accumulator(Scalar limit){ accumulator_limit_ = limit*VectorNS::Ones();}

  /**
   * @brief Function to set the vector accumulator limit
   * @param limit accumulator limit
   */
  void set_accumulator(const VectorNS & limit){ accumulator_limit_ = limit;}


  /**
   * @brief Function to set the Saturation
   * @param saturation Saturation
   */
  void set_saturation_limits(std::array<VectorNS, 2> saturation) {
    saturation_ = saturation;
  }

  
  void set_saturation_limits(VectorNS saturation) {
    std::array<VectorNS, 2> out = {-saturation, saturation};
    set_saturation_limits(out);
  }

  void set_saturation_limits(std::array<Scalar,2> saturation) {
    set_saturation_limits({saturation[0]*VectorNS::Ones(), 
                           saturation[1]*VectorNS::Ones()});
  }

  void set_saturation_limits(Scalar saturation) {
    
    set_saturation_limits({-saturation*VectorNS::Ones(), 
                            saturation*VectorNS::Ones()});
  }
  /**
   * @brief Function to get the dimensions
   */
  int get_N(){ return N;}

  /**
   * @brief Function to reset the integral error to zero
   */
  void reset_i_error() { i_error_ = VectorNS::Zero();}

  /**
   * @brief Function to update the state setpoints and reset integral error
   * @param setpoint State Target of the controller.
   * @param derivative_setpoint Derivative State Target of the controller.
   */
  void set_setpoints(const VectorNS & setpoint, 
                     const VectorNS & derivative_setpoint = VectorNS::Zero()){
    setpoint_ = setpoint; 
    derivative_setpoint_ = derivative_setpoint;  
  }

  /**
   * @brief Function to update the state setpoints and reset integral error
   * @param setpoint State Target of the controller.
   * @param derivative_setpoint Derivative State Target of the controller.
   */
  void set_setpoints(Scalar setpoint, Scalar derivative_setpoint = 0){
    //checking if using for scalar PID
    static_assert(N==1, "Using Scalar function in vector PID");
    set_setpoints(setpoint*VectorNS::Ones(), 
                  derivative_setpoint*VectorNS::Ones());
  }
  
  /**
   * @brief Function to get State setpoint
   */
  VectorNS get_state_setpoint(){return setpoint_;}

  /**
   * @brief Function to get derivative state setpoint
   */
  VectorNS get_dstate_setpoint(){return derivative_setpoint_;}

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
   */

  VectorNS Update(const VectorNS & state,
                  const VectorNS & derivative_state,
                  const VectorNS & setpoint,
                  const VectorNS & derivative_setpoint,
                  double time_step) {
    //Calling the setter function 
    set_setpoint(setpoint, derivative_setpoint);
    
    // Calling Nominal Update function.
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
   * @brief Function to update setpoints and PID output.
   * @param state Current state for the controller.
   * @param derivative_state Current derivative state of the controller.
   * @param time_step   Time difference between 2 updates
   * @return Returns the Pid output
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
   */
  Scalar Update(Scalar state, Scalar derivative_state) {
    //checking if using for scalar PID
    static_assert(N==1, "Using Scalar function in vector PID");

    // converts scalar to vector form to calculate output
    return Update(state*VectorNS::Ones(), derivative_state*VectorNS::Ones())(0);
  }
  /**
   * @brief Function to update the PID output given the respective errors
   * @param state Current state for the controller.
   * @param derivative_state Current derivative state of the controller.  
   * @return Returns the Pid output
   */
  VectorNS Update(const VectorNS & state, const VectorNS & derivative_state) {

    return Update(state, derivative_state, time_step_);
  }

  /**
   * @brief Function to update the PID output given the respective errors
   * @param error_p Proportional Error
   * @param error_d Derivative Error
   * @return Returns the Pid output
   */
  
  virtual VectorNS UpdateWithError(const VectorNS & error_p, 
                                   const VectorNS &  error_d, 
                                   double time_step) {
    //accumulating integral error
    i_error_ += error_p*time_step;
    
    //adding a limit on integral error to prevent windup
    i_error_ = i_error_.cwiseMin(accumulator_limit_)
                       .cwiseMax(-accumulator_limit_);

    // Finding the control output from the errors
    output_ = kp_.array() * error_p.array() + kd_.array() * error_d.array() + 
              ki_.array() * i_error_.array();

    //Checking if in Deadband range
    for(int i=0;i<error_p.rows();i++){
      if ( std::abs(error_p.coeff(i)) < deadband_.coeff(i)) {
          output_(i) =0; 
          i_error_(i)=0;
      } 
    }
    output_ = output_.cwiseMin(saturation_[1]).cwiseMax(saturation_[0]);

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
  //Eigen::Matrix<Scalar,Eigen::Dynamic,1> setpoint_;
  VectorNS setpoint_;

  /**
   * @brief State derivative Setpoint
   */
  VectorNS derivative_setpoint_;
  /**
   * @brief Time difference between 2 updates
   */
  double time_step_;

  /**
   * @brief Proportional Gain
   */
  VectorNS kp_;

  /**
   * @brief Derivative Gain
   */
  VectorNS kd_;

  /**
   * @brief Integral Gain
   */
  VectorNS ki_;

  /**
   * @brief Stores the previous error for the d term
   */
  VectorNS d_error_ ;

  /**
   * @brief Sums up the error for calculating the I term
   */
  VectorNS i_error_ ;

  /**
   * @brief Error at each step.
   */
  VectorNS error_ ;

  /**
   * @brief Deadband of the system
   */
  VectorNS deadband_;

  /**
   * @brief accumulator limit of the system if needed
   */
  VectorNS accumulator_limit_;
  
  /**
   * @brief Max control output
   */
  std::array<VectorNS, 2> saturation_;

  /**
   * @brief Output of the function used internally
   */
  VectorNS output_ ;

  static constexpr int kSaturationLowerLimitIndex = 0;
  static constexpr int kSaturationUpperLimitIndex = 1;

};


/**
 * @brief PIDCascade class that allows you to use Cascade PID control.
 */

template <typename Scalar, int N=1>
class PIDCascade: public PIDController<Scalar, N> {

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
   */
  PIDCascade( const VectorNS & p_gain,
              const VectorNS & i_gain,
              const VectorNS & d_gain, 
              double time_step = 0.001,
              VectorNS deadband = 0.5*VectorNS::Ones(),
              VectorNS accumulator_limit = 
                  std::numeric_limits<Scalar>::max()*VectorNS::Ones(),
              std::array<VectorNS, 2> saturation = 
                  { -std::numeric_limits<Scalar>::max()*VectorNS::Ones(), 
                     std::numeric_limits<Scalar>::max()*VectorNS::Ones() })
    : 
      PIDController<Scalar, N>(p_gain, i_gain, d_gain, time_step, deadband,
                              accumulator_limit, saturation){
    
    max_vel_ = std::numeric_limits<Scalar>::max()*VectorNS::Ones();
    }

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
   */
  PIDCascade( Scalar p_gain,
              Scalar i_gain,
              Scalar d_gain, 
              double time_step = 0.001,
              VectorNS deadband = 0.0*VectorNS::Ones(),
              VectorNS accumulator_limit = 
                  std::numeric_limits<Scalar>::max()*VectorNS::Ones(),
              std::array<VectorNS, 2> saturation = 
                  { -std::numeric_limits<Scalar>::max()*VectorNS::Ones(),
                     std::numeric_limits<Scalar>::max()*VectorNS::Ones() })
    : 
      PIDController<Scalar, N>(p_gain, i_gain, d_gain, time_step, deadband,
                               accumulator_limit, saturation){
    
    max_vel_ = std::numeric_limits<Scalar>::max()*VectorNS::Ones();
    }

  /**
   * @brief Function to set maximum velocity
   * @param max_vel velocity limit for the controller.
   */
  void set_max_velocity(VectorNS max_vel) { max_vel_ = max_vel;}

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
    v_error_ = v_cmd_-derivative_state;
    
    // Calling UpdateWithError.   
    return UpdateWithError(error_, v_error_, time_step);
  }

  /**
   * @brief Function to update the cascade PID output given the respective errors
   * @param error_p Proportional Error
   * @param error_d Derivative Error
   * @return Returns the Pid output
   */
  
  VectorNS UpdateWithError(const VectorNS & error_p, 
                           const VectorNS & error_v, 
                           double time_step) override  {
    
    //checking if within deadband range
    for (int i=0; i < error_p.rows();i++){
      if ( std::abs(error_v.coeff(i)) < deadband_.coeff(i)){
        //implementing deadband control logic
        output_(i) = ki_(i)*i_error_(i);
        std::cout << i_error_(i) << std::endl;  
      }
      else {
        //accumulating integral error
        i_error_(i) += error_v(i)*time_step;

        //adding a limit on integral error to prevent windup
        i_error_ = i_error_.cwiseMin(accumulator_limit_)
                           .cwiseMax(-accumulator_limit_);

        //implementing cascade control logic
        output_(i) = kd_(i)*error_v(i) + ki_(i)*i_error_(i);
      }
    }
    
    output_ = output_.cwiseMin(saturation_[1]).cwiseMax(saturation_[0]);
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
   * @brief Maximum Velocity 
   */
  VectorNS max_vel_ ;

};


} // namespace kodlab
 