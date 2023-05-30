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
   * @param d_gain Derivative Gain
   * @param i_gain Integral Gain
   * @param time_step Time difference between 2 updates
   */
  PIDController(const VectorNS & p_gain,const VectorNS & d_gain,const VectorNS & i_gain, double time_step=0.001,
    VectorNS deadband = 0.5*VectorNS::Ones(), VectorNS windup_limit = std::numeric_limits<Scalar>::max()*VectorNS::Ones(),
    std::array<Scalar, 2> saturation = { -std::numeric_limits<Scalar>::max(), std::numeric_limits<Scalar>::max() })
    : 
      deadband_(deadband),
      windup_limit_(windup_limit),
      saturation_(saturation),
      time_step_(time_step){

    set_gains(p_gain,d_gain,i_gain); 
    const int input_size = kp_.rows();
    error_ = VectorNS::Zero();
    d_error_ = VectorNS::Zero();

    setpoint_ = VectorNS::Zero();
    derivative_setpoint_ =VectorNS::Zero();    
  }
  
  /**
   * @brief Construct a scalar Pid object
   * @param p_gain Proportional Gain
   * @param d_gain Derivative Gain
   * @param i_gain Integral Gain
   * @param time_step Time difference between 2 updates
   */
  PIDController(Scalar p_gain,Scalar  d_gain,Scalar  i_gain, double time_step=0.001,
    VectorNS deadband = 0.5*VectorNS::Ones(),VectorNS windup_limit = std::numeric_limits<Scalar>::max()*VectorNS::Ones(),
    std::array<Scalar, 2> saturation = { -std::numeric_limits<Scalar>::max(), std::numeric_limits<Scalar>::max() })
    : 
      deadband_(deadband),
      windup_limit_(windup_limit),
      saturation_(saturation),
      time_step_(time_step){

    set_gains(p_gain,d_gain,i_gain); 
    const int input_size = kp_.rows();
    error_ = VectorNS::Zero(input_size);
    d_error_ = VectorNS::Zero(input_size);

    setpoint_ = VectorNS::Zero(input_size);
    derivative_setpoint_ =VectorNS::Zero(input_size);    
  }

  /**
   * @brief Function to set the vector PID gains 
   * @param p_gain Proportional Gain
   * @param d_gain Derivative Gain
   * @param i_gain Integral Gain
   */
  void set_gains(VectorNS p_gain, VectorNS d_gain,VectorNS i_gain) {
    kp_ = p_gain;
    kd_ = d_gain;
    ki_ = i_gain;
  }
  /**
   * @brief Function to set the scalar  PID gains
   * @param p_gain Proportional Gain
   * @param d_gain Derivative Gain
   * @param i_gain Integral Gain
   */
  
  void set_gains(Scalar p_gain, Scalar d_gain, Scalar i_gain) {
    //checking if using for scalar PID
    static_assert(N==1, "Using Scalar function in vector PID");
    kp_ = p_gain*VectorNS::Ones();
    kd_ = d_gain*VectorNS::Ones();
    ki_ = i_gain*VectorNS::Ones();
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
  void set_deadband(double deadband) { deadband_ = deadband*VectorNS::Ones(); }

  /**
   * @brief Function to set the vector deadband
   * @param deadband Deadband that needs to be set
   */
  void set_deadband(VectorNS deadband) { deadband_ = deadband; }

  /**
   * @brief Function to get deadband 
   */
  VectorNS get_deadband(){return deadband_;}


  /**
   * @brief Function to set the scalar Windup limit
   * @param limit Windup limit
   */
  void set_windup(double limit){ windup_limit_ = limit*VectorNS::Ones();}

  /**
   * @brief Function to set the vector Windup limit
   * @param limit Windup limit
   */
  void set_windup(const VectorNS & limit){ windup_limit_ = limit;}

  /**
   * @brief Function to set the Saturation
   * @param saturation Saturation
   */
  void set_saturation(std::array<Scalar, 2> saturation) {
    saturation_ = saturation;
  }

  /**
   * @brief Function to get the dimensions
   */
  int get_N(){ return N;}

  /**
   * @brief Function to update the state setpoints and reset integral error
   * @param setpoint State Target of the controller.
   */

  void set_setpoints(const VectorNS & setpoint){
    setpoint_ = setpoint;
    i_error_ = VectorNS::Zero(setpoint.rows());           
  }

  /**
   * @brief Function to update the state setpoints and reset integral error
   * @param setpoint State Target of the controller.
   * @param derivative_setpoint Derivative State Target of the controller.
   */
  void set_setpoints(const VectorNS & setpoint, const VectorNS & derivative_setpoint){
    setpoint_ = setpoint;
    derivative_setpoint_ = derivative_setpoint;
    i_error_ = VectorNS::Zero(setpoint.rows());
  }

  /**
   * @brief Function to update the state setpoints and reset integral error
   * @param setpoint State Target of the controller.
   * @param derivative_setpoint Derivative State Target of the controller.
   */
  void set_setpoints(Scalar setpoint, Scalar derivative_setpoint = 0){
    //checking if using for scalar PID
    static_assert(N==1, "Using Scalar function in vector PID");
    setpoint_ = setpoint*VectorNS::Ones();
    derivative_setpoint_ = derivative_setpoint*VectorNS::Ones();
    i_error_ = VectorNS::Zero(); //required?
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

  void Update(const VectorNS & state,const VectorNS & derivative_state,const VectorNS & setpoint,const VectorNS & 
      derivative_setpoint,double time_step) {
    
    //Calling the setter function 
    set_setpoint(setpoint,derivative_setpoint);
    
    // Calling Nominal Update function.
    Update(state,derivative_state,time_step);
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
  void Update(Scalar state,Scalar derivative_state,Scalar setpoint,Scalar derivative_setpoint,double time_step) {
    //checking if using for scalar PID
    static_assert(N==1, "Using Scalar function in vector PID");
    set_setpoint(setpoint,derivative_setpoint);
    
    // Calling Nominal Update function.
    Update(state,derivative_state,time_step);
  }

  /**
   * @brief Function to update setpoints and PID output.
   * @param state Current state for the controller.
   * @param derivative_state Current derivative state of the controller.
   * @param time_step   Time difference between 2 updates
   * @return Returns the Pid output
   */
  virtual VectorNS Update(const VectorNS & state,const VectorNS & derivative_state,double time_step) {
    // Calculate the errors
    // Proportional error
    error_ = setpoint_ - state;
    
    // Derivative error
    d_error_ =  derivative_setpoint_ - derivative_state;
    
    
    // Calling UpdateWithError.
    return UpdateWithError(error_, d_error_,time_step);;
  }

  /**
   * @brief Function to update setpoints and PID output.
   * @param state Scalar state for the controller.
   * @param derivative_state Scalar derivative state of the controller.
   * @return Returns the Pid output
   */
  VectorNS Update(Scalar state,Scalar derivative_state) {
    //checking if using for scalar PID
    static_assert(N==1, "Using Scalar function in vector PID");
    // converts scalar to vector form to calculate output
    return Update(state*VectorNS::Ones(),derivative_state*VectorNS::Ones());
  }
  /**
   * @brief Function to update the PID output given the respective errors
   * @param state Current state for the controller.
   * @param derivative_state Current derivative state of the controller.  
   * @return Returns the Pid output
   */
  VectorNS Update(const VectorNS & state,const VectorNS & derivative_state) {

    return Update(state,derivative_state,time_step_);
  }

  /**
   * @brief Function to update the PID output given the respective errors
   * @param error_p Proportional Error
   * @param error_d Derivative Error
   * @return Returns the Pid output
   */
  
  virtual VectorNS UpdateWithError(const VectorNS & error_p, const VectorNS &  error_d, double time_step) {
    // Computing the integral error
    i_error_ += error_p*time_step;

    //adding a limit on integral error to prevent windup
    i_error_ = i_error_.cwiseMin(windup_limit_).cwiseMax(-windup_limit_);

    // Finding the control output from the errors
    output_ = kp_.array() * error_p.array() + kd_.array() * error_d.array() + ki_.array() * i_error_.array();

    //Checking if in Deadband range
    for(int i=0;i<error_p.rows();i++){
      if ( std::abs(error_p.coeff(i)) < deadband_.coeff(i)) {
          output_(i) =0; 
          i_error_(i)=0;
      } 
    }
    
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
   * @brief Windup limit of the system if needed
   */
  VectorNS windup_limit_;
  
  /**
   * @brief Max control output
   */
  std::array<Scalar, 2> saturation_;

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
   * @param d_gain Derivative Gain
   * @param i_gain Integral Gain
   * @param time_step Time difference between 2 updates
   */
  PIDCascade( const VectorNS & p_gain,
              const VectorNS & d_gain,
              const VectorNS & i_gain, 
              double time_step=0.001,
              VectorNS deadband = 0.5*VectorNS::Ones(),
              VectorNS windup_limit = std::numeric_limits<Scalar>::max()*VectorNS::Ones(),
              std::array<Scalar, 2> saturation = { -std::numeric_limits<Scalar>::max(), 
                                                    std::numeric_limits<Scalar>::max() })
    : PIDController<Scalar, N>(p_gain,d_gain,i_gain,time_step,deadband,windup_limit,saturation){}

  /**
   * @brief Construct a scalar Pid object
   * @param p_gain Proportional Gain
   * @param d_gain Derivative Gain
   * @param i_gain Integral Gain
   * @param time_step Time difference between 2 updates
   */
  PIDCascade(Scalar p_gain,Scalar  d_gain,Scalar  i_gain, double time_step=0.001,
    VectorNS deadband = 0.5*VectorNS::Ones(),VectorNS windup_limit = std::numeric_limits<Scalar>::max()*VectorNS::Ones(),
    std::array<Scalar, 2> saturation = { -std::numeric_limits<Scalar>::max(), std::numeric_limits<Scalar>::max() })
    : 
      PIDController<Scalar, N>(p_gain,d_gain,i_gain,time_step,deadband,windup_limit,saturation){}

  
  VectorNS Update(Scalar state,Scalar derivative_state) {
    //checking if using for scalar PID
    static_assert(N==1, "Using Scalar function in vector PID");
    // converts scalar to vector form to calculate output
    return Update(state*VectorNS::Ones(),derivative_state*VectorNS::Ones());
  }

  VectorNS Update(const VectorNS & state,const VectorNS & derivative_state) {

    return Update(state,derivative_state,time_step_);
  }  

  /**
   * @brief Function to update setpoints and PID output.
   * @param state Current state for the controller.
   * @param derivative_state Current derivative state of the controller.
   * @param time_step   Time difference between 2 updates
   * @return Returns the Pid output
   */
  VectorNS Update (const VectorNS & state,const VectorNS & derivative_state,double time_step) override  {
    // Calculate the errors
    // Proportional error
    error_ = setpoint_ - state;
    
    v_cmd_ = kp_.array() *error_.array() + derivative_setpoint_.array();
    
    v_error_ = v_cmd_-derivative_state;
    
    // Calling UpdateWithError.
    return UpdateWithError(error_, v_error_,time_step);
  }

  /**
   * @brief Function to update the PID output given the respective errors
   * @param error_p Proportional Error
   * @param error_d Derivative Error
   * @return Returns the Pid output
   */
  
  VectorNS UpdateWithError(const VectorNS & error_p, const VectorNS &  error_v, double time_step) override  {
    // Computing the integral error
    i_error_ += error_v*time_step;
    
    //adding a limit on integral error to prevent windup
    i_error_ = i_error_.cwiseMin(windup_limit_).cwiseMax(-windup_limit_);

    // Finding the control output from the errors
    output_ = kd_.array() * error_v.array() + ki_.array() * i_error_.array();
     
    //Checking if in Deadband range
    for(int i=0;i<error_p.rows();i++){
      if ( std::abs(error_p.coeff(i)) < deadband_.coeff(i)) {
          output_(i) =0; 
          i_error_(i)=0;
      } 
    }
    
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
  using PIDController<Scalar,N>::windup_limit_;
  using PIDController<Scalar,N>::setpoint_;
  using PIDController<Scalar,N>::derivative_setpoint_;

  /**
   * @brief Stores the previous error for the d term
   */
  VectorNS v_error_ ;

  /**
   * @brief Stores the previous error for the d term
   */
  VectorNS v_cmd_ ;
};


} // namespace kodlab
 