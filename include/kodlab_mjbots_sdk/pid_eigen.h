#pragma once

#include <math.h>
#include <array>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>

namespace kodlab {

  /**
   * @brief PIDController class that allows you to use PID control.
   */
  
  template <typename Scalar, int N>
  class PIDController {

   public:
    typedef Eigen::Matrix<Scalar,N,1> VectorNS;
    /**
     * @brief Construct a Pid object
     * @param p_gain Proportional Gain
     * @param d_gain Derivative Gain
     * @param i_gain Integral Gain
     * @param time_step Time difference between 2 updates
     */
    
    PIDController(const VectorNS & p_gain,const VectorNS & d_gain,const VectorNS & i_gain, double time_step=0.001,
       double deadband = 1.0,
      std::array<double, 2> saturation = { -INFINITY, INFINITY })
      : 
        deadband_(deadband),
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
     * @brief Function to set the PID gains
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
     * @brief Function to update the state setpoints and reset integral error
     * @param setpoint State Target of the controller.
     * @param derivative_setpoint Derivative State Target of the controller.
     */
    void set_setpoint(const VectorNS & setpoint){
      setpoint_ = setpoint;
      i_error_ = VectorNS::Zero(setpoint.rows());           
    }

    void set_setpoint(const VectorNS & setpoint, const VectorNS & derivative_setpoint){
      setpoint_ = setpoint;
      derivative_setpoint_ = derivative_setpoint;
      i_error_ = VectorNS::Zero(setpoint.rows());
    }
    /**
     * @brief Function to print out setpoints
     */
    void get_setpoints(){std::cout << "Setpoints are" << setpoint_  <<"\n";}

    /**
     * @brief Function to update setpoints and PID output.
     * @param state Current state for the controller.
     * @param derivative_state Current derivative state of the controller.
     * @param setpoint Desired state of the controller
     * @param derivative_setpoint Desired derivative state of the controller
     * @param time_step   Time difference between 2 updates
     * @return Returns the Pid output
     */
    void Update(VectorNS state,VectorNS derivative_state,VectorNS setpoint,VectorNS derivative_setpoint,double time_step) {
      
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
    VectorNS Update(VectorNS state,VectorNS derivative_state,double time_step) {
      // Calculate the errors
      // Proportional error
      error_ = setpoint_ - state;
      
      // Derivative error
      d_error_ =  derivative_setpoint_ - derivative_state;
      
      // Checking if in Deadband RAnge
      if ((state-setpoint_).isMuchSmallerThan(Eigen::ArrayXd::Ones(N)*deadband_/2)){
        return VectorNS::Zero(N);
      }
      // Calling UpdateWithError.
      else return UpdateWithError(error_, d_error_,time_step);;
    }

    /**
     * @brief Function to update the PID output given the respective errors
     * @param state Current state for the controller.
     * @param derivative_state Current derivative state of the controller.  
     * @return Returns the Pid output
     */
    VectorNS Update(VectorNS state,VectorNS derivative_state) {

      return Update(state,derivative_state,time_step_);
    }

    /**
     * @brief Function to update the PID output given the respective errors
     * @param error_p Proportional Error
     * @param error_d Derivative Error
     * @return Returns the Pid output
     */
    VectorNS UpdateWithError(VectorNS error_p, VectorNS error_d, double time_step) {
      // Computing the integral error
      //error_p = time_step;
      
      i_error_ += error_p*time_step; //check

      // Finding the control output from the errors
      output_ = kp_.array() * error_p.array() + kd_.array() * error_d.array() + ki_.array() * i_error_.array();

      // Checking if in Deadband range
      
      
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
    double deadband_;

    /**
     * @brief Max control output
     */
    std::array<double, 2> saturation_{ -INFINITY, INFINITY };

    /**
     * @brief Output of the function used internally
     */
    VectorNS output_ ;

    

    

  private:
    static constexpr int kSaturationLowerLimitIndex = 0;
    static constexpr int kSaturationUpperLimitIndex = 1;

  };

}  // namespace kodlab