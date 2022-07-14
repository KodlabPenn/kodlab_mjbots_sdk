/**
 * @file attitude_example.cpp
 * @author Ethan Musser (emusser@seas.upenn.edu)
 * @brief Basic example script demonstrating how to use the mjbots_control_loop
 *        to read attitude data.
 * @date 7/9/22
 * 
 * @copyright 2022 The Trustees of the University of Pennsylvania. All rights reserved.
 * 
 */

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream> // cout
#include <iomanip> // setprecision, fixed
#include "kodlab_mjbots_sdk/mjbots_control_loop.h"
#include "kodlab_mjbots_sdk/joint_moteus.h"
#include "kodlab_mjbots_sdk/lcm_subscriber.h"
#include "kodlab_mjbots_sdk/attitude.h"
#include "AttitudeLog.hpp"

class AttitudeExample : public kodlab::mjbots::MjbotsControlLoop<AttitudeLog,
                                                                 VoidLcm>
{
public:
  using MjbotsControlLoop::MjbotsControlLoop;

private:
  /**
   * @brief Attitude object to store IMU data
   * @note Uses a modified world offset rotation of \f$\pi\f$ radians about the
   *       roll axis
   */
  kodlab::Attitude<float> att_ = {robot_->GetAttitude(),
                                  Eigen::Quaternionf(Eigen::AngleAxisf(M_PI,
                                                                       Eigen::Vector3f::UnitX()))};

  /**
   * @brief Gathers attitude data and sets command torques to zero
   */
  void CalcTorques() override
  {
    // Get Updated Attitude Representations
    att_.Update(robot_->GetAttitude());
    Eigen::Quaternionf quat = att_.get_att_quat();
    kodlab::rotations::EulerAngles euler = att_.get_att_euler();
    Eigen::Matrix3f rot_mat = att_.get_att_rot_mat();

    // Print Attitude Representations to Terminal
    att_.PrintAttitude();

    // Set Torques to Zero
    std::vector<float> torques(num_motors_, 0);
    robot_->SetTorques(torques);
  }

  /**
   * @brief Prepares log
   */
  void PrepareLog() override
  {
    // Attitude Quaternion
    log_data_.attitude_quaternion[0] = att_.get_att_quat().x();
    log_data_.attitude_quaternion[1] = att_.get_att_quat().y();
    log_data_.attitude_quaternion[2] = att_.get_att_quat().z();
    log_data_.attitude_quaternion[3] = att_.get_att_quat().w();

    // Attitude Euler Angles
    log_data_.attitude_euler[0] = att_.get_att_euler().roll;
    log_data_.attitude_euler[1] = att_.get_att_euler().pitch;
    log_data_.attitude_euler[2] = att_.get_att_euler().yaw;

    // Angular Velocity
    log_data_.angular_velocity[0] = att_.get_ang_rate().x();
    log_data_.angular_velocity[1] = att_.get_ang_rate().y();
    log_data_.angular_velocity[2] = att_.get_ang_rate().z();

    // Linear Acceleration
    log_data_.linear_acceleration[0] = att_.get_accel().x();
    log_data_.linear_acceleration[1] = att_.get_accel().y();
    log_data_.linear_acceleration[2] = att_.get_accel().z();
  }

};

int main(int argc, char **argv)
{
  // Setup Joints
  std::vector<kodlab::mjbots::JointMoteus> joints;
  joints.emplace_back(100, 4, 1, -1.3635165, 1, 1);
  joints.emplace_back(101, 4, -1, 2.688, 5.0 / 3.0, 1);
  joints.emplace_back(108, 4, 1, -0.4674585, 1, 1);

  // Define Robot Options
  kodlab::mjbots::ControlLoopOptions options;
  options.log_channel_name = "attitude_data";
  options.frequency = 10;
  options.realtime_params.main_cpu = 3;
  options.realtime_params.can_cpu = 2;

  // Set IMU Options
  options.imu_mounting_deg.roll = 0;
  options.imu_mounting_deg.pitch = 0;
  options.imu_mounting_deg.yaw = 180;
  options.attitude_rate_hz = 1000;

  // Create Control Loop
  AttitudeExample control_loop(joints, options);

  // Starts the Loop, Then Joins It
  control_loop.Start();
  control_loop.Join();

  return 0;
}


