/**
 * @file imu_example.cpp
 * @author Ethan Musser (emusser@seas.upenn.edu)
 * @brief Basic example script demonstrating how to use the mjbots_control_loop
 *        to read IMU data.
 * @date 7/9/22
 * 
 * @copyright 2022 The Trustees of the University of Pennsylvania. All rights reserved.
 * 
 */

#include <Eigen/Core>
#include <Eigen/Dense>
#include "kodlab_mjbots_sdk/mjbots_control_loop.h"
#include "kodlab_mjbots_sdk/imu_data.h"
#include "IMULog.hpp"

class AttitudeExample : public kodlab::mjbots::MjbotsControlLoop<IMULog,
                                                                 VoidLcm>
{
public:
  using MjbotsControlLoop::MjbotsControlLoop;

private:
  /**
   * @brief Shared pointer to an IMU data object
   * @note This is is a shared pointer to a continually updated `IMUData`
   *       object.  There is no need to update in the control loop itself.
   */
  std::shared_ptr<kodlab::IMUData<float>> imu_ = robot_->GetIMUDataSharedPtr();

  /**
   * @brief Read-only IMU data object
   * @note Unlike `imu_` above which is a shared pointer to a continually
   *       updated `IMUData` object, this object is only updated when
   *       specifically assigned.
   */
  kodlab::IMUData<float> imu_read_only_;

  /**
   * @brief Gathers IMU data and sets command torques to zero
   */
  void CalcTorques() override
  {
    // Update Read-Only IMUData
    imu_read_only_ = robot_->GetIMUData();

    // Read Various IMUData Information from `imu_`
    Eigen::Quaternionf quat = imu_->get_quat();    // attitude quaternion
    Eigen::Matrix3f rot_mat = imu_->get_rot_mat(); // attitude rot. matrix
    Eigen::Vector3f ang_vel = imu_->get_ang_rate();    // angular velocity

    // Read Various IMUData Information from `imu_read_only_`
    kodlab::rotations::EulerAngles<float>
        euler = imu_read_only_.get_euler();           // attitude euler ang.
    Eigen::Vector3f lin_acc = imu_read_only_.get_accel(); // linear acceleration

    // Print `imu_` Data to Terminal
    imu_->PrintIMUData();

    // Set Torques to Zero
    std::vector<float> torques(num_motors_, 0);
    robot_->SetTorques(torques);
  }

  /**
   * @brief Prepares log
   */
  void PrepareLog() override
  {
    // IMUData Quaternion
    log_data_.attitude_quaternion[0] = imu_->get_quat().x();
    log_data_.attitude_quaternion[1] = imu_->get_quat().y();
    log_data_.attitude_quaternion[2] = imu_->get_quat().z();
    log_data_.attitude_quaternion[3] = imu_->get_quat().w();

    // IMUData Euler Angles
    log_data_.attitude_euler[0] = imu_->get_euler().roll();
    log_data_.attitude_euler[1] = imu_->get_euler().pitch();
    log_data_.attitude_euler[2] = imu_->get_euler().yaw();

    // Angular Velocity
    log_data_.angular_velocity[0] = imu_->get_ang_rate().x();
    log_data_.angular_velocity[1] = imu_->get_ang_rate().y();
    log_data_.angular_velocity[2] = imu_->get_ang_rate().z();

    // Linear Acceleration
    log_data_.linear_acceleration[0] = imu_->get_accel().x();
    log_data_.linear_acceleration[1] = imu_->get_accel().y();
    log_data_.linear_acceleration[2] = imu_->get_accel().z();
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

  // Set IMU world offset to be a rotation of 180-deg about the x-axis
  options.imu_world_offset_deg.roll = 180;
  options.imu_world_offset_deg.pitch = 0;
  options.imu_world_offset_deg.yaw = 0;

  // Create Control Loop
  AttitudeExample control_loop(joints, options);

  // Starts the Loop, Then Joins It
  control_loop.Start();
  control_loop.Join();

  return 0;
}


