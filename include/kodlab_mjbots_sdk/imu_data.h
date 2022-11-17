/**
 * @file imu_data.h
 * @author Ethan Musser (emusser@seas.upenn.edu)
 * @brief Defines an attitude object for storing inertial parameters.
 * @date 7/7/22
 * 
 * @copyright 2022 The Trustees of the University of Pennsylvania. All rights reserved.
 * 
 */

#pragma once

#include <Eigen/Geometry>
#include "kodlab_mjbots_sdk/pi3hat.h"
#include "kodlab_mjbots_sdk/common_header.h"
#include "kodlab_mjbots_sdk/rotations.h"
#include <iostream>
#include <iomanip>

namespace kodlab
{

/**
 * @brief IMUData object storing inertial data.
 * @note The body, world, and modified world frames are referred to throughout
 *       this class' documentation. The body frame is fixed to the robot body
 *       and offset from the IMU/Pi3Hat frame based on the Mjbots control loop
 *       options `imu_mounting_deg` parameters. The world frame describes a
 *       fixed inertial frame which is defined at robot startup. This frame
 *       always has \f$\hat{z}\f$ pointing in the direction of gravity. The
 *       modified world frame is also a fixed inertial frame, but is offset from
 *       the world frame by a user-defined `world_offset_`. This is intended
 *       to allow reorientation of the common world frame to something more
 *       intuitive to a given use case, e.g., reorienting gravity to be upwards.
 * @tparam Scalar[optional] numeric type
 */
template<typename Scalar = float>
class IMUData
{

private:
  /**
   * @brief Inputted attitude quaternion before world offset rotation
   */
  Eigen::Quaternion<Scalar> quat_raw_ = {1, 0, 0, 0};

  /**
   * @brief IMUData quaternion in world frame
   */
  Eigen::Quaternion<Scalar> quat_ = {1, 0, 0, 0};

  /**
   * @brief IMUData euler angles in world frame (default rad/s)
   * @note Always follow the default <b>Extrinsic X-Y-Z> convention
   */
  mutable kodlab::ValidatedCache<rotations::EulerAngles<Scalar>>
      euler_ = {rotations::EulerAngles<Scalar>(0, 0, 0), false};

  /**
   * @brief IMUData rotation matrix in world frame
   */
  mutable kodlab::ValidatedCache<Eigen::Matrix3<Scalar>>
      rot_mat_ = {Eigen::Matrix3<Scalar>::Identity(), false};;

  /**
   * @brief Angular velocity about <code>[x, y, z]</code> axes (default rad/s)
   */
  Eigen::Vector3<Scalar> ang_rate_ = {0, 0, 0};

  /**
   * @brief Linear acceleration along <code>[x, y, z]</code> axes (default
   *        m/s^2)
   */
  Eigen::Vector3<Scalar> accel_ = {0, 0, 0};

  /**
   * @brief Gyroscope bias about <code>[x, y, z]</code> axes (default rad/s)
   */
  Eigen::Vector3<Scalar> ang_rate_bias_ = {0, 0, 0};

  /**
   * @brief IMUData uncertainty about the raw attitude
   */
  Eigen::Quaternion<Scalar> att_uncertainty_ = {1, 0, 0, 0};

  /**
   * @brief Gyroscope bias uncertainty about <code>[x, y, z]</code> axes
   * (default rad/s)
   */
  Eigen::Vector3<Scalar> ang_bias_uncertainty_ = {0, 0, 0};

  /**
   * @brief World frame rotation offset
   * @note Useful to change the direction of gravity
   */
  Eigen::Quaternion<Scalar> world_offset_ = {1, 0, 0, 0};

  /**
   * @brief Constant for taking degrees to radians
   */
  static constexpr float kDegsToRads_ = M_PI/180.0;
  
  /**
   * @brief Constant for taking degrees to radians
   */
  static constexpr float kRadsToDegs_ = 180.0/M_PI;

  /**
   * @brief Invalidates cached data
   */
  void InvalidateCache()
  {
    euler_.invalidate();
    rot_mat_.invalidate();
  }

public:
  /**
   * @brief Construct a new \c IMUData object.
   */
  IMUData() = default;

  /**
   * @brief Construct a new \c IMUData object with populated data.
   *
   * @param quat_in attitude quaternion
   * @param ang_rate_in angular velocity [rad/s] about <code>[x, y, z]</code> axes
   * @param accel_in linear acceleration [m/s^2] along <code>[x, y, z]</code> axes
   * @param world_offset_in world frame rotation offset (default identity)
   */
  IMUData(const Eigen::Quaternion<Scalar> &quat_in,
          const Eigen::Vector3<Scalar> &ang_rate_in,
          const Eigen::Vector3<Scalar> &accel_in,
          const Eigen::Quaternion<Scalar> &world_offset_in = {1, 0, 0, 0})
      : quat_raw_(quat_in),
        quat_(world_offset_in * quat_in),
        ang_rate_(ang_rate_in),
        accel_(accel_in),
        world_offset_(world_offset_in) {}

  /**
   * @brief Construct a new \c IMUData object with populated data, biases,
   *        and uncertainty.
   *
   * @param quat_in attitude quaternion
   * @param ang_rate_in angular velocity [rad/s] about <code>[x, y, z]</code> axes
   * @param accel_in linear acceleration [m/s^2] along <code>[x, y, z]</code> axes
   * @param ang_rate_bias_in gyroscope bias [rad/s] about <code>[x, y, z]</code> axes
   * @param att_uncertainty_in attitude uncertainty quaternion
   * @param ang_bias_uncertainty_in gyroscope [rad/s] bias uncertainty about
   *                                <code>[x, y, z]</code> axes
   * @param world_offset_in world frame rotation offset (default identity)
   */
  IMUData(const Eigen::Quaternion<Scalar> &quat_in,
          const Eigen::Vector3<Scalar> &ang_rate_in,
          const Eigen::Vector3<Scalar> &accel_in,
          const Eigen::Vector3<Scalar> &ang_rate_bias_in,
          const Eigen::Quaternion<Scalar> &att_uncertainty_in,
          const Eigen::Vector3<Scalar> &ang_bias_uncertainty_in,
          const Eigen::Quaternion<Scalar> &world_offset_in = {1, 0, 0, 0})
      : quat_raw_(quat_in),
        quat_(world_offset_in * quat_in),
        ang_rate_(ang_rate_in),
        accel_(accel_in),
        ang_rate_bias_(ang_rate_bias_in),
        att_uncertainty_(att_uncertainty_in),
        ang_bias_uncertainty_(ang_bias_uncertainty_in),
        world_offset_(world_offset_in) {}

  /**
   * @brief Construct a new \c IMUData object from a
   *        \c mjbots::pi3hat::IMUData object
   *
   * @param pi_att attitude object
   * @param world_offset_in world frame rotation offset (default identity)
   */
  IMUData(const ::mjbots::pi3hat::Attitude &pi_att,
          const Eigen::Quaternion<Scalar> &world_offset_in = {1, 0, 0, 0})
      : quat_raw_(pi_att.quat.x,
                  pi_att.quat.y,
                  pi_att.quat.z,
                  pi_att.quat.w),
        quat_(world_offset_in * quat_raw_),
        ang_rate_(pi_att.rate_dps.x * kDegsToRads_, 
                  pi_att.rate_dps.y * kDegsToRads_,
                  pi_att.rate_dps.z * kDegsToRads_),
        accel_(pi_att.accel_mps2.x, pi_att.accel_mps2.y, pi_att.accel_mps2.z),
        ang_rate_bias_(pi_att.bias_dps.x * kDegsToRads_,
                       pi_att.bias_dps.y * kDegsToRads_, 
                       pi_att.bias_dps.z * kDegsToRads_),
        att_uncertainty_(pi_att.attitude_uncertainty.x,
                         pi_att.attitude_uncertainty.y,
                         pi_att.attitude_uncertainty.z,
                         pi_att.attitude_uncertainty.w),
        ang_bias_uncertainty_(pi_att.bias_uncertainty_dps.x * kDegsToRads_,
                              pi_att.bias_uncertainty_dps.y * kDegsToRads_,
                              pi_att.bias_uncertainty_dps.z * kDegsToRads_),
        world_offset_(world_offset_in) {}

  /**
   * @brief Update this \c IMUData object's data
   *
   * @param quat_in attitude quaternion
   * @param ang_rate_in angular velocity [rad/s] about <code>[x, y, z]</code> axes
   * @param accel_in linear acceleration [m/s^2] along <code>[x, y, z]</code> axes
   */
  void Update(const Eigen::Quaternion<Scalar> &quat_in,
              const Eigen::Vector3<Scalar> &ang_rate_in,
              const Eigen::Vector3<Scalar> &accel_in)
  {
    InvalidateCache();
    quat_raw_ = quat_in;
    quat_ = world_offset_ * quat_in;
    ang_rate_ = ang_rate_in;
    accel_ = accel_in;
  }

  /**
   * @brief Update this \c IMUData object's data
   *
   * @param quat_in attitude quaternion
   * @param ang_rate_in angular velocity [rad/s] about <code>[x, y, z]</code> axes
   * @param accel_in linear acceleration [m/s^2] along <code>[x, y, z]</code> axes
   * @param ang_rate_bias_in gyroscope bias [rad/s] about <code>[x, y, z]</code> axes
   * @param att_uncertainty_in attitude uncertainty quaternion
   * @param ang_bias_uncertainty_in gyroscope bias [rad/s] uncertainty about
   *                                <code>[x, y, z]</code> axes
   */
  void Update(const Eigen::Quaternion<Scalar> &quat_in,
              const Eigen::Vector3<Scalar> &ang_rate_in,
              const Eigen::Vector3<Scalar> &accel_in,
              const Eigen::Vector3<Scalar> &ang_rate_bias_in,
              const Eigen::Quaternion<Scalar> &att_uncertainty_in,
              const Eigen::Vector3<Scalar> &ang_bias_uncertainty_in)
  {
    Update(quat_in, ang_rate_in, accel_in);
    ang_rate_bias_ = ang_rate_bias_in;
    att_uncertainty_ = att_uncertainty_in;
    ang_bias_uncertainty_ = ang_bias_uncertainty_in;
  }

  /**
   * @brief Update this \c IMUData object's data
   *
   * @param pi_att
   */
  void Update(const ::mjbots::pi3hat::Attitude &pi_att) {
    Update({pi_att.quat.w, pi_att.quat.x, pi_att.quat.y, pi_att.quat.z},
           {pi_att.rate_dps.x * kDegsToRads_, pi_att.rate_dps.y * kDegsToRads_,
            pi_att.rate_dps.z * kDegsToRads_},
           {pi_att.accel_mps2.x, pi_att.accel_mps2.y, pi_att.accel_mps2.z},
           {pi_att.bias_dps.x * kDegsToRads_, pi_att.bias_dps.y * kDegsToRads_,
            pi_att.bias_dps.z * kDegsToRads_},
           {pi_att.attitude_uncertainty.w, pi_att.attitude_uncertainty.x,
            pi_att.attitude_uncertainty.y, pi_att.attitude_uncertainty.z},
           {pi_att.bias_uncertainty_dps.x * kDegsToRads_,
            pi_att.bias_uncertainty_dps.y * kDegsToRads_,
            pi_att.bias_uncertainty_dps.z * kDegsToRads_});
  }

  /**
   * @brief Prints attitude, angular velocity, and linear acceleration in
   * various representations
   * @param stream[optional] output stream
   */
  void PrintIMUData(FILE* stream = stdout)
  {
    // Gather IMUData Data
    Eigen::Quaternionf qr = get_quat_raw();
    Eigen::Quaternionf q = get_quat();
    kodlab::rotations::EulerAngles<float> e = get_euler();
    Eigen::Vector3f w = get_ang_rate();
    Eigen::Vector3f a = get_accel();
    Eigen::Matrix3f r = get_rot_mat();

    // Print IMUData Information to Console
    using std::fprintf;
    fprintf(stream,
            "+------------------------ ATTITUDE ----------------------+\n");
    fprintf(stream,
            "|      Raw Quat:  % 3.2fi + % 3.2fj + % 3.2fk + % 3.2f       |\n",
            qr.x(), qr.y(), qr.z(), qr.w());
    fprintf(stream,
            "|    Quaternion:  % 3.2fi + % 3.2fj + % 3.2fk + % 3.2f       |\n",
            q.x(), q.y(), q.z(), q.w());
    fprintf(stream,
            "+--------------------------------------------------------+\n");
    fprintf(stream,
            "|   Euler (rpy):  ( % 7.2f, % 7.2f, % 7.2f )          |\n",
            e.roll(), e.pitch(), e.yaw());
    fprintf(stream,
            "| Ang Vel (xyz):  ( % 7.2f, % 7.2f, % 7.2f )          |\n",
            w.x(), w.y(), w.z());
    fprintf(stream,
            "| Lin Acc (xyz):  ( % 7.2f, % 7.2f, % 7.2f )          |\n",
            a.x(), a.y(), a.z());
    fprintf(stream,
            "+--------------------------------------------------------+\n");
    fprintf(stream,
            "|       Rot Mat:  [ % 7.2f, % 7.2f, % 7.2f ]          |\n",
            r(0, 0), r(0, 1), r(0, 2));
    fprintf(stream,
            "|                 [ % 7.2f, % 7.2f, % 7.2f ]          |\n",
            r(1, 0), r(1, 1), r(1, 2));
    fprintf(stream,
            "|                 [ % 7.2f, % 7.2f, % 7.2f ]          |\n",
            r(2, 0), r(2, 1), r(2, 2));
    fprintf(stream,
            "+--------------------------------------------------------+\n");
  }

  /**
   * @param world_offset_in world offset rotation
   */
  void set_world_offset(Eigen::Quaternion<Scalar> world_offset_in)
  {
    world_offset_ = world_offset_in;
  }

  /**
   * @return attitude quaternion in the unmodified world frame (i.e., not yet
   *         offset)
   */
  Eigen::Quaternion<Scalar> get_quat_raw() const { return quat_raw_; }

  /**
   * @return attitude quaternion in world frame
   */
  Eigen::Quaternion<Scalar> get_quat() const { return quat_; }

  /**
   * @return attitude rotation matrix in world frame
   */
  Eigen::Matrix3<Scalar> get_rot_mat() const
  {
    if (!rot_mat_.valid())
    {
      rot_mat_.set(rotations::QuaternionToRotationMatrix(quat_));
    }
    return rot_mat_;
  }

  /**
   * @return attitude euler angles in the world frame
   */
  rotations::EulerAngles<Scalar> get_euler() const
  {
    if (!euler_.valid())
    {
      euler_.set(rotations::QuaternionAndRotationMatrixToDefaultEulerAngles(
          quat_,
          get_rot_mat()));
    }
    return euler_;
  }

  /**
   * @return angular rate in the world frame
   */
  Eigen::Vector3<Scalar> get_ang_rate() const { return ang_rate_; }

  /**
   * @return linear acceleration in the world frame
   */
  Eigen::Vector3<Scalar> get_accel() const { return accel_; }

  /**
   * @return angular rate bias in the world frame
   */
  Eigen::Vector3<Scalar> get_ang_rate_bias() const { return ang_rate_bias_; }

  /**
   * @return attitude uncertainty quaternion
   */
  Eigen::Quaternion<Scalar> get_att_uncertainty() const { return att_uncertainty_; }

  /**
   * @return angular bias uncertainty
   */
  Eigen::Vector3<Scalar> get_ang_bias_uncertainty() const { return ang_bias_uncertainty_; }

  /**
   * @return world offset quaternion
   */
  Eigen::Quaternion<Scalar> get_world_offset() const { return world_offset_; }

};

} // kodlab
