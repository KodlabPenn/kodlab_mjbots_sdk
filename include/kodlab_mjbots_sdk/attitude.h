/**
 * @file attitude.h
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
 * @brief Attitude object storing inertial data.
 * @tparam Scalar[optional] numeric type
 */
template<typename Scalar = float>
class Attitude
{

private:
  /**
   * @brief Inputted attitude quaternion before world offset rotation
   */
  Eigen::Quaternion<Scalar> att_quat_raw_ = {1, 0, 0, 0};

  /**
   * @brief Attitude quaternion in world frame
   */
  Eigen::Quaternion<Scalar> att_quat_ = {1, 0, 0, 0};

  /**
   * @brief Attitude euler angles in world frame
   * @note Always follow the default <b>Extrinsic X-Y-Z> convention
   */
  mutable kodlab::ValidatedCache<rotations::EulerAngles<Scalar>> att_euler_= {rotations::EulerAngles<Scalar>(0,0,0), false};

  /**
   * @brief Attitude rotation matrix in world frame
   */
  mutable kodlab::ValidatedCache<Eigen::Matrix3<Scalar>> att_rot_mat_ = {Eigen::Matrix3<Scalar>::Identity(), false};;

  /**
   * @brief Angular velocity about <code>[x, y, z]</code> axes (default deg/s)
   */
  Eigen::Vector3<Scalar> ang_rate_ = {0, 0, 0};

  /**
   * @brief Linear acceleration along <code>[x, y, z]</code> axes (default
   *        m/s^2)
   */
  Eigen::Vector3<Scalar> accel_ = {0, 0, 0};

  /**
   * @brief Gyroscope bias about <code>[x, y, z]</code> axes (default deg/s)
   */
  Eigen::Vector3<Scalar> ang_rate_bias_ = {0, 0, 0};

  /**
   * @brief Attitude uncertainty about the raw attitude
   */
  Eigen::Quaternion<Scalar> att_uncertainty_ = {1, 0, 0, 0};

  /**
   * @brief Gyroscope bias uncertainty about <code>[x, y, z]</code> axes
   * (default deg/s)
   */
  Eigen::Vector3<Scalar> ang_bias_uncertainty_ = {0, 0, 0};

  /**
   * @brief World frame rotation offset
   * @note Useful to change the direction of gravity
   */
  Eigen::Quaternion<Scalar> world_offset_ = {1, 0, 0, 0};

  /**
   * @brief Invalidates cached data
   */
  void InvalidateCached()
  {
    att_euler_.invalidate();
    att_rot_mat_.invalidate();
  }

public:
  /**
   * @brief Construct a new \c Attitude object.
   */
  Attitude() = default;

  /**
   * @brief Construct a new \c Attitude object with populated data.
   *
   * @param att_quat_in attitude quaternion
   * @param ang_rate_in angular velocity about <code>[x, y, z]</code> axes
   * @param accel_in linear acceleration along <code>[x, y, z]</code> axes
   * @param world_offset_in world frame rotation offset (default identity)
   */
  Attitude(const Eigen::Quaternion<Scalar> &att_quat_in,
           const Eigen::Vector3<Scalar> &ang_rate_in,
           const Eigen::Vector3<Scalar> &accel_in,
           const Eigen::Quaternion<Scalar> &world_offset_in = {1, 0, 0, 0})
      : att_quat_raw_(att_quat_in),
        att_quat_(world_offset_in * att_quat_in),
        ang_rate_(ang_rate_in),
        accel_(accel_in),
        world_offset_(world_offset_in) {}

  /**
   * @brief Construct a new \c Attitude object with populated data, biases,
   *        and uncertainty.
   *
   * @param att_quat_in attitude quaternion
   * @param ang_rate_in angular velocity about <code>[x, y, z]</code> axes
   * @param accel_in linear acceleration along <code>[x, y, z]</code> axes
   * @param ang_rate_bias_in gyroscope bias about <code>[x, y, z]</code> axes
   * @param att_uncertainty_in attitude uncertainty quaternion
   * @param ang_bias_uncertainty_in gyroscope bias uncertainty about
   *                                <code>[x, y, z]</code> axes
   * @param world_offset_in world frame rotation offset (default identity)
   */
  Attitude(const Eigen::Quaternion<Scalar> &att_quat_in,
           const Eigen::Vector3<Scalar> &ang_rate_in,
           const Eigen::Vector3<Scalar> &accel_in,
           const Eigen::Vector3<Scalar> &ang_rate_bias_in,
           const Eigen::Quaternion<Scalar> &att_uncertainty_in,
           const Eigen::Vector3<Scalar> &ang_bias_uncertainty_in,
           const Eigen::Quaternion<Scalar> &world_offset_in = {1, 0, 0, 0})
      : att_quat_raw_(att_quat_in),
        att_quat_(world_offset_in * att_quat_in),
        ang_rate_(ang_rate_in),
        accel_(accel_in),
        ang_rate_bias_(ang_rate_bias_in),
        att_uncertainty_(att_uncertainty_in),
        ang_bias_uncertainty_(ang_bias_uncertainty_in),
        world_offset_(world_offset_in) {}

  /**
   * @brief Construct a new \c Attitude object from a
   *        \c mjbots::pi3hat::Attitude object
   *
   * @param pi_att attitude object
   * @param world_offset_in world frame rotation offset (default identity)
   */
  Attitude(const ::mjbots::pi3hat::Attitude &pi_att,
           const Eigen::Quaternion<Scalar> &world_offset_in = {1, 0, 0, 0})
      : att_quat_raw_(pi_att.quat.x,
                      pi_att.quat.y,
                      pi_att.quat.z,
                      pi_att.quat.w),
        att_quat_(world_offset_in * att_quat_raw_),
        ang_rate_(pi_att.rate_dps.x, pi_att.rate_dps.y, pi_att.rate_dps.z),
        accel_(pi_att.accel_mps2.x, pi_att.accel_mps2.y, pi_att.accel_mps2.z),
        ang_rate_bias_(pi_att.bias_dps.x, pi_att.bias_dps.y, pi_att.bias_dps.z),
        att_uncertainty_(pi_att.attitude_uncertainty.x,
                         pi_att.attitude_uncertainty.y,
                         pi_att.attitude_uncertainty.z,
                         pi_att.attitude_uncertainty.w),
        ang_bias_uncertainty_(pi_att.bias_uncertainty_dps.x,
                              pi_att.bias_uncertainty_dps.y,
                              pi_att.bias_uncertainty_dps.z),
        world_offset_(world_offset_in) {}

  /**
   * @brief Update this \c Attitude object's data
   *
   * @param att_quat_in attitude quaternion
   * @param ang_rate_in angular velocity about <code>[x, y, z]</code> axes
   * @param accel_in linear acceleration along <code>[x, y, z]</code> axes
   */
  void Update(const Eigen::Quaternion<Scalar> &att_quat_in,
              const Eigen::Vector3<Scalar> &ang_rate_in,
              const Eigen::Vector3<Scalar> &accel_in)
  {
    InvalidateCached();
    att_quat_raw_ = att_quat_in;
    att_quat_ = world_offset_ * att_quat_in;
    ang_rate_ = ang_rate_in;
    accel_ = accel_in;
  }

  /**
   * @brief Update this \c Attitude object's data
   *
   * @param att_quat_in attitude quaternion
   * @param ang_rate_in angular velocity about <code>[x, y, z]</code> axes
   * @param accel_in linear acceleration along <code>[x, y, z]</code> axes
   * @param ang_rate_bias_in gyroscope bias about <code>[x, y, z]</code> axes
   * @param att_uncertainty_in attitude uncertainty quaternion
   * @param ang_bias_uncertainty_in gyroscope bias uncertainty about
   *                                <code>[x, y, z]</code> axes
   */
  void Update(const Eigen::Quaternion<Scalar> &att_quat_in,
              const Eigen::Vector3<Scalar> &ang_rate_in,
              const Eigen::Vector3<Scalar> &accel_in,
              const Eigen::Vector3<Scalar> &ang_rate_bias_in,
              const Eigen::Quaternion<Scalar> &att_uncertainty_in,
              const Eigen::Vector3<Scalar> &ang_bias_uncertainty_in)
  {
    Update(att_quat_in, ang_rate_in, accel_in);
    ang_rate_bias_ = ang_rate_bias_in;
    att_uncertainty_ = att_uncertainty_in;
    ang_bias_uncertainty_ = ang_bias_uncertainty_in;
  }

  /**
   * @brief Update this \c Attitude object's data
   *
   * @param pi_att
   */
  void Update(const ::mjbots::pi3hat::Attitude &pi_att)
  {
    Update({pi_att.quat.w, pi_att.quat.x, pi_att.quat.y, pi_att.quat.z},
           {pi_att.rate_dps.x, pi_att.rate_dps.y, pi_att.rate_dps.z},
           {pi_att.accel_mps2.x, pi_att.accel_mps2.y, pi_att.accel_mps2.z},
           {pi_att.bias_dps.x, pi_att.bias_dps.y, pi_att.bias_dps.z},
           {pi_att.attitude_uncertainty.w, pi_att.attitude_uncertainty.x,
            pi_att.attitude_uncertainty.y, pi_att.attitude_uncertainty.z},
           {pi_att.bias_uncertainty_dps.x, pi_att.bias_uncertainty_dps.y,
            pi_att.bias_uncertainty_dps.z});
  }

  /**
   * @brief Prints attitude, angular velocity, and linear acceleration in
   * various representations
   */
  void PrintAttitude()
  {
    // Gather Attitude Data
    Eigen::Quaternionf qr = get_att_quat_raw();
    Eigen::Quaternionf q = get_att_quat();
    kodlab::rotations::EulerAngles<float> e = get_att_euler();
    Eigen::Vector3f w = get_ang_rate();
    Eigen::Vector3f a = get_accel();
    Eigen::Matrix3f r = get_att_rot_mat();

    // Print Attitude Information to Console
    using std::fprintf;
    std::cout << std::fixed;
    fprintf(stdout,
            "+------------------------ ATTITUDE ----------------------+\n");
    fprintf(stdout,
            "|      Raw Quat:  % 3.2fi + % 3.2fj + % 3.2fk + % 3.2f       |\n",
            qr.x(), qr.y(), qr.z(), qr.w());
    fprintf(stdout,
            "|    Quaternion:  % 3.2fi + % 3.2fj + % 3.2fk + % 3.2f       |\n",
            q.x(), q.y(), q.z(), q.w());
    fprintf(stdout,
            "+--------------------------------------------------------+\n");
    fprintf(stdout,
            "|   Euler (rpy):  ( % 7.2f, % 7.2f, % 7.2f )          |\n",
            e.roll(), e.pitch(), e.yaw());
    fprintf(stdout,
            "| Ang Vel (xyz):  ( % 7.2f, % 7.2f, % 7.2f )          |\n",
            w.x(), w.y(), w.z());
    fprintf(stdout,
            "| Lin Acc (xyz):  ( % 7.2f, % 7.2f, % 7.2f )          |\n",
            a.x(), a.y(), a.z());
    fprintf(stdout,
            "+--------------------------------------------------------+\n");
    fprintf(stdout,
            "|       Rot Mat:  [ % 7.2f, % 7.2f, % 7.2f ]          |\n",
            r(0, 0), r(0, 1), r(0, 2));
    fprintf(stdout,
            "|                 [ % 7.2f, % 7.2f, % 7.2f ]          |\n",
            r(1, 0), r(1, 1), r(1, 2));
    fprintf(stdout,
            "|                 [ % 7.2f, % 7.2f, % 7.2f ]          |\n",
            r(2, 0), r(2, 1), r(2, 2));
    fprintf(stdout,
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
  Eigen::Quaternion<Scalar> get_att_quat_raw() const { return att_quat_raw_; }

  /**
   * @return attitude quaternion in world frame
   */
  Eigen::Quaternion<Scalar> get_att_quat() const { return att_quat_; }

  /**
   * @return attitude rotation matrix in world frame
   */
  Eigen::Matrix3<Scalar> get_att_rot_mat()
  {
    if (!att_rot_mat_.valid())
    {
      att_rot_mat_.set(rotations::QuaternionToRotationMatrix(att_quat_));
    }
    return att_rot_mat_;
  }

  /**
   * @return attitude euler angles in the world frame
   */
  rotations::EulerAngles<Scalar> get_att_euler()
  {
    if (!att_euler_.valid())
    {
      att_euler_.set(rotations::QuaternionAndRotationMatrixToEulerAngles(
          att_quat_,
          get_att_rot_mat()));
    }
    return att_euler_;
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
