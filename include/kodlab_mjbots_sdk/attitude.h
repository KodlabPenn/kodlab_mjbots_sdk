/**
 * @file attitude.h
 * @author Ethan Musser (emusser@seas.upenn.edu)
 * @brief Class for storing and converting IMU data
 * @version 0.1
 * @date 2022-06-21
 *
 * @copyright Copyright (c) 2022 The Trustees of the University of Pennsylvania. All Rights Reserved
 *
 */

#pragma once

#include "kodlab_mjbots_sdk/rotations.h"
#include "kodlab_mjbots_sdk/math.h"
#include <Eigen/Geometry>

namespace kodlab
{

  /**
   * @brief Attitude object for inertial measurement unit data.
   * @note  Derivative classes must be used to explicitly instantiate templates
   *        of \c MjbotsRobotInterface in \c mjbots_robot_interface.cpp.
   *
   * @tparam Scalar[optional] numeric type
   */
  template <typename Scalar = float>
  class Attitude
  {
    static_assert(std::is_arithmetic<Scalar>::value, "Attitude data must have a numeric type.");

  public:
    /**
     * @brief Attitude quaternion
     *
     */
    Eigen::Quaternion<Scalar> attitude;

    /**
     * @brief Angular velocity about [x, y, z] axes (default deg/s)
     *
     */
    Eigen::Vector3<Scalar> ang_rate;

    /**
     * @brief Linear acceleration along [x, y, z] axes (default m/s^2)
     *
     */
    Eigen::Vector3<Scalar> accel;

    /**
     * @brief Gyroscope bias about [x, y, z] axes (default deg/s)
     *
     */
    Eigen::Vector3<Scalar> ang_rate_bias;

    /**
     * @brief Attitude uncertainty
     *
     */
    Eigen::Quaternion<Scalar> attitude_uncertainty;

    /**
     * @brief Gyroscope bias uncertainty about [x, y, z] axes (default deg/s)
     *
     */
    Eigen::Vector3<Scalar> ang_bias_uncertainty;

    /**
     * @brief
     *
     */
    Eigen::Quaternion<Scalar> world_offset = {1, 0, 0, 0};

    /**
     * @brief Construct a new \c Attitude object.
     *
     */
    Attitude() {}

    /**
     * @brief Construct a new \c Attitude object with populated data.
     *
     * @param attitude_in attitude quaternion
     * @param ang_rate_in angular velocity about [x, y, z] axes
     * @param accel_in linear acceleration along [x, y, z] axes
     */
    Attitude(const Eigen::Quaternion<Scalar> &attitude_in,
             const Eigen::Vector3<Scalar> &ang_rate_in,
             const Eigen::Vector3<Scalar> &accel_in)
        : attitude(attitude_in),
          ang_rate(ang_rate_in),
          accel(accel_in) {}

    /**
     * @brief Construct a new \c Attitude object with populated data, biases,
     *        and uncertainty.
     *
     * @param attitude_in attitude quaternion
     * @param ang_rate_in angular velocity about [x, y, z] axes
     * @param accel_in linear acceleration along [x, y, z] axes
     * @param ang_bias_in gyroscope bias about [x, y, z] axes
     * @param attitude_uncertainty_in attitude uncertainty quaternion
     * @param ang_bias_uncertainty_in gyroscope bias uncertainty about [x, y, z]
     *                                axes
     */
    Attitude(const Eigen::Quaternion<Scalar> &attitude_in,
             const Eigen::Vector3<Scalar> &ang_rate_in,
             const Eigen::Vector3<Scalar> &accel_in,
             const Eigen::Vector3<Scalar> &ang_bias_in,
             const Eigen::Quaternion<Scalar> &attitude_uncertainty_in,
             const Eigen::Vector3<Scalar> &ang_bias_uncertainty_in)
        : attitude(attitude_in),
          ang_rate(ang_rate_in),
          accel(accel_in),
          ang_rate_bias(ang_bias_in),
          attitude_uncertainty(attitude_uncertainty_in),
          ang_bias_uncertainty(ang_bias_uncertainty_in) {}

    /**
     * @brief Sets class member data for all attitude representations using
     *        \c attitude member data.
     * @details Computes and sets class member data for all attitude
     *          representations using data in class member \c attitude.
     *
     */
    virtual void SetAttitude() {}

    /**
     * @brief Sets class member data for all attitude representations
     *
     * @param attitude_in attitude quaternion
     */
    virtual void SetAttitude(const Eigen::Quaternion<Scalar> &attitude_in)
    {
      attitude = attitude_in;
    }
  };

  /****************************************************************************/
  /* Attitude Derivative Classes                                              */
  /****************************************************************************/

  /**
   * @brief
   *
   * @tparam Scalar[optional] numeric type
   */
  template <typename Scalar = float>
  class AttitudeWithEulerAng : public Attitude<Scalar>
  {
  public:
    /**
     * @brief Attitude X-Y-Z Extrinsic Euler angle representation
     *
     */
    kodlab::rotations::EulerAngles<Scalar> euler;

    /**
     * @brief
     *
     */
    void SetAttitude() override
    {
      euler = kodlab::rotations::QuaternionToEulerAngles(this->attitude);
    }

    /**
     * @brief Set the Attitude object
     *
     * @param attitude_in
     */
    void SetAttitude(const Eigen::Quaternion<Scalar> &attitude_in) override
    {
      this->attitude = attitude_in;
      SetAttitude();
    }
  };

  /**
   * @brief
   *
   * @tparam Scalar[optional] numeric type
   */
  template <typename Scalar = float>
  class AttitudeWithRotMat : public Attitude<Scalar>
  {
  public:
    /**
     * @brief
     *
     */
    Eigen::Matrix3<Scalar> rot_mat;

    /**
     * @brief
     *
     */
    void SetAttitude() override
    {
      rot_mat = kodlab::rotations::QuaternionToRotationMatrix(this->attitude); // FIXME: Assumes that this->attitude is unit quaternion.
    }

    /**
     * @brief Set the Attitude object
     *
     * @param attitude_in
     */
    void SetAttitude(const Eigen::Quaternion<Scalar> &attitude_in) override
    {
      this->attitude = attitude_in;
      SetAttitude();
    }
  };

} // namespace kodlab
