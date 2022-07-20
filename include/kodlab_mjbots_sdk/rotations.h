/**
 * @file rotations.h
 * @author Ethan Musser (emusser@seas.upenn.edu)
 * @brief 
 * @version 
 * @date 7/8/22
 * 
 * @copyright 2022 The Trustees of the University of Pennsylvania. All rights reserved.
 * 
 */

#pragma once

#include <Eigen/Geometry>
#include <type_traits>
#include <array>
#include <algorithm>

namespace kodlab::rotations
{

///////////////////////////////////////////////////////////////////////////////
// Euler Angles Representation                                               //
///////////////////////////////////////////////////////////////////////////////

enum EulerRotationType
{
  Extrinsic = 0,
  Intrinsic
};

const std::array<int, 3> DEFAULT_EULER_ANGLE_AXES = {0, 1, 2};
const EulerRotationType DEFAULT_EULER_ANGLE_ROTATION_TYPE = Extrinsic;

/**
 * @brief Euler angle rotation object.
 * @note Default convention is <b>X-Y-Z Extrinsic</b> Euler angles.
 *
 * @tparam Scalar[optional] numeric type
 */
template<typename Scalar = float>
struct EulerAngles
{
public:
  static_assert(std::is_arithmetic<Scalar>::value,
                "Euler angles must have a numeric type.");

  /**
   * @brief Euler rotation angles [alpha, beta, gamma]
   */
  std::array<Scalar, 3> angles = {0, 0, 0};

  /**
   * @brief Euler rotation axes.
   * @details The axes about which the Euler rotations are done. Default is
   * the <b>X-Y-Z Extrinsic (Tait-Bryan)</b> Euler angle convention.
   */
  std::array<int, 3> axes = DEFAULT_EULER_ANGLE_AXES;

  /**
   * @brief Euler frame rotation convention.
   * @details The Euler frame rotation convention, either \c Extrinsic (static
   * frame) or \c Intrinsic (rotating frame).
   *
   */
  EulerRotationType rot_type = DEFAULT_EULER_ANGLE_ROTATION_TYPE;

  /**
   * @brief Construct an Euler Angles object with default values and
   * convention.
   */
  EulerAngles() = default;

  /**
   * @brief Construct an Euler Angles object with default convention.
   *
   * @param angles_in array of alpha, beta, and gamma angles
   */
  explicit EulerAngles(const std::array<Scalar, 3> &angles_in)
      : angles(angles_in) {}

  /**
   * @brief Construct an Euler Angles object with default convention.
   *
   * @param angles_in array of alpha, beta, and gamma angles
   */
  explicit EulerAngles(const Eigen::Vector3<Scalar> &angles_in)
      : angles({angles_in(0), angles_in(1), angles_in(2)}) {}

  /**
   * @brief Construct an Euler Angles object with default convention.
   *
   * @param alpha_in alpha angle
   * @param beta_in beta angle
   * @param gamma_in gamma angle
   */
  EulerAngles(const Scalar &alpha_in,
              const Scalar &beta_in,
              const Scalar &gamma_in)
      : angles({alpha_in, beta_in, gamma_in}) {}

  /**
   * @brief Construct an Euler Angles object.
   *
   * @param alpha_in alpha angle
   * @param beta_in beta angle
   * @param gamma_in gamma angle
   * @param axes_in array of rotation axes identifiers
   * @param rot_type_in rotation type (`Extrinsic` or `Intrinsic`)
   */
  EulerAngles(const Scalar &alpha_in,
              const Scalar &beta_in,
              const Scalar &gamma_in,
              const std::array<int, 3> &axes_in,
              const EulerRotationType &rot_type_in)
      : angles({alpha_in, beta_in, gamma_in}), 
        axes(axes_in),
        rot_type(rot_type_in) {}

  /**
   * @brief Construct an Euler Angles object.
   *
   * @param angles_in array of alpha, beta, and gamma angles
   * @param axes_in array of rotation axes identifiers
   * @param rot_type_in rotation type (Extrinsic or Intrinsic)
   */
  EulerAngles(const std::array<Scalar, 3> &angles_in,
              const std::array<int, 3> &axes_in,
              const EulerRotationType &rot_type_in)
      : angles(angles_in), 
        axes(axes_in),
        rot_type(rot_type_in) {}

  /**
   * @brief Construct an Euler Angles object.
   *
   * @param angles_in array of alpha, beta, and gamma angles
   * @param axes_in array of rotation axes identifiers
   * @param rot_type_in rotation type (Extrinsic or Intrinsic)
   */
  EulerAngles(const Eigen::Vector3<Scalar> &angles_in,
              const std::array<int, 3> &axes_in,
              const EulerRotationType &rot_type_in)
      : angles({angles_in(0), angles_in(1), angles_in(2)}), 
        axes(axes_in),
        rot_type(rot_type_in) {}

  /**
   * @brief Returns quaternion representation of this rotation.
   *
   * @return quaternion
   */
  Eigen::Quaternion<Scalar> ToQuaternion() const
  {
    return EulerAnglesToQuaternion(*this);
  }

  /**
   * @brief Returns 3x3 rotation matrix representation of this rotation.
   *
   * @return 3x3 rotation matrix
   */
  Eigen::Matrix3<Scalar> ToRotationMatrix() const
  {
    return EulerAnglesToRotationMatrix(*this);
  }

  /**
   * @brief Returns roll angle if using a [Tait-Bryan convention]
   *        (https://en.wikipedia.org/wiki/Davenport_chained_rotations)
   * @note Returned value is incorrect if convention is not Tait-Bryan (e.g. if 
   *       using Pure Euler convention)
   * 
   * @return const Scalar 
   */
  const Scalar roll() const { return angles[tait_bryan_rpy_map_[0]]; }

  /**
   * @brief Returns pitch angle if using a [Tait-Bryan convention]
   *        (https://en.wikipedia.org/wiki/Davenport_chained_rotations)
   * @note Returned value is incorrect if convention is not Tait-Bryan (e.g. if 
   *       using Pure Euler convention)
   * 
   * @return const Scalar 
   */
  const Scalar pitch() const { return angles[tait_bryan_rpy_map_[1]]; }

  /**
   * @brief Returns yaw angle if using a [Tait-Bryan convention]
   *        (https://en.wikipedia.org/wiki/Davenport_chained_rotations)
   * @note Returned value is incorrect if convention is not Tait-Bryan (e.g. if 
   *       using Pure Euler convention)
   * 
   * @return const Scalar 
   */
  const Scalar yaw() const { return angles[tait_bryan_rpy_map_[2]]; }

private:
  /**
   * @brief Indices map from axes convention to roll-pitch-yaw angles
   */
  std::array<int, 3> tait_bryan_rpy_map_ = {0, 1, 2};

  /**
   * @brief Computes indices map from axes convention to roll-pitch-yaw angles.
   * @link https://en.wikipedia.org/wiki/Davenport_chained_rotations @endlink
   * 
   * @param axes array of Tait-Bryan a
   */
  void ComputeAngleIndexMap(const std::array<int, 3> &axes)
  {
    for (unsigned int i = 0; i < axes.size(); i++)
    {
      tait_bryan_rpy_map_[axes[i]] = i;
    }
  }

  /**
   * @brief Get the axes in extrinsic order
   * 
   * @return std::array<int,3> 
   */
  std::array<int,3> get_axes_extrinsic()
  {
    std::array<int,3> axes_out(axes);
    if(rot_type==INTRINSIC){
        std::reverse(axes_out.begin(),axes_out.end());
    }
    return axes_out;
  }
  
  /**
   * @brief Get the angles in extrinsic order
   * 
   * @return std::array<float,3> 
   */
  std::array<float,3> get_angles_extrinsic()
  {
    std::array<float,3> angles_out(angles_out);
    if(rot_type==INTRINSIC){
        std::reverse(angles_out.begin(),angles_out.end());
    }
    return angles_out;
  }
    
   


};

/**
 * @brief Static assertion that \c EulerAngle object follows X-Y-Z Extrinsic
 *        Euler angle convention.
 *
 * @tparam Scalar numeric type
 * @param euler Euler angle object
 */
template<typename Scalar>
void ASSERT_DEFAULT_EULER_CONVENTION(const EulerAngles<Scalar> &euler)
{
  if (!((euler.axes == DEFAULT_EULER_ANGLE_AXES) &&
      (euler.rot_type == DEFAULT_EULER_ANGLE_ROTATION_TYPE)))
  {
    std::cerr << "Method only supports X-Y-Z Extrinsic Euler angle convention."
              << std::endl;
  }
}

///////////////////////////////////////////////////////////////////////////////
// Rotation Representation Conversions                                       //
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Returns a quaternion representation of an \c EulerAngles object
 * @note See the cited Drake code for details
 * @cite Russ Tedrake and the Drake Development Team, "Drake: Model-based design
 *       and verification for robotics" (2019)
 * @link https://github.com/RobotLocomotion/drake/blob/5022e3832e90f988f1b6d3015b2794ea26e527b1/math/roll_pitch_yaw.h#L159 @endlink
 *
 * @tparam Scalar[optional] numeric type (default float)
 * @param euler
 * @return Eigen::Quaternion<Scalar>
 */
template<typename Scalar = float>
inline Eigen::Quaternion<Scalar> EulerAnglesToQuaternion(
    const EulerAngles<Scalar> &euler)
{
  ASSERT_DEFAULT_EULER_CONVENTION(euler);

  using std::cos;
  using std::sin;
  const Scalar q0Half = euler.roll / 2;
  const Scalar q1Half = euler.pitch / 2;
  const Scalar q2Half = euler.yaw / 2;
  const Scalar c0 = cos(q0Half), s0 = sin(q0Half);
  const Scalar c1 = cos(q1Half), s1 = sin(q1Half);
  const Scalar c2 = cos(q2Half), s2 = sin(q2Half);
  const Scalar c1_c2 = c1 * c2, s1_c2 = s1 * c2;
  const Scalar s1_s2 = s1 * s2, c1_s2 = c1 * s2;
  const Scalar w = c0 * c1_c2 + s0 * s1_s2;
  const Scalar x = s0 * c1_c2 - c0 * s1_s2;
  const Scalar y = c0 * s1_c2 + s0 * c1_s2;
  const Scalar z = c0 * c1_s2 - s0 * s1_c2;

  return Eigen::Quaternion<Scalar>(w, x, y, z);
}

/**
 * @brief Computes a rotation matrix from a normalized quaternion
 * @note See the cited Drake code for details
 * @cite Russ Tedrake and the Drake Development Team, "Drake: Model-based design
 *       and verification for robotics" (2019)
 * @link https://github.com/RobotLocomotion/drake/blob/5022e3832e90f988f1b6d3015b2794ea26e527b1/math/rotation_matrix.h#L146 @endlink
 *
 * @tparam Scalar[optional] numeric type (default float)
 * @param euler
 * @return Eigen::Matrix3<Scalar>
 */
template<typename Scalar = float>
inline Eigen::Matrix3<Scalar> EulerAnglesToRotationMatrix(
    const EulerAngles<Scalar> &euler)
{
  ASSERT_DEFAULT_EULER_CONVENTION(euler);

  const Scalar &r = euler.roll;
  const Scalar &p = euler.pitch;
  const Scalar &y = euler.yaw;
  using std::cos;
  using std::sin;
  const Scalar c0 = cos(r), c1 = cos(p), c2 = cos(y);
  const Scalar s0 = sin(r), s1 = sin(p), s2 = sin(y);
  const Scalar c2_s1 = c2 * s1, s2_s1 = s2 * s1;

  Eigen::Matrix3<Scalar> m;
  m.coeffRef(0, 0) = c2 * c1;
  m.coeffRef(0, 1) = c2_s1 * s0 - s2 * c0;
  m.coeffRef(0, 2) = c2_s1 * c0 + s2 * s0;
  m.coeffRef(1, 0) = s2 * c1;
  m.coeffRef(1, 1) = s2_s1 * s0 + c2 * c0;
  m.coeffRef(1, 2) = s2_s1 * c0 - c2 * s0;
  m.coeffRef(2, 0) = -s1;
  m.coeffRef(2, 1) = c1 * s0;
  m.coeffRef(2, 2) = c1 * c0;

  return m;
}

/**
 * @brief Constructs a 3x3 rotation matrix from a normalized quaternion.
 * @note See the cited Drake code for details
 * @cite Russ Tedrake and the Drake Development Team, "Drake: Model-based design
 *       and verification for robotics" (2019)
 * @link https://github.com/RobotLocomotion/drake/blob/5022e3832e90f988f1b6d3015b2794ea26e527b1/math/rotation_matrix.h#L970 @endlink
 *
 * @tparam Scalar[optional] numeric type (default float)
 * @param quaternion
 * @param two_over_norm_squared
 * @return Eigen::Matrix3<Scalar>
 */
template<typename Scalar = float>
inline Eigen::Matrix3<Scalar> QuaternionToRotationMatrix(
    const Eigen::Quaternion<Scalar> &quaternion,
    const Scalar &two_over_norm_squared)
{
  const Scalar w = quaternion.w();
  const Scalar x = quaternion.x();
  const Scalar y = quaternion.y();
  const Scalar z = quaternion.z();
  const Scalar sx = two_over_norm_squared * x; // scaled x-value.
  const Scalar sy = two_over_norm_squared * y; // scaled y-value.
  const Scalar sz = two_over_norm_squared * z; // scaled z-value.
  const Scalar swx = sx * w;
  const Scalar swy = sy * w;
  const Scalar swz = sz * w;
  const Scalar sxx = sx * x;
  const Scalar sxy = sy * x;
  const Scalar sxz = sz * x;
  const Scalar syy = sy * y;
  const Scalar syz = sz * y;
  const Scalar szz = sz * z;

  Eigen::Matrix3<Scalar> m;
  m.coeffRef(0, 0) = Scalar(1) - syy - szz;
  m.coeffRef(0, 1) = sxy - swz;
  m.coeffRef(0, 2) = sxz + swy;
  m.coeffRef(1, 0) = sxy + swz;
  m.coeffRef(1, 1) = Scalar(1) - sxx - szz;
  m.coeffRef(1, 2) = syz - swx;
  m.coeffRef(2, 0) = sxz - swy;
  m.coeffRef(2, 1) = syz + swx;
  m.coeffRef(2, 2) = Scalar(1) - sxx - syy;

  return m;
}

/**
 * @brief Constructs a 3x3 rotation matrix from a normalized quaternion.
 * @note See the cited Drake code for details
 * @cite Russ Tedrake and the Drake Development Team, "Drake: Model-based design
 *       and verification for robotics" (2019)
 * @link https://github.com/RobotLocomotion/drake/blob/5022e3832e90f988f1b6d3015b2794ea26e527b1/math/rotation_matrix.h#L970 @endlink
 *
 * @tparam Scalar[optional] numeric type (default float)
 * @param unit_quaternion
 * @return Eigen::Matrix3<Scalar>
 */
template<typename Scalar = float>
inline Eigen::Matrix3<Scalar> QuaternionToRotationMatrix(
    const Eigen::Quaternion<Scalar> &unit_quaternion)
{
  return QuaternionToRotationMatrix(unit_quaternion, Scalar(2));
}

/**
 * @brief Constructs an euler angle representation from a normalized quaternion
 *        and a rotation matrix
 * @note See the cited Drake code for details
 * @cite Russ Tedrake and the Drake Development Team, "Drake: Model-based design
 *       and verification for robotics" (2019)
 * @link https://github.com/RobotLocomotion/drake/blob/5022e3832e90f988f1b6d3015b2794ea26e527b1/math/roll_pitch_yaw.cc#L112 @endlink
 *
 * @tparam Scalar[optional] numeric type (default float)
 * @param quaternion
 * @param R
 * @return EulerAngles<Scalar>
 */
template<typename Scalar = float>
EulerAngles<Scalar> QuaternionAndRotationMatrixToEulerAngles(
    const Eigen::Quaternion<Scalar> &quaternion,
    const Eigen::Matrix3<Scalar> &R)
{
  using std::abs;
  using std::atan2;
  using std::sqrt;

  // Calculate q2 using lots of information in the rotation matrix.
  // Rsum = abs( cos(q2) ) is inherently non-negative.
  // R20 = -sin(q2) may be negative, zero, or positive.
  const Scalar R22 = R(2, 2);
  const Scalar R21 = R(2, 1);
  const Scalar R10 = R(1, 0);
  const Scalar R00 = R(0, 0);
  const Scalar Rsum = sqrt((R22 * R22 + R21 * R21 + R10 * R10 + R00 * R00) / 2);
  const Scalar R20 = R(2, 0);
  const Scalar q2 = atan2(-R20, Rsum);

  // Calculate q1 and q3 from Steps 2-6 (see link).
  const Scalar e0 = quaternion.w(), e1 = quaternion.x();
  const Scalar e2 = quaternion.y(), e3 = quaternion.z();
  const Scalar yA = e1 + e3, xA = e0 - e2;
  const Scalar yB = e3 - e1, xB = e0 + e2;
  const Scalar epsilon = Eigen::NumTraits<Scalar>::epsilon();
  const auto isSingularA = abs(yA) <= epsilon && abs(xA) <= epsilon;
  const auto isSingularB = abs(yB) <= epsilon && abs(xB) <= epsilon;
  const Scalar zA = isSingularA ? Scalar{0.0} : atan2(yA, xA);
  const Scalar zB = isSingularB ? Scalar{0.0} : atan2(yB, xB);
  Scalar q1 = zA - zB; // First angle in rotation sequence.
  Scalar q3 = zA + zB; // Third angle in rotation sequence.

  // If necessary, modify angles q1 and/or q3 to be between -pi and pi.
  q1 = q1 > M_PI ? q1 - 2 * M_PI : q1;
  q1 = q1 < -M_PI ? q1 + 2 * M_PI : q1;
  q3 = q3 > M_PI ? q3 - 2 * M_PI : q3;
  q3 = q3 < -M_PI ? q3 + 2 * M_PI : q3;

  // Return in conventional SpaceXYZ q1, q2, q3 (roll-pitch-yaw) order
  // (which is equivalent to BodyZYX q3, q2, q1 order).
  return EulerAngles<Scalar>(q1, q2, q3);
}

/**
 * @brief Constructs euler angles from a normalized quaternion
 *
 * @tparam Scalar[optional] numeric type (default float)
 * @param unit_quaternion
 * @return EulerAngles<Scalar>
 */
template<typename Scalar = float>
EulerAngles<Scalar> QuaternionToEulerAngles(
    const Eigen::Quaternion<Scalar> &unit_quaternion)
{
  return QuaternionAndRotationMatrixToEulerAngles(
      unit_quaternion, QuaternionToRotationMatrix(unit_quaternion));
}

}

