/**
 * @file quadruped_3dof_limb.h
 * @author Kodlab - J. Diego Caporale (jdcap@seas.upenn.edu)
 * @brief A LimbBase derived class example of the common quadrupedal serial limb
 * configuration with a point toe
 * @details 3-DoF limb implementation demonstrating the LimbBase class
 * @date 2022-09-02
 *
 * @copyright (c) Copyright 2022 The Trustees of the University of Pennsylvania
 *
 */

#pragma once

#include <Eigen/Dense>

#include "kodlab_mjbots_sdk/limb_base.h"

namespace quad_example_helpers{
    
/**
 * @brief Creates a 3x3 skew symmetric matrix from a 3-vector
 *
 * @param w 3-vector
 * @return 3x3 skew-symmetric matrix
 */
Eigen::Matrix3f Skew(const Eigen::Vector3f &w)
{
    Eigen::Matrix3f S;
    S <<    0, -w(2),  w(1),
         w(2),     0, -w(0),
        -w(1),  w(0),     0;
    return S;
}

/**
 * @brief Get the sign of a flag for use in mathematic equations
 *
 * @param flag boolean flag
 * @return 1 if `flag` evaluates to true, -1 otherwise
 */
static int GetSign(bool flag) { return (flag ? 1 : -1); }

/**
     * @brief Solving the equation a0 Sin(th) + a1 Cos(th) = a2
     *
     * @param a0 Sin coefficient
     * @param a1 Cos coefficient
     * @param a2 Right side coefficient
     * @return float returns nan if no solution exists
*/
float SolveSinCosSum(float a0, float a1, float a2)
{
    // a0 Sin(x) + a1 Cos(x) = a2 Solution for x
    float sqDiff = a0 * a0 + a1 * a1 - a2 * a2;
    if (sqDiff < 0)
    { // Check that solution is feasible
        std::cout << "ERROR NO SOLUTION SinCosSum" << std::endl;
        return 0.0 / 0.0; // NaN
    }

    // Mathematica Code:
    //  ArcTan[a1*a2 + Sqrt[a0^2 + a1^2 - a2^2]*Abs[a0], 
    //         a0*a2 - (a1*Sqrt[a0^2 + a1^2 - a2^2])/Sign[a0]]
    float diff = sqrt(sqDiff);
    return atan2(a0 * a2 - a1 * diff / std::copysignf(1.0, a0), 
                 a1 * a2 + diff * std::abs(a0));
}

/**
 * @brief Wraps an angle -Pi to Pi
 * @param angle
 */
float angle_mod(float angle){
    return fmod(angle+M_PI, 2*M_PI) - M_PI;
}

/**
 * @brief Quick Float Nan
 * 
 */
float qnanf(){
    return float(0.0/0.0);
}
} //namespace example_helpers

/**
 * @brief A 3-DoF limb based on the common serial linkage in 12DoF quadrupeds
 * @details Based on the limbs of popular quadrupeds
 * @warning Needs to be tested, use at your own risk!!!!
 * @todo    Needs to be tested ^^^
 * @note    Needs to be tested ^^^
 */
class QuadrupedLimb : public kodlab::LimbBase<Eigen::Vector3f> {
  
  public:

    /**
     * @brief struct containing structural information to a limb with the common
     * quadruped morphology 
     * @details The rotation axis can be described in the zero state to be 
     * parallel to the body axes. In particular, the abduction axis is the 
     * x-direction and the hip and knee axes are the y-direction
     */
    struct LimbConfig {
        /**
         * @brief Offset between centers of two joints, in the frame of the 
         * first joint
         */
        std::array<Eigen::Vector3f,4> actuator_offsets; 
    };

    // Limb index is defined as follows
    typedef uint8_t LimbIndex_t;
    enum {
        kFrontLeft = 0,
        kRearLeft,
        kFrontRight,
        kRearRight
    };

    QuadrupedLimb(
        const std::vector<std::shared_ptr<kodlab::JointBase>> &joints,
        const LimbConfig &config,
        LimbIndex_t limb_id = kFrontLeft)
        : kodlab::LimbBase<Eigen::Vector3f>::LimbBase("", joints),
          config_(config)
    {
        is_front_limb_ = (int(limb_id) % 2) == 0;
        is_left_limb_ = (int(limb_id) / 2) == 0;
    }    
    
    Eigen::Vector3f ForwardKinematicsImpl(
        std::vector<float>thetas) override 
    {      
        Eigen::Vector3f d0 = config_.actuator_offsets[0];
        Eigen::Vector3f d1 = config_.actuator_offsets[1];
        Eigen::Vector3f d2 = config_.actuator_offsets[2];
        Eigen::Vector3f d3 = config_.actuator_offsets[3];

        Eigen::Matrix3f R_AB(Eigen::AngleAxisf(thetas[0], Eigen::Vector3f::UnitX()));
        Eigen::Matrix3f R_BC(Eigen::AngleAxisf(thetas[1], Eigen::Vector3f::UnitY()));
        Eigen::Matrix3f R_CD(Eigen::AngleAxisf(thetas[2], Eigen::Vector3f::UnitY()));

        return d0 + R_AB * (d1 + R_BC * (d2 + R_CD * d3)); 
    }

    Eigen::MatrixXf JacobianImpl(
        std::vector<float>thetas) override 
    {
        Eigen::Vector3f d0 = config_.actuator_offsets[0];
        Eigen::Vector3f d1 = config_.actuator_offsets[1];
        Eigen::Vector3f d2 = config_.actuator_offsets[2];
        Eigen::Vector3f d3 = config_.actuator_offsets[3];

        Eigen::Matrix3f R_AB(Eigen::AngleAxisf(thetas[0], Eigen::Vector3f::UnitX()));
        Eigen::Matrix3f R_BC(Eigen::AngleAxisf(thetas[1], Eigen::Vector3f::UnitY()));
        Eigen::Matrix3f R_CD(Eigen::AngleAxisf(thetas[2], Eigen::Vector3f::UnitY()));

        Eigen::Matrix3f xhat_skew = quad_example_helpers::Skew(Eigen::Vector3f::UnitX());
        Eigen::Matrix3f yhat_skew = quad_example_helpers::Skew(Eigen::Vector3f::UnitY());

        Eigen::Matrix3f jac;
        jac.col(0) = R_AB * xhat_skew * (d1 + R_BC * (d2 + R_CD * d3));
        jac.col(1) = R_AB * R_BC * yhat_skew * (d2 + R_CD * d3);
        jac.col(2) = R_AB * R_BC * R_CD * yhat_skew * (d3);

        return jac;
    }

    std::vector<float> InverseKinematicsImpl(
        const Eigen::Vector3f &EE_pos) override 
    {
        /* Convenience Definitions */
        using quad_example_helpers::qnanf;
        using quad_example_helpers::GetSign;

        Eigen::Vector3f d0 = config_.actuator_offsets[0];
        Eigen::Vector3f d1 = config_.actuator_offsets[1];
        Eigen::Vector3f d2 = config_.actuator_offsets[2];
        Eigen::Vector3f d3 = config_.actuator_offsets[3];
        float L0 = d0.norm();
        float L1 = d1.norm();
        float L2 = d2.norm();
        float L3 = d3.norm();
        Eigen::Vector4f L_vec;
        L_vec << L0, L1, L2, L3;

        /* Dealing with multiple solutions */
        // Assume abductor solutions are not inverted
        bool abd_is_inverted = false; 
        // Will naively aim for nearer solution to current state
        Eigen::Vector3f pose_seed = Eigen::Map<Eigen::Vector3f>(
            joint_positions_.get().data());

        
        Eigen::Vector3f pos = EE_pos; // Copy toe pos

        /* Offset, offset desired toe position by abductor offset vector */
        pos -= config_.actuator_offsets[0]; 
        
        /* Abduction Angle */
        float acos_l1_yznorm = GetSign(is_left_limb_) * acos(L1 / sqrt(pos.y() * pos.y() + pos.z() * pos.z()));
        float atan_z_y = atan2(GetSign(is_left_limb_) * pos.z(), GetSign(is_left_limb_) * pos.y());

        // Assume the solution is the closer to the seed abduction 
        float signed_acos_l1_yznorm = GetSign(is_left_limb_) * GetSign(!abd_is_inverted) * acos_l1_yznorm;
        float abd_sol_1 = atan_z_y + signed_acos_l1_yznorm;
        float abd_sol_2 = atan_z_y - signed_acos_l1_yznorm;
        bool is_sol_1_nearer = (fabs(abd_sol_1 - pose_seed(0)) < fabs(abd_sol_2 - pose_seed(0)));
        float theta_abd = is_sol_1_nearer ? abd_sol_1 : abd_sol_2;
        theta_abd = quad_example_helpers::angle_mod(theta_abd);

        // Check Validity of solution given constraints 
        if (!IsValidAngle(theta_abd, 0))
        {
            theta_abd = !is_sol_1_nearer ? abd_sol_1 : abd_sol_2;; // try second abduction solution
            if (!IsValidAngle(theta_abd, 0))
            {
                std::cout << "[QuadLimb::InverseKinematics] Cannot compute valid abduction angle." << std::endl;
                return {qnanf(), qnanf(), qnanf()}; // return NaN vector
            }
        }

        /* Knee Angle, using law of cosines */
        float pos_sq_norm = pos.squaredNorm();
        float len_sq_norm = L_vec.tail<3>().squaredNorm();
        float theta_knee = -(acos((-len_sq_norm + pos_sq_norm) / (2.0 * L2 * L3)));
        if (!IsValidAngle(theta_knee, 2))
        {
            std::cout << "[QuadrupedLimb::InverseKinematics] Cannot compute valid knee angle." << std::endl;
            return {qnanf(), qnanf(), qnanf()}; // return NaN vector
        }

        /* Hip Angle */
        float sin_th_ab, cos_th_ab;
        sincosf(theta_abd, &sin_th_ab, &cos_th_ab);

        // Move toe into hip plane for hip knee calculation
        // Projected Leg Vector on the xz plane of the HIP
        // [1,0,0;0,0,1] (Rx(th) (xToe - d0) - d1  - [0;L1;0])
        // Projection Toe
        // [1,0,0;0,0,1] Rx(th)
        Eigen::Matrix<float, 2, 3> toeProjMatrix;
        toeProjMatrix << 1, 0, 0,
            0, -sin_th_ab, cos_th_ab; // angle sum identity 
        Eigen::Vector2f projToe = toeProjMatrix * pos - config_.actuator_offsets[1]({0, 2});
        // Hip-Knee Plane Leg Length
        float r_sq = pos_sq_norm - L1 * L1;
        float r = sqrt(r_sq);
        // Compute & Clip Angle
        float theta_hip = atan2(-projToe.x(), -projToe.y()) + acos(-(L3 * L3 - L2 * L2 - r_sq) / (2.0 * L2 * r));
        theta_hip = quad_example_helpers::angle_mod(theta_hip);
        
        /* Return Computed Angles */
        return {theta_abd, theta_hip, theta_knee};
    }

    // Slight optimization over individual calculations
    void FKAndJacobianImpl(
        std::vector<float> joint_positions) override
    {
        using quad_example_helpers::Skew;
        Eigen::Matrix3f R_AB(Eigen::AngleAxisf(joint_positions[0], 
                                               Eigen::Vector3f::UnitX()));
        Eigen::Matrix3f R_BC(Eigen::AngleAxisf(joint_positions[1], 
                                               Eigen::Vector3f::UnitY()));
        Eigen::Matrix3f R_CD(Eigen::AngleAxisf(joint_positions[2], 
                                               Eigen::Vector3f::UnitY()));

        Eigen::Matrix3f abduc_frame = R_AB;
        Eigen::Matrix3f hip_frame = abduc_frame * R_BC;
        Eigen::Matrix3f knee_frame = hip_frame * R_CD;
        
        Eigen::Vector3f abduc_pos = config_.actuator_offsets[0];
        Eigen::Vector3f hip_pos = abduc_pos + 
                                  abduc_frame * config_.actuator_offsets[1];
        Eigen::Vector3f knee_pos = hip_pos +
                                   hip_frame * config_.actuator_offsets[2]; 

        Eigen::Matrix3f xhat_skew = Skew(Eigen::Vector3f::UnitX());
        Eigen::Matrix3f yhat_skew = Skew(Eigen::Vector3f::UnitY());

        Eigen::Vector3f toe_position;
        Eigen::Matrix3f toe_jacobian;
        
        toe_position = config_.actuator_offsets[3];
        toe_jacobian.col(2) = knee_frame * yhat_skew * toe_position;
        toe_position = config_.actuator_offsets[2] + R_CD * toe_position;
        toe_jacobian.col(1) = hip_frame * yhat_skew * toe_position;
        toe_position = config_.actuator_offsets[1] + R_BC * toe_position;
        toe_jacobian.col(0) = abduc_frame * xhat_skew * toe_position;
        toe_position = config_.actuator_offsets[0] + R_AB * toe_position;

        // Set the cached kinematics
        fk_.set(toe_position);
        jac_.set(toe_jacobian);
    } 



    /**
     * @brief Returns true if it is a front limb
     * 
     * @return true 
     * @return false 
     */
    bool IsFrontLimb(){
        return is_front_limb_;
    }

    /**
     * @brief Returns true if it is a left limb
     * 
     * @return true 
     * @return false 
     */
    bool IsLeftLimb(){
        return is_left_limb_;
    }


  private:
    bool is_front_limb_ = true; ///< True is index is front limb
    bool is_left_limb_ = true; ///< True is index is left limb
    LimbConfig config_; ///< Config the common quad

    /**
     * @brief Evaluate if an angle is valid for a specific joint.
     * 
     * @param ang angle in joint angle units
     * @param idx index of joint with limits to be verified
     * @return true if `ang` is valid for joint at index `idx`, 
     * false otherwise or if NaN
     */
    bool IsValidAngle(float ang, int idx)
    {
        return (ang >= joints_[idx]->get_pos_limit_min()) && 
               (ang <= joints_[idx]->get_pos_limit_max());
    }
};
