/**
 * @file limb_base.h
 * @author Kodlab - Zac Gong (zacgong@seas.upenn.edu)
 * @author Kodlab - J. Diego Caporale (jdcap@seas.upenn.edu)
 * @brief Abstract class for limbs, consisting of multiple joints
 * 
 * @date 2022-08-18
 *
 * @copyright BSD 3-Clause License, 
 * Copyright (c) 2021 The Trustees of the University of Pennsylvania. 
 * All Rights Reserved
 *
 */

#pragma once

#include <limits>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include <Eigen/Dense>

#include "kodlab_mjbots_sdk/common_header.h"
#include "kodlab_mjbots_sdk/joint_base.h"

namespace kodlab
{
    /**
     * @brief struct containing structural information to a limb
     * @details all information provided by user at initialization
     */
    struct LimbConfig {
        /**
         * @brief Offset between ceneters of two joints, in the frame of the 
         * first joint
         */
        std::vector<std::array<float, 3>> actuator_offsets; 

        /**
         * @brief Rotation matrix transforming the previous joint frame to 
         * the current joint frame, beginning with \f${}^0 R_1\f$
         */
        std::vector<Eigen::Matrix3f> actuator_orientations; 

        /**
         * @brief Matrix of zeros and ones for reordering joints in member 
         * methods 
         */
        Eigen::MatrixXf selection_matrix;
    };

    template<typename EndEffectorOutput = Eigen::VectorXf>
    class LimbBase {
      public:
        /**
         * @brief Construct a new Limb Base object with a name
         *
         * @param name Sets the limb name
         * @param joints Joints that make up the limb
         * @param config Configuration of the joints in the limb
         */
        LimbBase(const std::string &name,
                 const std::vector<std::shared_ptr<JointBase>> &joints,
                 const LimbConfig &config);

        /**
         * @brief Construct a new Limb Base object
         *
         * @param joints Joints that make up the limb
         * @param config Configuration of the joints in the limb
         */
        LimbBase(const std::vector<std::shared_ptr<JointBase>> &joints,
                 const LimbConfig &config);


        /**
         * @brief Destroy the Limb Base object
         * 
         */
        virtual ~LimbBase(){};

        /**
         * @brief Update position and velocity of each joint within limb
         *
         */
        void UpdateJointStates();

        /**
         * @brief Update FK and jacobian for the limb, can be overloaded 
         * for optimization
         */
        virtual void Update();

        /**
         * @brief Invalidate caches in limb
         *
         */
        void InvalidateCache();

        /**
         * @brief Calculates and returns the forward kinematics of the limb
         *
         * @return Eigen::VectorXf
         */
        EndEffectorOutput ForwardKinematics();

        /**
         * @brief Calculates and returns the Jacobian of the limb
         *
         * @return Eigen::MatrixXf
         */
        Eigen::MatrixXf Jacobian();
        
        /**
         * @brief Calculates the inverse kinematics of the limb given a desired 
         * pose
         *
         * @param EE_pos desired end effector position, type: EndEffectorOutput
         *
         * @return std::vector<float>, joint positions
         */
         std::vector<float> InverseKinematics(const EndEffectorOutput &EE_pos);

        /**
         * @brief Implementation of the forward kinematics, to be overloaded by user
         *
         * @return EndEffectorOutput
         */
        virtual EndEffectorOutput ForwardKinematicsImpl(
            std::vector<float> joint_positions) = 0;

        /**
         * @brief Implementation of Jacobian, to be overloaded by user
         *
         * @return Eigen::MatrixXf
         */
        virtual Eigen::MatrixXf JacobianImpl(
            std::vector<float> joint_positions) = 0;

        /**
         * @brief Implementation of FK and Jac combined for optimization, 
         * to be overloaded by user
         *
         * @return None, need to set fk_ and jac_ in implementation if used
         */
        virtual void FKAndJacobianImpl(std::vector<float> joint_positions) {};

        /**
         * @brief Implementation of the inverse kinematics of the limb 
         * given a desired pose
         *
         * @return std::vector<float> joint positions
         */
        virtual std::vector<float> InverseKinematicsImpl(
            const EndEffectorOutput &EE_pos) = 0;

        /**
         * @brief Get the positions of each joint in the limb
         *
         * @return std::vector<float>
         */
        std::vector<float> get_joint_positions();

        /**
         * @brief Get the velocities of each joint in the limb
         *
         * @return std::vector<float>
         */
        std::vector<float> get_joint_velocities();

      protected:
        /**
         * @brief Set the positions of each joint in the limb
         *
         * @param positions
         */
        void set_joint_positions(const std::vector<float> &positions);

        /**
         * @brief Set the velocities of each joint in the limb
         *
         * @param velocities
         */
        void set_joint_velocities(const std::vector<float> &velocities);
    
        std::string name_ = ""; // Optional limb name

        // Vector of joints within array
        std::vector<std::shared_ptr<JointBase>> joints_;

        // Limb Shape and Size
        int limbDOFs_;
        LimbConfig config_;

        // Limb State/Commands
        bool joint_cache_bool_ = false;        /// check if limb state is up to date
        std::vector<float> joint_positions_;   /// positions of each joint in limb [rad]
        std::vector<float> joint_velocities_;  /// velocities of each joint in limb [rad/s]
        ValidatedLoopCache<EndEffectorOutput> fk_;///< forward kinematics storage
        ValidatedLoopCache<Eigen::MatrixXf> jac_; ///< jacobian storage
    };
}