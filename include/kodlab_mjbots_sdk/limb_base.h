/**
 * @file limb_base.h
 * @author Kodlab - Zac Gong (zacgong@seas.upenn.edu)
 * @brief Abstract class for limbs, consisting of multiple joints
 * @version 0.1
 * @date 2022-08-18
 *
 * @copyright BSD 3-Clause License, Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
 *
 */

#pragma once

#include <limits>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include <Eigen/Dense>

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

    class LimbBase {
    public:
        /**
         * @brief Construct a new Limb Base object with a name
         *
         * @param name Sets the limb name
         * @param joints Joints that make up the leg
         * @param config Configuration of the joints in the leg
         */
        LimbBase(const std::string &name,
                 const std::vector<std::shared_ptr<JointBase>> &joints,
                 const LimbConfig &config);

        /**
         * @brief Construct a new Limb Base object
         *
         * @param joints Joints that make up the leg
         * @param config Configuration of the joints in the leg
         */
        LimbBase(const std::vector<std::shared_ptr<JointBase>> &joints,
                 const LimbConfig &config);


        /**
         * @brief Destroy the Limb Base object
         * 
         */
        virtual ~LimbBase(){};

        /**
         * @brief Update position and velocity of each joint within leg
         *
         * @param pos_list desired positions of each joint
         * @param vel_list desired velocities of each joint
         *
         */
        void Update(const std::vector<float> &pos_list, 
                    const std::vector<float> &vel_list);

        /**
         * @brief Calculates the forward kinematics of the leg
         *
         * @return Eigen::VectorXf
         */
        virtual void ForwardKinematics() = 0;

        /**
         * @brief Calculates the Jacobian of the leg
         *
         * @return Eigen::MatrixXf
         */
        virtual void Jacobian() = 0;

        /**
         * @brief Calculates the Inverse kinematics of a leg given a desired position
         * measured from the origin of the leg (????)
         *
         * @param x_des desired x position
         * @param y_des desired y position
         * @param z_des desired z position
         *
         * @return std::vector<float> positions
         */
        virtual void InverseKinematics(const std::vector<float> &EE_pos) = 0;
}