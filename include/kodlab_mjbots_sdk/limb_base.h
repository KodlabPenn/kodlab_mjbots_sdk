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
#include <vector>
#include <type_traits>
#include <string>
#include "kodlab_mjbots_sdk/joint_base.h"
#include <Eigen/Dense>

namespace kodlab
{
/** 
 * Abstract limb class that takes in a series of joints and treats it as an abstract limb
 * Contains the kinematics and configurations for a leg
 * @brief Abstract base class for legs consisting of multiple joints to be used in the robot class
 */

struct LimbConfig{
    std::vector<std::array<float,3>> actuator_offsets; // Offset between centers of two joints, in the frame of the first joint
    std::vector<Eigen::Matrix3f> actuator_orientations; // Rotation matrices for each joint with respect to the previous joint (begins with R0_1)
    Eigen::MatrixXf selection_matrix;
};

class LimbBase {
    public: 

        /**
         * @brief Construct a new Limb Base object with a name
         * 
         * @param name          /// Sets the limb name
         * @param joints        /// Joints that make up the leg
         * @param config        /// Configuration of the joints in the leg
         * 
         */

        LimbBase(
            std::string name,
            std::vector<std::shared_ptr<JointBase>> joints,
            LimbConfig config
        );

        /**
         * @brief Construct a new Limb Base object
         * 
         * @param joints        /// Joints that make up the leg
         * @param config        /// Configuration of the joints in the leg
         * 
         */
        
        LimbBase(
            std::vector<std::shared_ptr<JointBase>> joints,
            LimbConfig config
        );

        virtual ~LimbBase() {};

        /**
         * @brief Update position and velocity of each joint within leg
         * 
         * @param pos_list desired positions of each joint
         * @param vel_list desired velocities of each joint
         * 
         */
        void Update(std::vector<float> pos_list, std::vector<float> vel_list);


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
         * @return Eigen::VectorXf 
         */

        virtual void InverseKinematics(std::vector<float> EE_pos) = 0;


        /**
         * @brief Get the Positions of each joint in the leg
         * 
         * @return std::vector<float> 
         */
        std::vector<float> get_positions();

        /**
         * @brief Get the velocities of each joint in the leg
         * 
         * @return std::vector<float> 
         */
        std::vector<float> get_velocities();

        /**
         * @brief Get the Torques of each joint in the leg
         * 
         * @return std::vector<float> 
         */
        std::vector<float> get_torques();

        /**
         * @brief Set the positions of each joint in the leg
         * 
         * @param positions 
         */

        void set_positions(std::vector<float> positions);

        /**
         * @brief Set the velocities of each joint in the leg
         * 
         * @param velocities 
         */

        void set_velocities(std::vector<float> velocities);


    protected: 
        std::string name_ = ""; // Optional leg name

        // Vector of joints within array
        std::vector<std::shared_ptr<JointBase>> joints_;

        // Leg Shape and Size 
        int legDOFs_;
        LimbConfig config_;

        // Leg State/Commands 
        std::vector<float> positions_;      /// positions of each joint in leg [rad]
        std::vector<float> velocities_;     /// velocities of each joint in leg [rad/s]
        std::vector<float> torques_;        /// Constrained torques for each joint in the leg [N m]
};
}