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
#include <Eigen>
#include "kodlab_mjbots_sdk/joint_base.h"

namespace kodlab
{
/** 
 * Abstract limb class that takes in a series of joints and treats it as an abstract limb
 * Contains the kinematics and configurations for a leg
 * @brief Abstract base class for legs consisting of multiple joints to be used in the robot class
 */

class LimbBase {
    public: 

        /**
         * @brief Construct a new Limb Base object with a name
         * 
         * @param name          /// Sets the limb name
         * @param joints        /// Joints that make up the leg
         * 
         */

        LimbBase(
            std::string name,
            std::vector<std::shared_ptr<JointBase>> joints
        );

        /**
         * @brief Construct a new Limb Base object
         * 
         * @param joints        /// Joints that make up the leg
         * 
         */
        LimbBase(
            std::vector<std::shared_ptr<JointBase>> joints
        );


        /**
         * @brief Update position and velocity of each joint within leg
         * 
         * @param pos_list desired positions of each joint
         * @param vel_list desired velocities of each joint
         * 
         */
        virtual void Update(std::vector<float> pos_list, std::vector<float> vel_list);

    protected: 
        std::string name_ = ""; // Optional leg name

        // Leg State/Commands 
        std::vector<float> positions_;      /// positions of each joint in leg [rad]
        std::vector<float> velocities_;     /// velocities of each joint in leg [rad/s]
        std::vector<float> torques_;        /// Constrained torques for each joint in the leg [N m]

};
}