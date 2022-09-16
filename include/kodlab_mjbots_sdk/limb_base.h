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
}