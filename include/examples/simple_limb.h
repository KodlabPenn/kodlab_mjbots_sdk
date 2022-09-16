/**
 * @file simple_limb.h
 * @author Zac Gong
 * @brief A simple LimbBase derived class example, a 1DOF limb with 1 joint at
 *        the origin and a end effector
 * @date 2022-09-02
 *
 * @copyright (c) Copyright 2022 The Trustees of the University of Pennsylvania
 *
 */

#pragma once

#include <Eigen/Dense>

#include "kodlab_mjbots_sdk/limb_base.h"

/**
 * @brief A simple 1-DOF limb consisting of a single joint with a striaght
 * end-effector
 * @details A simple, single-degree-of-greedom limb implementation demonstrating 
 * the specification of a Cartesian offset and implementation of kinematics
 * methods
 */
class SimpleLimb : public kodlab::LimbBase {

    using kodlab::LimbBase::LimbBase;
}