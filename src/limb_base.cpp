/**
 * @file limb_base.cpp
 * @author Kodlab - Zac Gong (zacgong@seas.upenn.edu)
 * @brief Class implementation for LimbBase.
 * @date 2022-08-18
 *
 * @copyright BSD 3-Clause License, Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
 *
 */

#include "kodlab_mjbots_sdk/limb_base.h"

namespace kodlab
{
    LimbBase::LimbBase(const std::string &name,
                       const std::vector<std::shared_ptr<JointBase>> &joints,
                       const LimbConfig &config)
    {
        joints_ = joints;
        config_ = config;
        legDOFs_ = joints.size();
    }

    LimbBase::LimbBase(const std::vector<std::shared_ptr<JointBase>> &joints,
                       const LimbConfig &config)
    {
        joints_ = joints;
        config_ = config;
    }
}