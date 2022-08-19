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
    LimbBase::LimbBase(std::string name, 
                       std::vector<std::shared_ptr<JointBase>> joints, 
                       LimbConfig config) 
                       {
                        joints_ = joints;
                        config_ = config;
                        legDOFs_ = joints.size();
                       }
    
    LimbBase::LimbBase(std::vector<std::shared_ptr<JointBase>> joints, 
                       LimbConfig config) 
                       {
                        joints_ = joints;
                        config_ = config;
                       }

    void LimbBase::Update(std::vector<float> pos_list, std::vector<float> vel_list) {
        for (int i = 0; i < joints_.size(); i++) {
            positions_.push_back(joints_[i]->get_position());
        }

        for (int i = 0; i < joints_.size(); i++) {
            velocities_.push_back(joints_[i]->get_velocity());
        }

        for (int i = 0; i < joints_.size(); i++) {
            torques_.push_back(joints_[i]->get_servo_torque());
        }
    }

    std::vector<float> LimbBase::get_positions() {
        return positions_;
    }

    std::vector<float> LimbBase::get_velocities() {
        return velocities_;
    }

    std::vector<float> LimbBase::get_torques() {
        return torques_;
    }

    Eigen::Vector3f LimbBase::ForwardKinematics() {
        
    }


}

