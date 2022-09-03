/**
 * @file simple_limb.h
 * @author Zac Gong
 * @brief A simple LimbBase derived class example, a 1DOF limb with 1 joint at 
 *        the origin and a end effector
 * @date 2022-09-02
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#include <Eigen/Dense>
#include "kodlab_mjbots_sdk/limb_base.h"

/**
 * @brief A simple 1DOF limb, just 1 joint with a straight end effector with a specified
 * xyz offset, shows implementation of kinematics
 * 
 */

class SimpleLimb : public kodlab::LimbBase {

    using kodlab::LimbBase::LimbBase;

    public: 
        void ForwardKinematics() override {
            Eigen::Matrix4f FK;
            float theta = positions_[0];
            FK(0,0) = cos(theta);
            FK(0,1) = -sin(theta);
            FK(1,0) = sin(theta);
            FK(1,1) = cos(theta);
            FK(2,2) = 1.0;
            FK(3,3) = 1.0;
            FK(0,3) = config_.actuator_offsets[0][0] * cos(theta);
            FK(1,3) = config_.actuator_offsets[0][1] * sin(theta); //DOUBLE CHECK THESE
            FK(2,3) = config_.actuator_offsets[0][2];

            //return FK;
        }

        void InverseKinematics(std::vector<float> EE_pos) override {
            std::vector<float> pos;
            float pos1 = atan(EE_pos[1]/EE_pos[0]);
            pos.push_back(pos1);

            //return pos;
        }

};