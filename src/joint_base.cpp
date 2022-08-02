/**
 * @file joint_base.cpp
 * @author Kodlab - J. Diego Caporale (jdcap@seas.upenn.edu)
 * @brief Class implementation for JointBase.
 * @date 2022-03-21
 * 
 * @copyright BSD 3-Clause License, Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
 * 
 */

#include <iostream>
#include "kodlab_mjbots_sdk/joint_base.h"

JointBase::JointBase(std::string name,
                     int direction,
                     float zero_offset,
                     float gear_ratio,
                     float max_torque,
                     float pos_min,
                     float pos_max)
    : name_(name), direction_(direction), zero_offset_(zero_offset),
      max_torque_(max_torque), gear_ratio_(gear_ratio),
      pos_limit_min_(pos_min), pos_limit_max_(pos_max)
{
    // Set to sign if not -1 or 1
    direction_ = (direction_ >= 0) - (direction_ < 0);
    // Don't allow zero or negative gear ratio
    if (gear_ratio <= 0)
    {
        //TODO Replace with LOG_WARN
        std::cout << "Warning: Gear ratio must be positive. \nGear ratio reset to 1.0" << std::endl;
        gear_ratio_ = 1.0;
    }
}

JointBase::JointBase(int direction,
                     float zero_offset,
                     float gear_ratio,
                     float max_torque,
                     float pos_min,
                     float pos_max)
    : JointBase("", direction, zero_offset, gear_ratio, max_torque, pos_min, pos_max) {}

float JointBase::UpdateTorque(float joint_torque){
    // Clip max torque
    joint_torque =  joint_torque >  max_torque_ ?  max_torque_ : joint_torque;
    joint_torque =  joint_torque < -max_torque_ ? -max_torque_ : joint_torque;

    // Do soft stop if active 
    // Soft Stop: don't allow torque in the direction of the limits
    // TODO: Test softstop
    if (soft_stop_active_) {
        if (position_ > pos_limit_max_  ){
            joint_torque = (joint_torque > 0) ? 0 : joint_torque ;
        }
        else if (position_ < pos_limit_min_  ){
            joint_torque = (joint_torque < 0) ? 0 : joint_torque ;
        }
    }
    torque_ = joint_torque;
    servo_torque_ =  direction_ * joint_torque/gear_ratio_;
    return servo_torque_;
}

void JointBase::UpdateState(float servo_pos, float servo_vel){  
    servo_position_ = servo_pos;
    servo_velocity_ = servo_vel;
    position_ = direction_ * servo_pos / gear_ratio_ - zero_offset_ ; 
    velocity_= direction_ * servo_vel / gear_ratio_ ;
}


