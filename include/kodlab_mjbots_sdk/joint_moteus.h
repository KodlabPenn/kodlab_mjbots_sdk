/**
 * @file joint_moteus.h
 * @author J. Diego Caporale (jdcap@seas.upenn.edu)
 * @brief Moteus powered joint class
 * @version 0.1
 * @date 2022-02-22
 * 
 * @copyright BSD 3-Clause License, Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
 * 
 */

#pragma once
#include "kodlab_mjbots_sdk/joint_base.h"
#include "kodlab_mjbots_sdk/moteus_protocol.h"

namespace kodlab{
namespace mjbots{
/**         
 * A JointBase child class that encapsulates parameters and functions of 
 * moteus motor driver powered joint 
 * @brief 
 * Moteus Joint class 
 */
class JointMoteus: public JointBase{
    public:
        /**

         *
         * @brief Construct a new Joint Moteus object
         * 
         * @param can_bus      /// the can bus the moteus communicates on [1-4]
         * @param can_id       /// the can id of this joint's moteus [1-127]
         * @param direction     /// 1 or -1, flips positive rotation direction (Default:1)
         * @param zero_offset   /// offset [rad] of joint zero position from servo zero postition (Default:0)
         * @param gear_ratio    /// Gear ratio joint to servo (ratio>1 means slower joint) (Default:1.0)
         * @param max_torque    /// Maximum torque limit of the joint [N m] (Default:inf)
         * @param pos_min       /// Minimum joint pose limit before taking protective measures such as torque limiting or shut off (Default:-inf)
         * @param pos_max       /// Maximum joint pose limit before taking protective measures such as torque limiting or shut off (Default:inf)
         */
        JointMoteus(
            int can_id, 
            int can_bus,
            int direction = 1, 
            float zero_offset = 0,
            float gear_ratio = 1.0, 
            float max_torque = std::numeric_limits<float>::infinity(),
            float pos_min = -std::numeric_limits<float>::infinity(), 
            float pos_max = std::numeric_limits<float>::infinity()
            )
            :JointBase(direction,zero_offset,gear_ratio,max_torque,pos_min,pos_max),
            can_bus_(can_bus), can_id_(can_id){}
        
        /**
         * @brief Update the joint of the moteus. Converts rot/s to rad/s and saves mode
         * 
         * @param reply_pos position reported by moteus [rot]
         * @param reply_vel velocity reported by moteus [rot/s]
         * @param mode      moteus mode
         */
        void UpdateMoteus(float reply_pos, float reply_vel, ::mjbots::moteus::Mode mode ){
            UpdateState( 2 * M_PI * reply_pos, 2 * M_PI * reply_vel );
            mode_ = mode;
        }

        /**
         * @brief Get the can id 
         * 
         * @return int 
         */
        int get_can_id() const {return can_id_;}

        /**
         * @brief Get the can bus object
         * 
         * @return int 
         */
        int get_can_bus() const {return can_bus_;}

        /**
         * @brief Get the mode reference 
         * 
         * @return const ::mjbots::moteus::Mode& 
         */
        const ::mjbots::moteus::Mode & get_mode_reference()   const {return mode_; }

    private:
        int can_id_;   /// the can id of this joint's moteus
        int can_bus_;  /// the can bus the moteus communicates on
        ::mjbots::moteus::Mode mode_ = ::mjbots::moteus::Mode::kStopped; /// joint's moteus mode
};
}//namespace mjbots
}//namespace kodlab