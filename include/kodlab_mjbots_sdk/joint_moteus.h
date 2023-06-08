/**
 * @file joint_moteus.h
 * @author J. Diego Caporale (jdcap@seas.upenn.edu)
 * @brief Moteus powered joint class
 * @date 2022-02-22
 * 
 * @copyright BSD 3-Clause License, Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
 * 
 */

#pragma once
#include "kodlab_mjbots_sdk/joint_base.h"
#include "kodlab_mjbots_sdk/moteus_protocol.h"
#include "kodlab_mjbots_sdk/log.h"
#include <string>

namespace kodlab{
namespace mjbots{
/**
 * @brief struct for storing or passing moteus joint configurations
 * 
 */
struct MoteusJointConfig{
    int can_id;
    int can_bus;
    std::string name = "";
    int direction = 1; 
    float zero_offset = 0;
    float gear_ratio = 1.0;
    float max_torque = std::numeric_limits<float>::infinity();
    float pos_min = -std::numeric_limits<float>::infinity();
    float pos_max = std::numeric_limits<float>::infinity();
    float moteus_kp = 0;
    float moteus_kd = 0;
    float soft_start_duration_ms = 1;
};

/**         
 * @brief A JointBase derived class that encapsulates parameters and functions of 
 * moteus motor driver powered joint 
 * 
 */
class JointMoteus: public JointBase
{
    public:
        /**
         * @brief Construct a new Joint Moteus object
         * 
         * @param name         /// name string for this joint
         * @param can_id       /// the can id of this joint's moteus [1-127]
         * @param can_bus      /// the can bus the moteus communicates on [1-4]
         * @param direction     /// 1 or -1, flips positive rotation direction (Default:1)
         * @param zero_offset   /// offset [rad] of joint zero position from servo zero postition (Default:0)
         * @param gear_ratio    /// Gear ratio joint to servo (ratio>1 means slower joint) (Default:1.0)
         * @param max_torque    /// Maximum torque limit of the joint [N m] (Default:inf)
         * @param pos_min       /// Minimum joint pose limit before taking protective measures such as torque limiting or shut off (Default:-inf)
         * @param pos_max       /// Maximum joint pose limit before taking protective measures such as torque limiting or shut off (Default:inf)
         * @param moteus_kp     /// Value of kp set on moteus in units of N m/rev
         * @param moteus_kd     /// Value of kd set on moteus in units of N m s/rev
         * @param soft_start_duration_ms /// Duration of torque limit ramp (soft start) in ms
         */
        JointMoteus(
            std::string name,
            int can_id,
            int can_bus,
            int direction = 1,
            float zero_offset = 0,
            float gear_ratio = 1.0,
            float max_torque = std::numeric_limits<float>::infinity(),
            float pos_min = -std::numeric_limits<float>::infinity(),
            float pos_max = std::numeric_limits<float>::infinity(),
            float soft_start_duration_ms = 1,
            float moteus_kp = 0,
            float moteus_kd = 0)
            : JointBase(name, direction, zero_offset, gear_ratio, max_torque, pos_min, pos_max, soft_start_duration_ms),
              can_bus_(can_bus),
              can_id_(can_id),
              moteus_kp_(moteus_kp),
              moteus_kd_(moteus_kd)  {}

        /**
         * @brief Construct a new Joint Moteus object without name
         * 
         * @param can_id       /// the can id of this joint's moteus [1-127]
         * @param can_bus      /// the can bus the moteus communicates on [1-4]
         * @param direction     /// 1 or -1, flips positive rotation direction (Default:1)
         * @param zero_offset   /// offset [rad] of joint zero position from servo zero postition (Default:0)
         * @param gear_ratio    /// Gear ratio joint to servo (ratio>1 means slower joint) (Default:1.0)
         * @param max_torque    /// Maximum torque limit of the joint [N m] (Default:inf)
         * @param pos_min       /// Minimum joint pose limit before taking protective measures such as torque limiting or shut off (Default:-inf)
         * @param pos_max       /// Maximum joint pose limit before taking protective measures such as torque limiting or shut off (Default:inf)
         * @param moteus_kp     /// Value of kp set on moteus in units of N m/rev
         * @param moteus_kd     /// Value of kd set on moteus in units of N m s/rev
         * @param soft_start_duration_ms /// Duration of torque limit ramp (soft start) in ms
         */
        JointMoteus(
            int can_id,
            int can_bus,
            int direction = 1,
            float zero_offset = 0,
            float gear_ratio = 1.0,
            float max_torque = std::numeric_limits<float>::infinity(),
            float pos_min = -std::numeric_limits<float>::infinity(),
            float pos_max = std::numeric_limits<float>::infinity(),
            float soft_start_duration_ms = 1,
            float moteus_kp = 0,
            float moteus_kd = 0)
            : JointBase("", direction, zero_offset, gear_ratio, max_torque, pos_min, pos_max, soft_start_duration_ms),
              can_bus_(can_bus),
              can_id_(can_id),
              moteus_kp_(moteus_kp),
              moteus_kd_(moteus_kd) {}


        /**
         * @brief Construct a new Joint Moteus object with a MoteusJointConfig struct
         * 
         * @param config MoteusJointConfig input
         */
        JointMoteus(MoteusJointConfig config)
            : JointBase( config.name,
                         config.direction,
                         config.zero_offset,
                         config.gear_ratio,
                         config.max_torque,
                         config.pos_min,
                         config.pos_max,
                         config.soft_start_duration_ms),
              can_id_( config.can_id),
              can_bus_( config.can_bus),
              moteus_kp_(config.moteus_kp),
              moteus_kd_(config.moteus_kd){}
        
        /**
         * @brief Update the joint of the moteus. Converts rot/s to rad/s and saves mode
         * 
         * @param reply_pos position reported by moteus [rot]
         * @param reply_vel velocity reported by moteus [rot/s]
         * @param reply_torque torque measured by the moteus [Nm]
         * @param mode      moteus mode
         */
        void UpdateMoteus(float reply_pos, float reply_vel, float reply_torque, ::mjbots::moteus::Mode mode ){
            UpdateState( 2 * M_PI * reply_pos, 2 * M_PI * reply_vel, reply_torque);
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

        /*!
         * @brief accessor for kp_scale
         * @return kp_scale
         */
        [[nodiscard]] float get_kp_scale() const {
          if(moteus_kp_ == 0){
            LOG_WARN("Moteus kp is set to 0, while attempting to send pd commands");
            return 0;
          }
          // Multiply by 2pi to convert kp from radians to revolutions
          // Divide by gear ratio to get servo kp rather than joint kp
          const float kp_scale = kp_/moteus_kp_ * 2 * M_PI/gear_ratio_;
          if(kp_scale > 1){
            LOG_WARN("kp_scale is greater than 1, will be limited to 1. Either use a lower kp or increase kp in the moteus");
            LOG_WARN("With the current moteus kp of %.3f, the max value of the joint kp is %.3f", moteus_kp_, kp_/kp_scale);
          }
          return kp_scale;
        }

        /*!
         * @brief accessor for kd_scale
         * @return kd_scale
         */
        [[nodiscard]] float get_kd_scale() const {
          if(moteus_kd_ == 0){
            LOG_WARN("Moteus kd is set to 0, while attempting to send pd commands");
            return 0;
          }
          const float kd_scale = kd_/moteus_kd_ * 2 * M_PI/gear_ratio_;
          if(kd_scale > 1){
            LOG_WARN("kd_scale is greater than 1, will be limited to 1. Either use a lower kd or increase kd in the moteus");
            LOG_WARN("With the current moteus kd of %.3f, the max value of the joint kd is %.3f", moteus_kp_, kd_/kd_scale);
          }
          return kd_scale;
        }

        /*!
         * @brief accessor for the moteus position target
         * @return the moteus position target in units of revolutions
         */
        [[nodiscard]] float get_moteus_position_target()const{return (position_target_ + zero_offset_) * gear_ratio_/direction_/2/M_PI;}

        /*!
         * @brief accessor for the moteus velocity target
         * @return the moteus velocity target in units of revolutions/s
         */
        [[nodiscard]] float get_moteus_velocity_target()const{return (velocity_target_) * gear_ratio_/direction_/2/M_PI;}


    private:
        int can_id_;   /// the can id of this joint's moteus
        int can_bus_;  /// the can bus the moteus communicates on
        ::mjbots::moteus::Mode mode_ = ::mjbots::moteus::Mode::kStopped; /// joint's moteus mode
        float moteus_kp_ = 0;
        float moteus_kd_ = 0;
};
}//namespace mjbots
}//namespace kodlab
