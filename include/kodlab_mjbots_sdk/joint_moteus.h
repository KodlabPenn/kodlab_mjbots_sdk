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
            float pos_max = std::numeric_limits<float>::infinity())
            : JointBase(name, direction, zero_offset, gear_ratio, max_torque, pos_min, pos_max),
              can_bus_(can_bus), can_id_(can_id) {}

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
         */
        JointMoteus(
            int can_id,
            int can_bus,
            int direction = 1,
            float zero_offset = 0,
            float gear_ratio = 1.0,
            float max_torque = std::numeric_limits<float>::infinity(),
            float pos_min = -std::numeric_limits<float>::infinity(),
            float pos_max = std::numeric_limits<float>::infinity())
            : JointBase("", direction, zero_offset, gear_ratio, max_torque, pos_min, pos_max),
              can_bus_(can_bus), can_id_(can_id) {}


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
                         config.pos_max),
              can_id_( config.can_id),
              can_bus_( config.can_bus) {}
        
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

        /*!
         * @brief Set the kp_scale for the moteus
         * @param kp_scale limited between 0 and 1
         */
        void set_kp_scale(float kp_scale){
          if(kp_scale > 1){
            LOG_WARN("kp_scale is greater than 1, will be limited to 1");
          }
          kp_scale = kp_scale;
        }

        /*!
         * @brief Set the kd_scale for the moteus
         * @param kd_scale limited between 0 and 1
         */
        void set_kd_scale(float kd_scale){
          if(kd_scale > 1){
            LOG_WARN("kd_scale is greater than 1, will be limited to 1");
          }
          kd_scale_ = kd_scale;
        }

        /*!
         * @brief set the joint position target in radians for the pd loop
         * @param position_target target position in radians
         */
        void set_joint_position_target(float position_target){
          position_target_ = position_target;
        }

       /*!
        * @brief set the joint velocity target in radians/s for the pd loop
        * @param velocity_target target velocity in rads/s
        */
        void set_joint_velocity_target(float velocity_target){
          position_target_ = velocity_target;
        }

        /*!
         * @brief accessor for kp_scale
         * @return kp_scale
         */
        [[nodiscard]] float get_kp_scale() const {return kp_scale_;}

        /*!
         * @brief accessor for kd_scale
         * @return kd_scale
         */
        [[nodiscard]] float get_kd_scale() const {return kd_scale_;}

        /*!
         * @brief accessor for joint position target
         * @return joint position target in rad
         */
        [[nodiscard]] float get_joint_position_target()const{return position_target_;}

        /*!
         * @brief accessor for joint velocity target
         * @return joint_velociyt target in radians/s
         */
        [[nodiscard]] float get_joint_velocity_target()const{return velocity_target_;}

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

        // PD setpoints and gain scales
        float kp_scale_ = 0;          ///< kp scale, limited between 0 and 1
        float kd_scale_ = 0;          ///< kd scale, limited between 0 and 1
        float position_target_ = 0;   ///< Target joint position
        float velocity_target_ = 0;   ///< Target joint velocity
};
}//namespace mjbots
}//namespace kodlab