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
 * @brief Forward definition of struct for storing moteus joint configurations
 * 
 */
struct MoteusJointConfig;

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
         * @param soft_start_duration_ms /// Duration of torque limit ramp (soft start) in ms
         * @param query_type    /// QueryCommand struct containing register resolutions
         * @param moteus_kp     /// Value of kp set on moteus in units of N m/rev
         * @param moteus_kd     /// Value of kd set on moteus in units of N m s/rev
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
            ::mjbots::moteus::QueryCommand query_type = kDefaultQuery,
            float moteus_kp = 0,
            float moteus_kd = 0
        );

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
         * @param soft_start_duration_ms /// Duration of torque limit ramp (soft start) in ms
         * @param query_type    /// QueryCommand struct containing register resolutions
         * @param moteus_kp     /// Value of kp set on moteus in units of N m/rev
         * @param moteus_kd     /// Value of kd set on moteus in units of N m s/rev
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
            ::mjbots::moteus::QueryCommand query_type = kDefaultQuery,
            float moteus_kp = 0,
            float moteus_kd = 0
        );

        /**
         * @brief Construct a new Joint Moteus object with a MoteusJointConfig struct
         * 
         * @param config MoteusJointConfig input
         */
        JointMoteus(MoteusJointConfig config);
        
        /**
         * @brief Update the joint of the moteus. Converts rot/s to rad/s and saves other state values.
         * 
         * @param reply_message reply message sent by moteus to pi3hat
         */
        void UpdateMoteus(::mjbots::moteus::QueryResult reply_message);

        /**
         * @brief Destroy the JointMoteus object, virtual to ensure proper 
         * destruction of derived classes
         */
        virtual ~JointMoteus(){}

        /**
         * @brief Determines whether the QueryCommand struct ignores position
         * and velocity when requesting data from the moteus, potentially
         * characteristic of an open-loop controller
         * 
         * @return bool 
         */
        bool is_open_loop_query();

        /**
         * @brief Get the can id 
         * 
         * @return int 
         */
        int get_can_id() const;

        /**
         * @brief Get the can bus object
         * 
         * @return int 
         */
        int get_can_bus() const;

        /**
         * @brief Get the QueryCommand object, which contains resolutions
         * 
         * @return ::mjbots::moteus::QueryCommand
         */
        [[nodiscard]] ::mjbots::moteus::QueryCommand get_query_command() const;

        /**
         * @brief Get the mode reference
         * 
         * @return const ::mjbots::moteus::Mode& 
         */
        const ::mjbots::moteus::Mode & get_mode_reference() const;

        /**
         * @brief Get the current operational mode of the servo
         * 
         * @return ::mjbots::moteus::Mode
         */
        [[nodiscard]] ::mjbots::moteus::Mode get_mode() const;

        /**
         * @brief Get the current position of the servo, measured in rotations
         * of the output shaft. Overrides function in joint_base.h
         * 
         * @return float 
         */
        float get_position() const override;

        /**
         * @brief Get the current velocity of the servo, measured in Hz at the 
         * output shaft. Overrides function in joint_base.h
         * 
         * @return float
         */
        float get_velocity() const override;

        /**
         * @brief Get the measured joint torque. Overrides function in
         * joint_base.h
         * 
         * @return float
         */
        float get_measured_torque() const override;

        /**
         * @brief Get the board temperature, measured in degrees Celsius
         * 
         * @return float
         */
        [[nodiscard]] float get_temperature() const;

        /**
         * @brief Get the current in the Q phase, measured in Amperes
         * 
         * @return float
         */
        [[nodiscard]] float get_q_current() const;

        /**
         * @brief Get the current in the D phase, measured in Amperes
         * 
         * @return float
         */
        [[nodiscard]] float get_d_current() const;

        /**
         * @brief Get the input voltage
         * 
         * @return float
         */
        [[nodiscard]] float get_voltage() const;

        /**
         * @brief Get the fault code, which will be set if the current
         * operational mode of the servo is set to 1 (Fault)
         * 
         * @return const ::mjbots::moteus::Fault&
         */
        const ::mjbots::moteus::Fault & get_fault() const;

        /*!
         * @brief accessor for kp_scale
         * @return kp_scale
         */
        [[nodiscard]] float get_kp_scale() const;

        /*!
         * @brief accessor for kd_scale
         * @return kd_scale
         */
        [[nodiscard]] float get_kd_scale() const;

        /*!
         * @brief accessor for the moteus position target
         * @return the moteus position target in units of revolutions
         */
        [[nodiscard]] float get_moteus_position_target()const;

        /*!
         * @brief accessor for the moteus velocity target
         * @return the moteus velocity target in units of revolutions/s
         */
        [[nodiscard]] float get_moteus_velocity_target()const;

        /**
         * @brief Define a default resolution struct
         */
        static const ::mjbots::moteus::QueryCommand kDefaultQuery;

        /**
         * @brief Define a resolution struct that includes torques
         */
        static const ::mjbots::moteus::QueryCommand kTorqueQuery;

        /**
         * @brief Define a resolution struct that adds fields commonly used in
         * debugging (board temperature and fault code) to kTorqueQuery
         */
        static const ::mjbots::moteus::QueryCommand kDebugQuery;

        /**
         * @brief Define a resolution struct that includes all useful registers
         */
        static const ::mjbots::moteus::QueryCommand kComprehensiveQuery;

    private:
        int can_id_;   /// the can id of this joint's moteus
        int can_bus_;  /// the can bus the moteus communicates on
        ::mjbots::moteus::QueryCommand query_type_;
        ::mjbots::moteus::Mode mode_ = ::mjbots::moteus::Mode::kStopped; /// joint's moteus mode
        float moteus_kp_ = 0;
        float moteus_kd_ = 0;
        float temperature_ = 0;
        float q_current_ = 0;
        float d_current_ = 0;
        float voltage_ = 0;
        ::mjbots::moteus::Fault fault_ = ::mjbots::moteus::Fault::kSuccess; // fault mode
};

/**
 * @brief struct for storing or passing moteus joint configurations
 * 
 */
struct MoteusJointConfig{
    int can_id;
    int can_bus;
    ::mjbots::moteus::QueryCommand query_type = JointMoteus::kDefaultQuery;
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
}//namespace mjbots
}//namespace kodlab
