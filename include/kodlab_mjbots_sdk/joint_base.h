/**
 * @file jointBase.h
 * @author Kodlab - J. Diego Caporale (jdcap@seas.upenn.edu)
 * @brief Abstract class for joints. Agnostic to servo/motor used.
 * @version 0.1
 * @date 2022-02-17
 * 
 * @copyright BSD 3-Clause License, Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
 * 
 */

#pragma once
#include <limits>

/**
 * @brief 
 * Abstract Base class for any joint used in the robot class.
 */
class JointBase {

    protected:
        // Joint Params
        float gear_ratio_  = 1.0;   /// external joint gear ratio
        float zero_offset_ = 0;     /// zero offset [rad]
        int   direction_   = 1;     /// direction [-1 or 1]

        // Joint State/Commands
        float position_;    /// position of the joint [rad]
        float velocity_;    /// velocity of the joint [rad/s]
        float torque_ = 0;  /// Constrained torque for the joint [N m]

        // Servo State/Commands
        float servo_position_;      /// position of the servo [rad]
        float servo_velocity_;      /// velocity of the servo [rad/s]
        float servo_torque_ = 0;    /// torque command for servo [N m]

        // Features
        bool  soft_stop_active_ = true;  /// enable soft stop flag 

        // Limits
        float max_torque_    =  std::numeric_limits<float>::infinity(); /// max torque [N m]
        float pos_limit_min_ = -std::numeric_limits<float>::infinity(); /// max position [rad] before soft stop
        float pos_limit_max_ = -std::numeric_limits<float>::infinity(); /// min position [rad] before soft stop

    public:
        /**
         * @brief Construct a new Joint Base object
         * 
         * @param direction     /// 1 or -1, flips positive rotation direction (Default:1)
         * @param zero_offset   /// offset [rad] of joint zero position from servo zero postition (Default:0)
         * @param max_torque    /// Maximum torque of the joint [N m] (Default:inf)
         * @param gear_ratio    /// Gear ratio joint to servo (ratio>1 means slower joint) (Default:1.0)
         * @param pos_min       /// Minimum joint limit (Default:inf)
         * @param pos_max       /// Maximum joint limit (Default:-inf)
         */
        JointBase(int direction = 1, 
                float zero_offset = 0,
                float max_torque = std::numeric_limits<float>::infinity(),
                float gear_ratio = 1.0, 
                float pos_min = -std::numeric_limits<float>::infinity(), 
                float pos_max = std::numeric_limits<float>::infinity())
                :direction_(direction), zero_offset_(zero_offset), 
                 max_torque_(max_torque), gear_ratio_(gear_ratio),
                 pos_limit_min_(pos_min), pos_limit_max_(pos_max){
                    // Set to sign if not -1 or 1
                    direction_ = (direction_ >= 0) - (direction_ < 0); 
                    gear_ratio_ = gear_ratio_ == 0 ? 1.0 : gear_ratio_;
                }

        /**
         * @brief Set the joint position directly
         * 
         * @param pos position override 
         */
        void set_position(float pos){position_ = pos;}

        /**
         * @brief Set the joint velocity directly
         * 
         * @param vel velocity override
         */
        void set_velocity(float vel){velocity_ = vel;}

        /**
         * @brief Update position and velocity using all the servo params (gear ratio, offset, direction)
         * 
         * @param servo_pos raw servo position
         * @param servo_vel raw servo velocity
         */
        virtual void UpdateState(float servo_pos, float servo_vel){  
            servo_position_ = servo_pos;
            servo_velocity_ = servo_vel;
            position_ = direction_ * (servo_pos / gear_ratio_ - zero_offset_); 
            velocity_= direction_ * servo_vel / gear_ratio_ ;
            }

        /**
         * @brief Get the position
         * 
         * @return float 
         */
        virtual float get_position()  {return position_;}

        /**
         * @brief Get the velocity
         * 
         * @return float 
         */
        virtual float get_velocity()  {return velocity_;}

        /**
         * @brief Get the torque command
         * 
         * @return float 
         */
        virtual float get_servo_torque() {return servo_torque_;}

        /**
         * @brief Calculate and limit torque and set servo torque_cmd_
         * 
         * @param joint_torque Desired torque command at the joint
         * @return float (a copy of the internal torque_cmd_)
         */
        virtual float UpdateTorque(float joint_torque){
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

        /**
         * @brief Set the soft stop flag
         * 
         * @param is_active 
         */
        void set_soft_stop(bool is_active) {soft_stop_active_ = is_active;}

        /**
         * @brief Get the position reference 
         * 
         * @return const float& 
         */
        const float& get_position_reference() const {return position_;   }

        /**
         * @brief Get the velocity reference
         * 
         * @return const float& 
         */
        const float& get_velocity_reference() const {return velocity_;   }

        /**
         * @brief Get the torque reference
         * 
         * @return const float& 
         */
        const float& get_servo_torque_reference()   const {return servo_torque_; }
};

