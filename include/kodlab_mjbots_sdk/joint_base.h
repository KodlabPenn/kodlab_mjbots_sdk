/**
 * @file joint_base.h
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
#include <memory>
#include <vector>
#include <type_traits>
#include <string>
#include <algorithm>
#include <functional>

namespace kodlab
{

/**
 * Abstract joint state class with torque and joint pose limiting capabilities.
 * Includes position limits that activate safety measures currently defaulting
 * to a torque zeroing capability. Includes capping torque limits that limit torque. 
 * @brief Abstract Base class for any joint used in the robot class.
 */
class JointBase {

    public:

        /**
         * @brief Construct a new Joint Base object with a name
         * 
         * @param name          /// Sets the joint name
         * @param direction     /// 1 or -1, flips positive rotation direction (Default:1)
         * @param zero_offset   /// offset [rad] of joint zero position (Default:0) 
         * @param gear_ratio    /// Gear ratio joint to servo (ratio>1 means slower joint) (Default:1.0)
         * @param max_torque    /// Maximum torque of the joint [N m] (Default:inf)
         * @param pos_min       /// Minimum joint limit before taking protective measures such as torque limiting or shut off (Default:inf)
         * @param pos_max       /// Maximum joint limit before taking protective measures such as torque limiting or shut off (Default:-inf)
         */
        JointBase(
                std::string name,
                int direction = 1, 
                float zero_offset = 0,
                float gear_ratio = 1.0, 
                float max_torque = std::numeric_limits<float>::infinity(),
                float pos_min = -std::numeric_limits<float>::infinity(), 
                float pos_max = std::numeric_limits<float>::infinity()
                );

        /**
         * @brief Construct a new Joint Base object
         * 
         * @param direction     /// 1 or -1, flips positive rotation direction (Default:1)
         * @param zero_offset   /// offset [rad] of joint zero position (Default:0) 
         * @param gear_ratio    /// Gear ratio joint to servo (ratio>1 means slower joint) (Default:1.0)
         * @param max_torque    /// Maximum torque of the joint [N m] (Default:inf)
         * @param pos_min       /// Minimum joint limit before taking protective measures such as torque limiting or shut off (Default:inf)
         * @param pos_max       /// Maximum joint limit before taking protective measures such as torque limiting or shut off (Default:-inf)
         */
        JointBase(
                int direction = 1, 
                float zero_offset = 0,
                float gear_ratio = 1.0, 
                float max_torque = std::numeric_limits<float>::infinity(),
                float pos_min = -std::numeric_limits<float>::infinity(), 
                float pos_max = std::numeric_limits<float>::infinity()
                );

        /**
         * @brief Update position and velocity using all the servo params (gear ratio, offset, direction)
         * 
         * @param servo_pos raw servo position
         * @param servo_vel raw servo velocity
         */
        virtual void UpdateState(float servo_pos, float servo_vel);

        /**
         * @brief Calculate and limit torque and set servo torque_cmd_
         * 
         * @param joint_torque Desired torque command at the joint
         * @return float (a copy of the internal torque_cmd_)
         */
        virtual float UpdateTorque(float joint_torque);

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
         * @brief Set the joint zero offset
         * 
         * @param zero zero offset override
         */
        void set_zero(float zero){zero_offset_ = zero;}

        /**
         * @brief Set the joint name
         *
         * @param name New joint name
         */
        void set_name(std::string name) { name_ = name; }

        /**
         * @brief Get the position
         * 
         * @return float 
         */
        virtual float get_position() const {return position_;}

        /**
         * @brief Get the raw servo position
         * 
         * @return float 
         */
        virtual float get_raw_position() const {return servo_position_;}

        /**
         * @brief Get the velocity
         * 
         * @return float 
         */
        virtual float get_velocity() const {return velocity_;}

        /**
         * @brief Get the torque command
         * 
         * @return float 
         */
        virtual float get_servo_torque() const {return servo_torque_;}

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
        const float& get_servo_torque_reference() const {return servo_torque_; }

        /**
         * @brief Get the minimum position limit
         * 
         * @return minimum position limit
         */
        const float get_pos_limit_min() const {return pos_limit_min_; }

        /**
         * @brief Get the maximum position limit
         * 
         * @return maximum position limit
         */
        const float get_pos_limit_max() const {return pos_limit_max_; }

        /**
         * @brief Get the joint name
         *
         * @return name as string
         */
        const std::string get_name() const { return name_; }

    protected:
        std::string name_ = "";  // Optional joint name
        // Joint Params
        float gear_ratio_  = 1.0;   /// external joint gear ratio
        float zero_offset_ = 0;     /// zero offset [rad]
        int   direction_   = 1;     /// direction [-1 or 1]

        // Joint State/Commands (includes params like gear ratio, direction and offset)
        float position_;    /// position of the joint [rad]
        float velocity_;    /// velocity of the joint [rad/s]
        float torque_ = 0;  /// Constrained torque for the joint [N m]

        // Servo State/Commands (the raw motor values)
        float servo_position_;      /// position of the servo [rad]
        float servo_velocity_;      /// velocity of the servo [rad/s]
        float servo_torque_ = 0;    /// torque command for servo [N m]

        // Features
        bool  soft_stop_active_ = true;  /// enable soft stop flag 

        // Limits
        float max_torque_    =  std::numeric_limits<float>::infinity(); /// max torque [N m]
        float pos_limit_min_ = -std::numeric_limits<float>::infinity(); /// max position [rad] before soft stop
        float pos_limit_max_ =  std::numeric_limits<float>::infinity(); /// min position [rad] before soft stop

};

namespace joint_utils{

/*!
 * @brief Helper class to wrap the vector of shared pointers of JointBaseDerived
 * 
 * @tparam joint_type must be JointBaseDerived type
 */
template <class joint_type>
class JointSharedVector
{
    static_assert(std::is_base_of<JointBase, joint_type>::value); // check that the joint_type is derived from JointBase
    std::vector<std::shared_ptr<joint_type>> v_;  // internal vector of shared pointers 

public:
    /*!
     * @brief constructs a shared_ptr joint_type and appends it to the internal vector
     * 
     * @tparam Args parameter pack for passing through the joint constructor
     * @param args a set of args that match the joint_type constructor
     */
    template <typename... Args>
    void addJoint(Args... args)
    {
        v_.push_back(std::make_shared<joint_type>(args...));
    }

    /*!
     * @brief implicit conversion to vector that returns a copy of the shared_ptr's
     * 
     * @return std::vector<std::shared_ptr<joint_type>> 
     */
    operator std::vector<std::shared_ptr<joint_type>>() { return v_; }
    /*!
     * @brief get a reference to the ith instance of joint_type
     * 
     * @return joint_type>
     */
    joint_type & operator[](size_t i) {return *(v_[i]);}
};

/**
 * @brief Get the joint positions from a vector of pointer to joints
 * 
 * @tparam JointTypePtr (JointType*,can be shared_ptr) )
 * @param joints 
 * @return std::vector<float> positions
 */
template<class JointTypePtr>
std::vector<float> PositionsFromJoints(std::vector<JointTypePtr> joints){
    using JointType =  typename std::pointer_traits<JointTypePtr>::element_type;
    static_assert(  std::is_base_of<JointBase, JointType>::value
                    ); // check that the joint_type is derived from JointBase
    std::vector<float> positions;
    std::transform(joints.begin(), joints.end(),
                   positions.begin(), std::mem_fn(&JointType::get_position));
    return positions;
}

/**
 * @brief Get the joint velocities from a vector of pointer to joints
 * 
 * @tparam JointTypePtr (JointType*,can be shared_ptr) )
 * @param joints 
 * @return std::vector<float> velocities
 */
template<class JointTypePtr>
std::vector<float> VelocitiesFromJoints(std::vector<JointTypePtr> joints){
    using JointType =  typename std::pointer_traits<JointTypePtr>::element_type;
    static_assert(  std::is_base_of<JointBase, JointType>::value
                    ); // check that the joint_type is derived from JointBase
    std::vector<float> velocities;
    std::transform(joints.begin(), joints.end(),
                   velocities.begin(), std::mem_fn(&JointType::get_velocity));
    return velocities;
}

} // joint_utils
} // kodlab
