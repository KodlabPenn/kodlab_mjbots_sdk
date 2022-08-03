/*!
 * @file robot_base.h
 * @author Kodlab - J. Diego Caporale (jdcap@seas.upenn.edu)
 * @brief Robot Base Class
 * @date 2022-07-12
 * 
 * @copyright Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
 *            BSD 3-Clause License
 * 
 */
#pragma once 

#include <vector>
#include <memory>
#include "kodlab_mjbots_sdk/common_header.h"
#include "kodlab_mjbots_sdk/joint_base.h"
#include "kodlab_mjbots_sdk/soft_start.h"

namespace kodlab
{
    /*!
     * @brief Sets up a kodlab robot base class
     * 
     */
    class RobotBase {
    public:
        static const int KILL_ROBOT = -1;  // Kill mode/behavior index; used to signal robot E-stop
        std::vector< std::shared_ptr<JointBase> > joints; ///the vector of shared_ptrs to joints 
        u_int64_t cycle_count_; //TODO Make this time based not cycle based for when the system fails to keep up (i.e. time_us_)
        
        /*!
         * @brief constructs a robot_interface that contains the basic state of a jointed robot with attitude
         * @param joint_vect a vector of shared pointers to jointbase defining the motors in the robot
         * @param soft_start_duration how long in dt to spend ramping the torque
         * @param robot_max_torque the maximum torque to allow per motor in the robot
         */
        template <class JointDerived = JointBase>
        RobotBase(std::vector<std::shared_ptr<JointDerived>> joint_vect,
                  float robot_max_torque,
                  int soft_start_duration)
            : soft_start_(robot_max_torque, soft_start_duration)
        {
            // Ensure at compile time that the template is JointBase or a child of JointBase
            static_assert(std::is_base_of<JointBase, JointDerived>::value);

            // Set up joint vector and joint states
            for (std::shared_ptr<JointDerived> j : joint_vect){
                joints.push_back(j); // Copy constructed, uses an implicit upcasting
                positions_.push_back( j->get_position_reference() );
                velocities_.push_back( j->get_velocity_reference() );
                torque_cmd_.push_back( j->get_servo_torque_reference() ); 
            }

            // Set number of joints
            num_joints_ = joint_vect.size();
        }

        /*!
         * @brief Destroy the Robot Interface object. Virtual destructor for proper derived pointer destruction.
         */
        virtual ~RobotBase() {};

        /*!
         * @brief Initialize the robot (e.g. communication or states etc.)
         */
        virtual void Init(){};

        /*!
         * @brief Update the robot state and torque command.
         * @warning All derivative classes overriding this method should include
         *          a cycle count increment (i.e., `cycle_count_++;`).
         */
        virtual void Update(){cycle_count_++;}; //TODO remove cycle_count and use time or more intelligently set cycle_count or properly handle softstart

        /*!
         * @brief Stop the robot by setting torques to zero. Can be overridden
         *        if other features are available e.g. mehanical or regen braking
         */
        virtual void Stop(){SetTorques(std::vector<float>(num_joints_, 0));}

        /*!
         * @brief accessor for joint positions, takes into account direction and offset
         * @return the joint positions
         */
        std::vector<float> GetJointPositions();

        /*!
         * @brief accessor for joint velocities, takes into account direction
         * @return the joint velocities
         */
        std::vector<float> GetJointVelocities();

        /*!
         * @brief setter for torque command
         * @param torques[in] vector of torques
         */
        void SetTorques(std::vector<float> torques);

        /*!
         * @brief accessor for the torque md, takes into account direction
         * @return the torque cmd
         */
        std::vector<float> GetJointTorqueCmd();

        //TODO virtual Attitude GetAttitude()

        /*! 
         * @brief Get sub-vector of shared_ptr to joint objects via a set of indices
         * @param joint_indices set of desired joint indices as std::vector of ints
         * @return a vector shared pointers to the desired joints
         */
        std::vector<std::shared_ptr<JointBase>> GetJoints(std::vector<int> joint_indices);

        /*!
         * @brief Get sub-vector of shared_ptr to joint objects via a set of indices
         * @param joint_indices set of desired joint indices std::initializer_list of ints
         * @return a vector shared pointers to the desired joints
         */
        std::vector<std::shared_ptr<JointBase>> GetJoints(std::initializer_list<int> joint_indices);

        /*!
         * @brief Get sub-vector of shared_ptr to joint objects via a set of indices
         * @param joint_indices set of desired joint indices std::array of ints
         * @return a vector shared pointers to the desired joints
         */
        template <size_t N>
        std::vector<std::shared_ptr<JointBase>> GetJoints(std::array<int,N> joint_indices);

        /*!
         * @brief Get the vector of shared_ptrs to joints 
         * @note Added getter to public member for interface consistency with subvector getters
         * @param joint_indices set of desired joint indices std::initializer_list of ints
         * @return a vector shared pointers to the desired joints
         */
        std::vector<std::shared_ptr<JointBase>> GetJoints(){return joints;}

    protected:
        std::vector<std::reference_wrapper<const float>> positions_;  /// Vector of the motor positions (references to the members of joints_)
        std::vector<std::reference_wrapper<const float>> velocities_; /// Vector of the motor velocities (references to the members of joints_)
        std::vector<std::reference_wrapper<const float>> torque_cmd_; /// Vector of the torque command sent to motors (references to the members of joints_)
        //TODO Attitude attitude_; 
        SoftStart soft_start_;                                        /// Soft Start object
        int num_joints_ = 0;                                          /// Number of joints
    };
} // namespace kodlab