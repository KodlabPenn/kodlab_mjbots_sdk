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
#include "kodlab_mjbots_sdk/imu_data.h"
#include "kodlab_mjbots_sdk/limb_base.h"

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
        std::vector<std::shared_ptr<LimbBase>> limbs; /// vector of limbs for the robot, if choosing to not use limbs can be an empty vector always

        /*!
         * @brief constructs a robot_interface that contains the basic state of a jointed robot with attitude
         * @param joint_vect a vector of shared pointers to jointbase defining the motors in the robot
         * @param soft_start_duration_ms how long in ms to spend ramping the torque
         * @param robot_max_torque the maximum torque to allow per motor in the robot
         * @param imu_data_ptr [Optional] Shared pointer to imu_data to use or nullptr if robot should make its own
         */
        template <class JointDerived = JointBase>
        RobotBase(std::vector<std::shared_ptr<JointDerived>> joint_vect,
                  float robot_max_torque,
                  float soft_start_duration_ms,
                  std::shared_ptr<::kodlab::IMUData<float>> imu_data_ptr = nullptr)
            : soft_start_(robot_max_torque, soft_start_duration_ms)
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
            
            // Initialize attitude shared pointer
            if (imu_data_ptr) {
                imu_data_ = imu_data_ptr;
            }
            else {
                imu_data_ = std::make_shared<::kodlab::IMUData<float>>();
            }
            run_timer_.tic(); // Start timer
        }

        /*!
         * @brief Destroy the robot base object. Virtual destructor for proper
         *        derived pointer destruction.
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
        virtual void Update(){};

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

        /*!
        * @brief accessor for the IMU data of the robot
        * @return const reference to the IMU data object for the robot
        */
        const ::kodlab::IMUData<float>& GetIMUData(){return *imu_data_;}

        /*!
        * @brief accessor for the IMU data of the robot
        * @return const IMU data shared pointer for the robot
        */
        const std::shared_ptr<::kodlab::IMUData<float>> GetIMUDataSharedPtr(){return imu_data_;}

        /*!
        * @brief Setter for the robot's IMU data pointer. Releases the previously owned IMU data object
        *
        * @param imu_data_ptr a shared pointer to kodlab::IMUData
        */
        void SetIMUDataSharedPtr(std::shared_ptr<::kodlab::IMUData<float>> imu_data_ptr){imu_data_ = imu_data_ptr;}

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


        /*!
         * @brief Add a user generated limb to the vector of limbs for the robot
         * @param Limb the limb to be added to the vector
         */
        void AddLimb(std::shared_ptr<LimbBase> Limb); 

    protected:
        std::vector<std::reference_wrapper<const float>> positions_;  /// Vector of the motor positions (references to the members of joints_)
        std::vector<std::reference_wrapper<const float>> velocities_; /// Vector of the motor velocities (references to the members of joints_)
        std::vector<std::reference_wrapper<const float>> torque_cmd_; /// Vector of the torque command sent to motors (references to the members of joints_)
        std::shared_ptr<::kodlab::IMUData<float>> imu_data_;          /// Robot IMU data
        real_time_tools::Timer run_timer_;                            /// Run timer for robot, started at construction
        SoftStart soft_start_;                                        /// Soft Start object
        int num_joints_ = 0;                                          /// Number of joints
    };
} // namespace kodlab
