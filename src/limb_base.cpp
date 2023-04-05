/**
 * @file limb_base.cpp
 * @author Kodlab - Zac Gong (zacgong@seas.upenn.edu)
 * @author Kodlab - J. Diego Caporale (jdcap@seas.upenn.edu)
 * @brief Class implementation for LimbBase.
 * @date 2022-08-18
 *
 * @copyright BSD 3-Clause License, 
 * Copyright (c) 2021 The Trustees of the University of Pennsylvania. 
 * All Rights Reserved
 *
 */

#include "kodlab_mjbots_sdk/limb_base.h"

namespace kodlab
{
    template<typename EndEffectorOutput>
    LimbBase<EndEffectorOutput>::LimbBase(
        const std::string &name,
        const std::vector<std::shared_ptr<JointBase>> &joints,
        const LimbConfig &config)
        : joints_(joints), 
          config_(config),
          limbDOFs_(joints.size()),
          name_(name) {}

    template<typename EndEffectorOutput>
    LimbBase<EndEffectorOutput>::LimbBase(
        const std::vector<std::shared_ptr<JointBase>> &joints,
        const LimbConfig &config) 
        : LimbBase("", joints, config) {}

    template<typename EndEffectorOutput>
    void LimbBase<EndEffectorOutput>::UpdateJointStates()
    {
        if(!joint_positions_.valid()){
            // update position/velocity/torque vectors
            joint_positions_.set(joint_utils::PositionsFromJoints(joints_)); 
        }
        fk_.invalidate();
        jac_.invalidate();
    }

    template<typename EndEffectorOutput>
    void LimbBase<EndEffectorOutput>::Update()
    {   
        UpdateJointStates(); // Get joint states

        // If implemented, fk_ and jac_ are set here and 
        // the following two lines should do nothing, 
        // if not function does nothing by default
        FKAndJacobianImpl(joint_positions_.get()); 

        ForwardKinematics(); 
        Jacobian();
    }

    template<typename EndEffectorOutput>
    EndEffectorOutput LimbBase<EndEffectorOutput>::ForwardKinematics()
    {
        if(!fk_.valid()){
            UpdateJointStates(); 
            fk_ = ForwardKinematicsImpl(joint_positions_.get());
        }
        return fk_;
    }

    template<typename EndEffectorOutput>
    Eigen::MatrixXf LimbBase<EndEffectorOutput>::Jacobian()
    {
        if(!jac_.valid()){
            UpdateJointStates();
            jac_ = JacobianImpl(joint_positions_);
        }
        return jac_;
    }

    template<typename EndEffectorOutput>
    std::vector<float> LimbBase<EndEffectorOutput>::InverseKinematics(
                       const EndEffectorOutput &EE_pos)
    {
        return InverseKinematicsImpl(EE_pos);
    }


    template<typename EndEffectorOutput>
    Eigen::VectorXf LimbBase<EndEffectorOutput>::EndEffectorVelocity(){
        EndEffectorVelocityImpl(joint_utils::VelocitiesFromJoints(joints_));
    }

    // Default implementation
    template<typename EndEffectorOutput>
    Eigen::VectorXf LimbBase<EndEffectorOutput>::EndEffectorVelocityImpl( 
        std::vector<float> velocities)
    {
        return Jacobian() * Eigen::Map<Eigen::VectorXf>(velocities.data(),
                                                        velocities.size());
    }


    template<typename EndEffectorOutput>
    void LimbBase<EndEffectorOutput>::InvalidateCache()
    {
        joint_positions_.invalidate();
        fk_.invalidate();
        jac_.invalidate();
    }

    template<typename EndEffectorOutput>
    std::vector<float> LimbBase<EndEffectorOutput>::get_joint_positions()
    {
        return joint_positions_.get();
    }


    template<typename EndEffectorOutput>
    void LimbBase<EndEffectorOutput>::set_joint_positions(
        const std::vector<float> &positions)
    {
        joint_positions_.set(positions);
    }

// Explicit template instantiation
template class LimbBase<Eigen::VectorXf>;
}
