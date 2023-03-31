/**
 * @file limb_base.cpp
 * @author Kodlab - Zac Gong (zacgong@seas.upenn.edu)
 * @author Kodlab - J. Diego Caporale (jdcap@seas.upenn.edu)
 * @brief Class implementation for LimbBase.
 * @date 2022-08-18
 *
 * @copyright BSD 3-Clause License, Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
 *
 */

#include "kodlab_mjbots_sdk/limb_base.h"

namespace kodlab
{
    template<typename EndEffectorOutput>
    LimbBase<EndEffectorOutput>::LimbBase(const std::string &name,
                       const std::vector<std::shared_ptr<JointBase>> &joints,
                       const LimbConfig &config)
    {
        joints_ = joints;
        config_ = config;
        limbDOFs_ = joints.size();
    }

    template<typename EndEffectorOutput>
    LimbBase<EndEffectorOutput>::LimbBase(const std::vector<std::shared_ptr<JointBase>> &joints,
                       const LimbConfig &config)
    {
        joints_ = joints;
        config_ = config;
    }

    template<typename EndEffectorOutput>
    void LimbBase<EndEffectorOutput>::UpdateJointStates()
    {
        if(!joint_cache_bool_){
            for (int i = 0; i < joints_.size(); i++)
            {
                // update position/velocity/torque vectors
                joint_positions_[i] = joints_[i]->get_position();
                joint_velocities_[i] = joints_[i]->get_velocity();
            }
            joint_cache_bool_ = true;
        }
        fk_.invalidate();
        jac_.invalidate();
    }

    template<typename EndEffectorOutput>
    void LimbBase<EndEffectorOutput>::Update()
    {   
        joint_cache_bool_ = false; 
        UpdateJointStates(); // Get joint states
        FKAndJacobianImpl(); // If implemented, fk_ and jac_ are set here and the following two would do nothing
        ForwardKinematics(); 
        Jacobian();
    }

    template<typename EndEffectorOutput>
    EndEffectorOutput LimbBase<EndEffectorOutput>::ForwardKinematics()
    {
        if(!fk_.valid()){
            UpdateJointStates(); 
            fk_ = ForwardKinematicsImpl();
        }
        return fk_;
    }

    template<typename EndEffectorOutput>
    Eigen::MatrixXf LimbBase<EndEffectorOutput>::Jacobian()
    {
        if(!jac_.valid()){
            UpdateJointStates();
            jac_ = JacobianImpl();
        }
        return jac_;
    }

    template<typename EndEffectorOutput>
    void LimbBase<EndEffectorOutput>::InvalidateCache()
    {
        joint_cache_bool_ = false;
        fk_.invalidate();
        jac_.invalidate();
    }

    template<typename EndEffectorOutput>
    std::vector<float> LimbBase<EndEffectorOutput>::get_joint_positions()
    {
        return joint_positions_;
    }

    template<typename EndEffectorOutput>
    std::vector<float> LimbBase<EndEffectorOutput>::get_joint_velocities()
    {
        return joint_velocities_;
    }

    template<typename EndEffectorOutput>
    void LimbBase<EndEffectorOutput>::set_joint_positions(const std::vector<float> &positions)
    {
        for (int i = 0; i < LimbBase::joints_.size(); i++)
        {
            // joints_[i]->set_position(positions[i]);
            joint_positions_[i] = positions[i];
        }
    }

    template<typename EndEffectorOutput>
    void LimbBase<EndEffectorOutput>::set_joint_velocities(const std::vector<float> &velocities)
    {
        for (int i = 0; i < LimbBase::joints_.size(); i++)
        {
            // joints_[i]->set_velocity(velocities[i]);
            joint_velocities_[i] = velocities[i];
        }
    }

// Explicit template instantiation
template class LimbBase<Eigen::VectorXf>;
}
