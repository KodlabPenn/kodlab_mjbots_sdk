/**
 * @file simple_limb.h
 * @author Kodlab - Zac Gong (zacgong@seas.upenn.edu)
 * @author Kodlab - J. Diego Caporale (jdcap@seas.upenn.edu)
 * @brief A simple LimbBase derived class example, a 1DOF limb with 1 joint at
 *        the origin and a end effector
 * @date 2022-09-02
 *
 * @copyright (c) Copyright 2022 The Trustees of the University of Pennsylvania
 *
 */

#pragma once

#include <Eigen/Dense>

#include "kodlab_mjbots_sdk/limb_base.h"

/**
 * @brief A simple 1-DOF limb consisting of a single joint with a striaght
 * link to an end-effector in Cartesian XYZ space
 * @details A simple, single-degree-of-freedom limb implementation demonstrating 
 * the specification of a Cartesian offset and implementation of kinematics
 * methods
 * @todo Implement running example
 */
class SimpleLimb : public kodlab::LimbBase<Eigen::Vector3f> {

  public:
    using kodlab::LimbBase<Eigen::Vector3f>::LimbBase;
    Eigen::Vector3f ForwardKinematicsImpl(
        std::vector<float>joint_positions) override 
    {
        float theta = joint_positions[0];
        Eigen::Vector3f fk;

        fk << config_.actuator_offsets[0].x() * cos(theta),
                config_.actuator_offsets[0].y() * sin(theta),
                config_.actuator_offsets[0].z();
        return fk;
    }

    Eigen::MatrixXf JacobianImpl(
        std::vector<float>joint_positions) override 
    {
        Eigen::MatrixXf jac(3,1);
        float theta = joint_positions[0];

        jac << -config_.actuator_offsets[0].x() * sin(theta),
                config_.actuator_offsets[0].y() * cos(theta),
                0;
        return jac;
    }

    std::vector<float> InverseKinematicsImpl(
        const Eigen::Vector3f &EE_pos) override 
    {
        std::vector<float> pos;
        float theta = atan2(EE_pos.y(), EE_pos.x());
        pos.push_back(theta);

        return pos;
    }

    void FKAndJacobianImpl(
        std::vector<float> joint_positions) override
    {
        // Optimized implementation for FK and Jac
        float s_theta;
        float c_theta;
        float theta = joint_positions[0];
        sincosf(theta, &s_theta, &c_theta);
        Eigen::Vector3f fk_multi;
        fk_multi << c_theta, s_theta, 1;
        Eigen::Vector3f jac_multi;
        jac_multi << -s_theta, c_theta, 0;

        // Set cached internal variables
        fk_.set( config_.actuator_offsets[0].cwiseProduct(fk_multi) );
        jac_.set( config_.actuator_offsets[0].cwiseProduct(jac_multi) );
    } 
};