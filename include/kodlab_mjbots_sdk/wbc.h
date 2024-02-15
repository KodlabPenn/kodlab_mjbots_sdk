/** 
 * @file wbc.h
 * @brief Whole Body Controller
 * @author Griffin Addison (gaddison@seas.upenn.edu)
*/

#pragma once 

#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <array>

#include <memory>
#include <iostream>
#include <iomanip>

#include "kodlab_mjbots_sdk/behavior.h" 
#include "kodlab_mjbots_sdk/robot_base.h" 
#include "kodlab_mjbots_sdk/math.h"
#include "kodlab_mjbots_sdk/joint_moteus.h"

// osc stuff
#include "osqp++.h"
#include <Eigen/Sparse>

// rbdl for mass matrix
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include "../addons/urdfreader/urdfreader.h"


template<class Robot = kodlab::RobotBase>
class Wbc {

  private:

    static const unsigned int n_limb = 4; // number of limbs

    static const unsigned int n_fb = 6; // number of floating base joints (unactuated)
    static const unsigned int n_u = 3 * n_limb + 1; // number of actuated joints (limb joints + spine joint)
    static const unsigned int n_vdot = n_fb + n_u; // number of generalized joint velocities (actuated joints + unactuated joints)

    static const unsigned int n_tau = n_u; // number of joint torques
    static const unsigned int n_cf = 3 * n_limb; // number of contact forces

    static const unsigned int n_dc = n_vdot + n_tau + n_cf; // number of decision variables

    static const unsigned int n_ct_fr = 5 * n_limb; // number of friction constraints
    static const unsigned int n_ct = n_vdot + n_cf + n_ct_fr + n_u; // number of constraints


    Eigen::VectorXd q;

    Eigen::VectorXd qdot;

    // Store objective matrix and vector (dense)
    Eigen::MatrixXd objective_matrix = Eigen::MatrixXd::Zero(n_dc, n_dc);
    Eigen::VectorXd objective_vector = Eigen::VectorXd::Zero(n_dc);

    // Store constraint matrix and bounds (dense)
    Eigen::MatrixXd constraint_matrix = Eigen::MatrixXd::Zero(n_ct, n_dc);
    Eigen::VectorXd constraint_lower_bounds = Eigen::VectorXd::Zero(n_ct);
    Eigen::VectorXd constraint_upper_bounds = Eigen::VectorXd::Zero(n_ct);

    Eigen::MatrixXd inertia_matrix;

    Eigen::VectorXd bias_vector;

    Eigen::MatrixXd contact_jacobian_T = Eigen::MatrixXd::Zero(n_vdot, n_cf);

    Eigen::MatrixXd weights = Eigen::MatrixXd::Zero(n_dc, n_dc);

    Eigen::VectorXd command = Eigen::VectorXd::Zero(n_dc);

    Eigen::MatrixXd task_jacobian = Eigen::MatrixXd::Identity(n_dc, n_dc);

    Eigen::MatrixXd selection_matrix_actuated_joints = Eigen::MatrixXd::Zero(n_u, n_vdot);

    Eigen::MatrixXd selection_matrix_rbdl = Eigen::MatrixXd::Zero(n_vdot, n_vdot);

    std::unique_ptr<RigidBodyDynamics::Model> model;

    int dof;

    osqp::OsqpInstance solverInstance;

    osqp::OsqpSolver solver;

    osqp::OsqpSettings settings;

    void updateObjectiveMatrix();

    void updateObjectiveVector();

    void updateContactJacobianT(const Robot& robot_);

    void updateDynamicsConstraint(const Robot& robot_);

    void updateContactConstraint();

    void setFrictionConstraint(float mu, float minimum_normal_force);

    void setInputConstraint(float tau_max);

  
  public:

    Wbc() 
    {

      /// LOAD MODEL
      const char* urdf_path = "/home/griffin/Documents/twist_mjbots/model/twist.urdf";
      model = std::make_unique<RigidBodyDynamics::Model>();
      if (!RigidBodyDynamics::Addons::URDFReadFromFile (urdf_path, model.get(), false)) {
        std::cerr << "Error loading model " << urdf_path << std::endl;
        abort();
      }
      /// LOAD MODEL

      this->dof = model->qdot_size;

      // std::cout << "\n model->qdot_size: " << model->qdot_size << std::endl;
      // std::cout << "\n model->q_size: " << model->q_size << std::endl;

      // Initialize matrix sizes based upon model dof
      q = Eigen::VectorXd::Zero(dof);
      qdot = Eigen::VectorXd::Zero(dof);

      solverInstance.objective_vector.resize(n_dc);
      solverInstance.lower_bounds.resize(n_ct);
      solverInstance.upper_bounds.resize(n_ct);

      inertia_matrix = Eigen::MatrixXd::Zero(dof, dof);

      bias_vector = Eigen::VectorXd::Zero(dof);

      selection_matrix_actuated_joints << Eigen::MatrixXd::Zero(n_u, n_fb), Eigen::MatrixXd::Identity(n_u, n_u); // This just turns joint torques (8x1) into generalized joint torques (14x1)

        Eigen::MatrixXd Identity3x3 = Eigen::Matrix<double, 3, 3>::Identity();
      selection_matrix_rbdl.block<3, 3>(0, 0) = Identity3x3; 
      selection_matrix_rbdl.block<3, 3>(3, 3) = Identity3x3;
      selection_matrix_rbdl.block<3, 3>(7, 7) = Identity3x3;
      selection_matrix_rbdl.block<3, 3>(13, 10) = Identity3x3;
      selection_matrix_rbdl.block<3, 3>(10, 13) = Identity3x3;
      selection_matrix_rbdl.block<3, 3>(16, 16) = Identity3x3;
      selection_matrix_rbdl(6, 18) = 1;

      // std::cout << "\n selection_matrix_rbdl: \n" << selection_matrix_rbdl << std::endl;

      // std::cout << "\n selection_matrix_rbdl.rows(): \n" << selection_matrix_rbdl.rows() 
      //           << "\n selection_matrix_rbdl.cols(): \n" << selection_matrix_rbdl.cols() << std::endl;

      setFrictionConstraint(0.7f, 1.f); // mu = 1, minimum_normal_force = 3N
      setInputConstraint(18.f);

    }



    void setFbAccelCmd(Eigen::VectorXd fb_accel_cmd);
    void setFbAccelCmd(double fb_accel_cmd);

    void setJointAccelCmd(Eigen::VectorXd joint_accel_cmd);
    void setJointAccelCmd(double joint_accel_cmd);

    void setJointTorqueCmd(Eigen::VectorXd joint_torque_cmd);

    void setContactForcesCmd(Eigen::VectorXd contact_forces_cmd);

    void setFbAccelCosts(Eigen::VectorXd fb_accel_costs);
    void setFbAccelCosts(double fb_accel_costs);

    void setJointAccelCosts(Eigen::VectorXd joint_accel_costs);
    void setJointAccelCosts(double joint_accel_costs);

    void setJointTorqueCosts(Eigen::VectorXd joint_torque_costs);
    void setJointTorqueCosts(double joint_torque_costs);

    void setContactForceCosts(Eigen::VectorXd contact_force_costs);

    Eigen::VectorXd updateAndSolve(const Robot& robot_);


};






template<class Robot>
Eigen::VectorXd Wbc<Robot>::updateAndSolve(const Robot& robot_) {

  // First, update contact_jacobian_T
  updateContactJacobianT(robot_);

  // Then, update the dependent constraints
  updateContactConstraint();
  updateDynamicsConstraint(robot_);



  // Solve the QP
  using namespace osqp;

  solverInstance.objective_matrix = this->objective_matrix.sparseView();
  solverInstance.objective_vector = this->objective_vector;
  solverInstance.constraint_matrix = this->constraint_matrix.sparseView();
  solverInstance.lower_bounds = this->constraint_lower_bounds;
  solverInstance.upper_bounds = this->constraint_upper_bounds;

  // print constraint matrix
  // std::cout << "\n constraint_matrix: \n" << std::fixed << std::setprecision(2) << constraint_matrix << std::endl;

  auto status = solver.Init(solverInstance, settings);
  OsqpExitCode exit_code = solver.Solve();
  double optimal_objective = solver.objective_value();
  Eigen::VectorXd optimal_solution = solver.primal_solution();

  return optimal_solution;
}


template <class Robot>
void Wbc<Robot>::updateDynamicsConstraint(const Robot& robot_) {
  
  // Update q and qdot
  for (int i = 0; i < n_u; ++i) {
    q(i + n_fb) = robot_.GetJointPositions()[i];
    qdot(i + n_fb) = robot_.GetJointPositions()[i];
  }

  // Clear the inertia matrix and bias vector
  inertia_matrix.setZero();
  bias_vector.setZero();

  Eigen::VectorXd q_rbdl = selection_matrix_rbdl * q;
  Eigen::VectorXd qdot_rbdl = selection_matrix_rbdl * qdot;

  Eigen::MatrixXd inertia_matrix_rbdl = Eigen::MatrixXd::Zero(dof, dof);
  Eigen::VectorXd bias_vector_rbdl = Eigen::VectorXd::Zero(dof);

  // Update the inertia matrix and bias vector
  CompositeRigidBodyAlgorithm(*model, q_rbdl, inertia_matrix_rbdl, true);
  NonlinearEffects(*model, q_rbdl, qdot_rbdl, bias_vector_rbdl);

  inertia_matrix = selection_matrix_rbdl.transpose() * inertia_matrix_rbdl * selection_matrix_rbdl;
  bias_vector = selection_matrix_rbdl.transpose() * bias_vector_rbdl;

  // Update constraint
  constraint_matrix.middleRows(0, n_vdot) << inertia_matrix, -selection_matrix_actuated_joints.transpose(), -contact_jacobian_T;
  constraint_lower_bounds.segment(0, n_vdot) = -bias_vector;
  constraint_upper_bounds.segment(0, n_vdot) = -bias_vector;

  // print this block to verify
  // std::cout << std::fixed << std::setprecision(2);
  // std::cout << "\n inertia_matrix: \n" << inertia_matrix << std::endl;
  // std::cout << "\n bias_vector: \n" << bias_vector << std::endl;



}

template<class Robot>
void Wbc<Robot>::setFrictionConstraint(float mu, float minimum_normal_force) {

  //// THE FRICTION CONSTRAINT MATRIX ////

  Eigen::MatrixXd friction_constraint_matrix = Eigen::MatrixXd::Zero(n_ct_fr, n_dc);

  Eigen::MatrixXd friction_constraint_matrix_nonzero_block = Eigen::MatrixXd::Zero(n_ct_fr, n_cf); 

  // construct an arbitrary 3x3 block representing friction constraint for 1 toe

  double max_tangential_friction = (std::sqrt(2.0) / 2.0) * mu;

  Eigen::Matrix<double, 5, 3> friction_constraint_matrix_one_toe;

  friction_constraint_matrix_one_toe <<  1,  0,  max_tangential_friction,
                                        1,  0, -max_tangential_friction,
                                        0,  1,  max_tangential_friction,
                                        0,  1, -max_tangential_friction,
                                        0,  0,  1;

  // place four of these blocks along the diagonal (for 4 toes)
  for (int i = 0; i < n_limb; ++i) {
    friction_constraint_matrix_nonzero_block.block<5, 3>(i * 5, i * 3) = friction_constraint_matrix_one_toe; 
  }



  friction_constraint_matrix << Eigen::MatrixXd::Zero(n_ct_fr, n_vdot), Eigen::MatrixXd::Zero(n_ct_fr, n_u), friction_constraint_matrix_nonzero_block;

  // put this into total constraint matrix
  constraint_matrix.middleRows(n_vdot + n_cf, n_ct_fr) = friction_constraint_matrix;



  //// THE FRICTION CONSTRAINT LOWER & UPPER BOUNDS ////

  const double kInfinity = std::numeric_limits<double>::infinity();


  Eigen::VectorXd friction_constraint_lower_bound = Eigen::VectorXd::Zero(n_ct_fr);

  Eigen::Vector5d friction_constraint_lower_bound_one_toe; 
    friction_constraint_lower_bound_one_toe << 0, -kInfinity , 0, -kInfinity, minimum_normal_force; 

  friction_constraint_lower_bound << friction_constraint_lower_bound_one_toe, 
                                      friction_constraint_lower_bound_one_toe, 
                                      friction_constraint_lower_bound_one_toe, 
                                      friction_constraint_lower_bound_one_toe;

  Eigen::VectorXd friction_constraint_upper_bound = Eigen::VectorXd::Zero(n_ct_fr); 

  Eigen::Vector5d friction_constraint_upper_bound_one_toe; 
    friction_constraint_upper_bound_one_toe << kInfinity, 0, kInfinity, 0, kInfinity;

  friction_constraint_upper_bound << friction_constraint_upper_bound_one_toe, 
                                      friction_constraint_upper_bound_one_toe, 
                                      friction_constraint_upper_bound_one_toe, 
                                      friction_constraint_upper_bound_one_toe;


  // put this into total constraint lower and upper bound vectors
  constraint_lower_bounds.segment(n_vdot + n_cf, n_ct_fr) = friction_constraint_lower_bound;
  constraint_upper_bounds.segment(n_vdot + n_cf, n_ct_fr) = friction_constraint_upper_bound;


  // print this block to verify
  // std::cout << "\n friction_constraint_matrix: \n" << friction_constraint_matrix << std::endl;
  // std::cout << "\n friction_constraint_lb: \n" << friction_constraint_lower_bound << std::endl;
  // std::cout << "\n friction_constraint_ub: \n" << friction_constraint_upper_bound << std::endl;

}


template<class Robot>
void Wbc<Robot>::updateContactJacobianT(const Robot& robot_) {

  Eigen::MatrixXd contact_jacobian_from_floating_base_T = Eigen::MatrixXd::Zero(n_fb, n_cf);
  for (int limbNum = 0; limbNum < n_limb; ++limbNum) {

    Eigen::Vector3f p_BTi_B = robot_.get_kinematics().toe_positions.template segment<3>(limbNum * 3); // need .template cuz otherwise the compiler thinks its operater< smh

    Eigen::Vector3f p_TiB_B = -p_BTi_B; // need position vector from toe_i to the body origin
    Eigen::MatrixXf adjointTransform_float = kodlab::geometry::AdjointTransformation(Eigen::Matrix3f::Identity(), p_TiB_B).transpose();
    Eigen::MatrixXd adjointTransform = adjointTransform_float.cast<double>();
    contact_jacobian_from_floating_base_T.block(0, limbNum * 3, n_fb, 3) = adjointTransform.block(0, 0, n_fb, 3); 
  }
  contact_jacobian_T.block(0, 0, n_fb, n_cf) = contact_jacobian_from_floating_base_T;

  Eigen::MatrixXf toe_jacobian_T_float = robot_.get_kinematics().toe_jacobian.transpose();
  Eigen::MatrixXd toe_jacobian_T = toe_jacobian_T_float.cast<double>();
  contact_jacobian_T.block(n_fb, 0, n_u, n_cf) = toe_jacobian_T;

  // TODO: twist conversion: check this whole updateContactJacobianT function

  // print this block to verify
  // std::cout << "\n contact_jacobian_T: \n" << contact_jacobian_T << std::endl;

}

template<class Robot>
void Wbc<Robot>::updateContactConstraint(){

  // update contact constraint matrix (which is just a certain location in the constraint_matrix)
  constraint_matrix.middleRows(n_vdot, n_cf) << contact_jacobian_T.transpose(), Eigen::MatrixXd::Zero(n_cf, n_u), Eigen::MatrixXd::Zero(n_cf, n_cf);

  // put this into total constraint lower and upper bound vectors
  constraint_lower_bounds.segment(n_vdot, n_cf) = Eigen::VectorXd::Zero(n_cf); //an equality constraint
  constraint_upper_bounds.segment(n_vdot, n_cf) = Eigen::VectorXd::Zero(n_cf);

}

template<class Robot>
void Wbc<Robot>::setInputConstraint(float tau_max) {

  //// THE INPUT/TORQUE LIMIT CONSTRAINT MATRIX ////

  Eigen::MatrixXd input_constraint_matrix = Eigen::MatrixXd::Zero(n_u, n_dc);

  Eigen::MatrixXd input_constraint_matrix_nonzero_block = Eigen::MatrixXd::Identity(n_u, n_u);

  input_constraint_matrix.block(0, n_vdot, n_u, n_u) = input_constraint_matrix_nonzero_block;

  // put this into total constraint matrix
  constraint_matrix.middleRows(n_vdot + n_cf + n_ct_fr, n_u) = input_constraint_matrix;


  //// THE INPUT/TORQUE LIMIT CONSTRAINT LOWER & UPPER BOUNDS ////

  Eigen::VectorXd input_constraint_lower_bound = -tau_max * Eigen::VectorXd::Ones(n_u);

  Eigen::VectorXd input_constraint_upper_bound = tau_max * Eigen::VectorXd::Ones(n_u);


  // put this into total constraint lower and upper bound vectors
  constraint_lower_bounds.segment(n_vdot + n_cf + n_ct_fr, n_u) = input_constraint_lower_bound;
  constraint_upper_bounds.segment(n_vdot + n_cf + n_ct_fr, n_u) = input_constraint_upper_bound;


}

template<class Robot>
void Wbc<Robot>::updateObjectiveMatrix() {

  this->objective_matrix = 2 * task_jacobian.transpose() * weights * task_jacobian; // Q = 2J^T * W * J
  this->objective_matrix += Eigen::MatrixXd::Identity(n_dc, n_dc) * 1e-6; // add a small diagonal term to make P positive definite

}

template<class Robot>
void Wbc<Robot>::updateObjectiveVector() {

  this->objective_vector = -2 * task_jacobian.transpose() * weights * command; // b = - 2 * command^T * W * J

}

template<class Robot>
void Wbc<Robot>::setFbAccelCmd(Eigen::VectorXd fb_accel_cmd) {

  command.segment(0, n_fb) = fb_accel_cmd;

  Wbc::updateObjectiveVector();

}

template<class Robot>
void Wbc<Robot>::setFbAccelCmd(double fb_accel_cmd) {

  Wbc::setFbAccelCmd(Eigen::VectorXd::Constant(n_fb, fb_accel_cmd));


}

template<class Robot>
void Wbc<Robot>::setJointAccelCmd(Eigen::VectorXd joint_accel_cmd) {

  command.segment(n_fb, n_u) = joint_accel_cmd;

  Wbc::updateObjectiveVector();

}

template<class Robot>
void Wbc<Robot>::setJointAccelCmd(double joint_accel_cmd) {
  
    Wbc::setJointAccelCmd(Eigen::VectorXd::Constant(n_u, joint_accel_cmd));
  
    Wbc::updateObjectiveVector();
}


template<class Robot>
void Wbc<Robot>::setJointTorqueCmd(Eigen::VectorXd joint_torque_cmd) {

  command.segment(n_vdot, n_u) = joint_torque_cmd;

  Wbc::updateObjectiveVector();

}

template<class Robot>
void Wbc<Robot>::setContactForcesCmd(Eigen::VectorXd contact_forces_cmd) {

  command.segment(n_vdot + n_u, n_cf) = contact_forces_cmd;

  Wbc::updateObjectiveVector();

}

template<class Robot>
void Wbc<Robot>::setFbAccelCosts(Eigen::VectorXd fb_accel_costs) {

  weights.block(0, 0, n_fb, n_fb) = fb_accel_costs.asDiagonal();

  Wbc::updateObjectiveMatrix();
  Wbc::updateObjectiveVector();

}
template<class Robot>
void Wbc<Robot>::setFbAccelCosts(double fb_accel_costs) {

  Wbc::setFbAccelCosts(Eigen::Vector<double, n_fb>::Constant(fb_accel_costs));

}

template<class Robot>
void Wbc<Robot>::setJointAccelCosts(Eigen::VectorXd joint_accel_costs) {

  weights.block(n_fb, n_fb, n_u, n_u) = joint_accel_costs.asDiagonal();

  Wbc::updateObjectiveMatrix();
  Wbc::updateObjectiveVector();

}

template<class Robot>
void Wbc<Robot>::setJointAccelCosts(double joint_accel_costs) {

  Wbc::setJointAccelCosts(Eigen::VectorXd::Constant(n_u, joint_accel_costs));

}

template<class Robot>
void Wbc<Robot>::setJointTorqueCosts(Eigen::VectorXd joint_torque_costs) {

  weights.block(n_vdot, n_vdot, n_u, n_u) = joint_torque_costs.asDiagonal();

  Wbc::updateObjectiveMatrix();
  Wbc::updateObjectiveVector();

}

template<class Robot>
void Wbc<Robot>::setJointTorqueCosts(double joint_torque_costs) {

  Wbc::setJointTorqueCosts(Eigen::VectorXd::Constant(n_u, joint_torque_costs));

}

template<class Robot>
void Wbc<Robot>::setContactForceCosts(Eigen::VectorXd contact_force_costs) {

  weights.block(n_vdot + n_u, n_vdot + n_u, n_cf, n_cf) = contact_force_costs.asDiagonal();

  Wbc::updateObjectiveMatrix();
  Wbc::updateObjectiveVector();

}


