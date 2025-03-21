/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "rotors_control/lee_payload_position_controller.h"

namespace rotors_control {

LeePayloadPositionController::LeePayloadPositionController()
    : initialized_params_(false),
      controller_active_(false) {
  InitializeParameters();
}

LeePayloadPositionController::~LeePayloadPositionController() {}

void LeePayloadPositionController::InitializeParameters() {
  calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, &(controller_parameters_.allocation_matrix_));
  // To make the tuning independent of the inertia matrix we divide here.
  normalized_attitude_gain_ = controller_parameters_.attitude_gain_.transpose()
      * vehicle_parameters_.inertia_.inverse();
  // To make the tuning independent of the inertia matrix we divide here.
  normalized_angular_rate_gain_ = controller_parameters_.angular_rate_gain_.transpose()
      * vehicle_parameters_.inertia_.inverse();

  Eigen::Matrix4d I;
  I.setZero();
  I.block<3, 3>(0, 0) = vehicle_parameters_.inertia_;
  I(3, 3) = 1;
  angular_acc_to_rotor_velocities_.resize(vehicle_parameters_.rotor_configuration_.rotors.size(), 4);
  // Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia matrix I.
  // A^{ \dagger} = A^T*(A*A^T)^{-1}
  angular_acc_to_rotor_velocities_ = controller_parameters_.allocation_matrix_.transpose()
      * (controller_parameters_.allocation_matrix_
      * controller_parameters_.allocation_matrix_.transpose()).inverse() * I;
  initialized_params_ = true;
}

void LeePayloadPositionController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const {	
  assert(rotor_velocities);
  assert(initialized_params_);

  rotor_velocities->resize(vehicle_parameters_.rotor_configuration_.rotors.size());
  // Return 0 velocities on all rotors, until the first command is received.
  if (!controller_active_) {
    *rotor_velocities = Eigen::VectorXd::Zero(rotor_velocities->rows());
    return;
  }

  Eigen::Vector3d acceleration;
  ComputeDesiredAcceleration(&acceleration);

  Eigen::Vector3d angular_acceleration;
  ComputeDesiredAngularAcc(acceleration, &angular_acceleration);

  // Project thrust onto body z axis.
  double thrust = -vehicle_parameters_.mass_ * acceleration.dot(mav_odometry_.orientation.toRotationMatrix().col(2));

  Eigen::Vector4d angular_acceleration_thrust;
  angular_acceleration_thrust.block<3, 1>(0, 0) = angular_acceleration;
  angular_acceleration_thrust(3) = thrust;

  *rotor_velocities = angular_acc_to_rotor_velocities_ * angular_acceleration_thrust;
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();
}

void LeePayloadPositionController::SetMavOdometry(const EigenOdometry& odometry) {
  mav_odometry_ = odometry;
}

void LeePayloadPositionController::SetPayloadOdometry(const EigenOdometry& odometry) {
  payload_odometry_ = odometry;
}

void LeePayloadPositionController::SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& _command_trajectory) {
  //command_trajectory_ = command_trajectory;
  //controller_active_ = true;
}

void LeePayloadPositionController::SetMavPayloadTrajectory(
    const mav_msgs::EigenTrajectoryPointDeque& _command_trajectory) {
  command_trajectory_ = _command_trajectory.front(); //mav
  command_payload_trajectory_ = _command_trajectory.back(); //payload
  controller_active_ = true;
}

// Implementation from the T. Lee et al. paper
// Geometric Control and Differential Flatness of a Quadrotor UAV with a Cable-Suspended Load
void LeePayloadPositionController::ComputeDesiredAcceleration(Eigen::Vector3d* acceleration) const {
  assert(acceleration);

 	Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());

  Eigen::Vector3d q, q_des;         //the measure/desired unit vector from quarotor to the load
	Eigen::Vector3d q_rate_des;
  //test
  //std::cout<<"Mav Trajectory (in controller):" << std::endl;
  //std::cout<<" desired_position:" << command_trajectory_.position_W << std::endl;
  //std::cout<<" desired_velocity:" << command_trajectory_.velocity_W << std::endl;
  //std::cout<<" desired_ori:" << command_trajectory_.orientation_W_B.toRotationMatrix() << std::endl;
  //std::cout<<" desired_angular_velocity:" << command_trajectory_.angular_velocity_W << std::endl;
  //test
  //std::cout<<"Payload Trajectory (in controller):" << std::endl;
  //std::cout<<" desired_position:" << command_payload_trajectory_.position_W << std::endl;
  //std::cout<<" desired_velocity:" << command_payload_trajectory_.velocity_W << std::endl;
  //std::cout<<" desired_ori:" << command_payload_trajectory_.orientation_W_B.toRotationMatrix() << std::endl;
  //std::cout<<" desired_angular_velocity:" << command_payload_trajectory_.angular_velocity_W << std::endl;

  Eigen::Vector3d position_error;
  position_error = mav_odometry_.position - command_trajectory_.position_W;
  
  // Transform velocity to world frame.
  const Eigen::Matrix3d R_W_I = mav_odometry_.orientation.toRotationMatrix();
  Eigen::Vector3d velocity_W =  R_W_I * mav_odometry_.velocity;
  Eigen::Vector3d velocity_error;
  velocity_error = velocity_W - command_trajectory_.velocity_W;
	// Transform payload velocity to world frame.
	Eigen::Vector3d payload_velocity_W =  payload_odometry_.orientation.toRotationMatrix() * payload_odometry_.velocity;

	// Force resultant design
	Eigen::Matrix3d q_matrix;
  Eigen::Vector3d q_rate, px_des, pv_des, pa_des; 
  Eigen::Vector3d q_acc_des; 
	Eigen::Vector3d q_error, q_rate_error, px_error, pv_error;
	Eigen::Vector3d w;                //Rotate rate
	Eigen::Vector3d A;
	Eigen::Vector3d F_n, F_rt;
	Eigen::Vector3d F_pd;
	Eigen::Vector3d F_ff, F_g;
	double p_mass(0.2);                          //payload mass
	double l(0.75);                              //cable length
	const Eigen::Vector3d kx = Eigen::Vector3d(2.0, 2.0, 1.5);
  const Eigen::Vector3d kv = Eigen::Vector3d(2.5, 2.5, 2.5);
  const Eigen::Vector3d kq = Eigen::Vector3d(2.5, 2.5, 1.5);
  const Eigen::Vector3d kw = Eigen::Vector3d(2.5, 2.5, 2.5);  //p d controller parameters
	const Eigen::Vector3d kmx = Eigen::Vector3d(1.0, 1.0, 1.0);
  const Eigen::Vector3d kmv = Eigen::Vector3d(1.5, 1.5, 2.5);

	px_des = command_payload_trajectory_.position_W;  //payload position and velocity
	pv_des = command_payload_trajectory_.velocity_W;  
	pa_des.setZero();
	q_acc_des.setZero();
	q = payload_odometry_.position - mav_odometry_.position;
	q = q/q.norm();
	w = (payload_velocity_W - velocity_W) / (payload_odometry_.position - mav_odometry_.position).norm();
	w = q.cross(w);
	q_rate = w.cross(q); 
  skewMatrixFromVector(q, &q_matrix);
	px_error = payload_odometry_.position - px_des;
	pv_error = payload_velocity_W - pv_des;

	F_g << 0, 0, -1 * (vehicle_parameters_.mass_ + p_mass) * vehicle_parameters_.gravity_;
	A = - px_error.cwiseProduct(kx) - pv_error.cwiseProduct(kv) + (vehicle_parameters_.mass_ + p_mass) * pa_des
  	       - F_g + vehicle_parameters_.mass_ * l * (q_rate.dot(q_rate) * q);  //F_g is very important here
  A += - position_error.cwiseProduct(kmx) - velocity_error.cwiseProduct(kmv);
	//  	- F_g; //F_g is very important here
	q_rate_error.setZero();
	//q_des = -1 * A/A.norm();
	q_des = command_payload_trajectory_.orientation_W_B.toRotationMatrix().transpose() * (-e_3); 
	q_rate_des = command_payload_trajectory_.angular_velocity_W.cross(q_des); 
	q_error = q_matrix * q_matrix * q_des;
	q_rate_error = q_rate - (q_des.cross(q_rate_des)).cross(q);
	//if((command_payload_trajectory_.orientation_W_B.toRotationMatrix() * (-e_3))(2) == -1)
	F_n = A;
	//else
	//F_n = A.dot(q) * q;
	F_pd = - q_error.cwiseProduct(kq) - q_rate_error.cwiseProduct(kw);
	F_ff = vehicle_parameters_.mass_ * l * q.dot(q_des.cross(q_rate_des)) * q.cross(q_rate)
	        + vehicle_parameters_.mass_ * l * (q_des.cross(q_acc_des)).cross(q);
	
	F_rt = F_n - F_pd - F_ff;

	*acceleration= -1 * F_rt / vehicle_parameters_.mass_;
}

void LeePayloadPositionController::ComputeDesiredAngularAcc(const Eigen::Vector3d& acceleration,
                                                     Eigen::Vector3d* angular_acceleration) const {
  assert(angular_acceleration);

  Eigen::Matrix3d R = mav_odometry_.orientation.toRotationMatrix(); //There are EigenOdometry type in mav_msgs and rotor_control (confused)
  // Get the desired rotation matrix.
  Eigen::Vector3d b1_des;
  b1_des << cos(command_trajectory_.getYaw()), sin(command_trajectory_.getYaw()), 0;

  Eigen::Vector3d b3_des;
  b3_des = -1 * acceleration / acceleration.norm();

  Eigen::Vector3d b2_des;
  b2_des = -1 * b3_des.cross(b3_des.cross(b1_des));
  b2_des.normalize();

  Eigen::Matrix3d R_des;
  R_des.col(0) = b2_des;
  R_des.col(1) = b3_des.cross(b1_des);
  R_des.col(2) = b3_des;

  // Angle error according to lee et al.
  Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
  Eigen::Vector3d angle_error;
  vectorFromSkewMatrix(angle_error_matrix, &angle_error);

  // TODO(burrimi) include angular rate references at some point.
  Eigen::Vector3d   (Eigen::Vector3d::Zero());
  angular_rate_des[2] = command_trajectory_.getYawRate();

  Eigen::Vector3d angular_rate_error = mav_odometry_.angular_velocity - R_des.transpose() * R * angular_rate_des;

  *angular_acceleration = -1 * angle_error.cwiseProduct(normalized_attitude_gain_)
                           - angular_rate_error.cwiseProduct(normalized_angular_rate_gain_)
                           + mav_odometry_.angular_velocity.cross(mav_odometry_.angular_velocity); // we don't need the inertia matrix here
}
}
