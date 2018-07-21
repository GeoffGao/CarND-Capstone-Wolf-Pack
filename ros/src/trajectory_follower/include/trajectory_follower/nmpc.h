/********************************************************
*   Copyright (C) 2018 All rights reserved.
*
*   Filename: nmpc.h
*   Author  : junlong.gao
*   Date    : June 1, 2018
*   Describe:
*
********************************************************/
#ifndef NMPC_INCLUDE_NMPC_H_
#define NMPC_INCLUDE_NMPC_H_
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

typedef CPPAD_TESTVECTOR(double) Dvector;

const int NUMBER_OF_STATES = 5; 	// px, py, psi, v
const int NUMBER_OF_ACTUATIONS = 2; // steering angle, acceleration

struct NMPCParam {
	double Lf;
	double dt;
	double W_dist_lateral;
	double W_dist_vertical;
	double W_epsi;
	double W_delta_acc;
	double W_delta_steer;
	double W_speed;
	double W_steer;
	double num_advanced;
	double acc_argument;
	int N; 					// prediction horizon
};

class NMPC {
public:
	NMPC();
	~NMPC();
	bool solve(autogo_msgs::TrajectoryPoint pose,
			  std::vector<autogo_msgs::TrajectoryPoint> traj,
			  Eigen::VectorXd lastAct,
			  NMPCParam param,
			  std::vector<double>& result);
	bool init_params(const NMPCParam& param_init);
	Dvector solution;
private:
	bool hasSolution;
	NMPCParam param;
};

#endif /* NMPC_INCLUDE_NMPC_H_ */
