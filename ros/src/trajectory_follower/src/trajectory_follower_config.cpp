/**
 * created by Ye Bo
 * date: 2017-12-05
 */

#include "trajectory_follower/trajectory_follower_config.h"

namespace trajectory_follower {
	void TrajectoryFollowerConfig::reconfigure(TrajectoryFollowerReconfigureConfig &cfg)
	{
		boost::mutex::scoped_lock l(config_mutex_);
		ROS_WARN("trajectory reconfigure!");
		//MPC control config
		configMPC.pub_pedals = cfg.pub_pedals;
        configMPC.pub_steering = cfg.pub_steering;
        configMPC.decel_max = cfg.decel_max;
        configMPC.accel_max = cfg.accel_max;
        configMPC.max_lat_accel = cfg.max_lat_accel;
    	configMPC.steer_kp = cfg.steer_kp;
    	configMPC.wheel_radius = cfg.wheel_radius;
    	configMPC.vehicle_mass = cfg.vehicle_mass;
    	configMPC.fuel_capacity = cfg.fuel_capacity;
    	configMPC.speed_kp = cfg.speed_kp;
    	configMPC.accel_kp = cfg.accel_kp;
    	configMPC.accel_ki = cfg.accel_ki;
    	configMPC.brake_deadband = cfg.brake_deadband;
    	configMPC.accel_tau = cfg.accel_tau;
    	configMPC.LF = cfg.LF;
		// Weight of variables
		configMPC.coeff_cte = cfg.coeff_cte;
		configMPC.coeff_epsi = cfg.coeff_epsi;
		configMPC.coeff_vel = cfg.coeff_vel;
		configMPC.coeff_delta = cfg.coeff_delta;
		configMPC.coeff_accel = cfg.coeff_accel;
		configMPC.coeff_vel_delta = cfg.coeff_vel_delta;
		configMPC.coeff_diff_delta = cfg.coeff_diff_delta;
		configMPC.coeff_diff_vel = cfg.coeff_diff_vel;
		configMPC.coeff_area = cfg.coeff_area;
		configMPC.kappa_for_area = cfg.kappa_for_area;
		configMPC.kappa_for_vel = cfg.kappa_for_vel;
		configMPC.kappa_for_delta = cfg.kappa_for_delta;
		configMPC.kappa_for_diff_delta = cfg.kappa_for_diff_delta;

		// boundary of variables
		configMPC.steer_lowerbound = cfg.steer_lowerbound;
		configMPC.steer_upperbound = cfg.steer_upperbound;
		configMPC.vel_lowerbound = cfg.vel_lowerbound;
		configMPC.vel_upperbound = cfg.vel_upperbound;
		configMPC.accel_lowerbound = cfg.accel_lowerbound;
		configMPC.accel_upperbound = cfg.accel_upperbound;
		configMPC.area_lowerbound = cfg.area_lowerbound;
		configMPC.area_upperbound = cfg.area_upperbound;
		
		// Boundary of constraints
		configMPC.delta_constraints_lowerbound = cfg.delta_constraints_lowerbound;
		configMPC.delta_constraints_upperbound = cfg.delta_constraints_upperbound;
		configMPC.vel_constraints_lowerbound = cfg.vel_constraints_lowerbound;
		configMPC.vel_constraints_upperbound = cfg.vel_constraints_upperbound;
		configMPC.accel_constraints_lowerbound = cfg.accel_constraints_lowerbound;
		configMPC.accel_constraints_upperbound = cfg.accel_constraints_upperbound;

		// Valve and other parameters
		configMPC.is_latency = cfg.is_latency;
		configMPC.speed_up = cfg.speed_up;
		configMPC.lookingforward_time = cfg.lookingforward_time;
		configMPC.lookingforward_distance_lowerbound = cfg.lookingforward_distance_lowerbound;
		configMPC.lookingforward_distance_upperbound = cfg.lookingforward_distance_upperbound;
		configMPC.kappa_number = cfg.kappa_number;
		configMPC.dead_zone = cfg.dead_zone;
		configMPC.regularized_constant = cfg.regularized_constant;
		
		// threshold for steering boundary
		configMPC.kappa_threshold = cfg.kappa_threshold;
		configMPC.cte_threshold = cfg.cte_threshold;
		configMPC.epsi_threshold = cfg.epsi_threshold;


		configMPC.kappa_for_vel_upperbound = cfg.kappa_for_vel_upperbound;
		configMPC.cte_for_vel_upperbound = cfg.cte_for_vel_upperbound;
		configMPC.epsi_for_vel_upperbound = cfg.epsi_for_vel_upperbound;
	}
}


