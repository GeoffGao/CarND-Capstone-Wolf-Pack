/**
 * created by Ye Bo
 * date: 2017-12-05
 */

#ifndef TRAJECTORY_FOLLOWER_CONFIG_H__
#define TRAJECTORY_FOLLOWER_CONFIG_H__

// ROS includes
#include <ros/console.h>
#include <ros/ros.h>

// math includes
#include <eigen3/Eigen/Core>

// user defined includes
#include "trajectory_follower/TrajectoryFollowerReconfigureConfig.h"
//#include <dbw_mkz_twist_controller/ControllerConfig.h>
// boost class
#include <boost/thread.hpp>
namespace trajectory_follower {
/**
 * @class  TrajectoryFollowerConfig
 * @brief  Config class for the trajectory_follower and its components
 */
class TrajectoryFollowerConfig {
 public:
  struct MPC {
    bool pub_pedals;
    bool pub_steering;
    double decel_max;
    double accel_max;
    double max_lat_accel;
    double steer_kp;
    double wheel_radius;
    double vehicle_mass;
    double fuel_capacity;
    double speed_kp;
    double accel_kp;
    double accel_ki;
    double brake_deadband;
    double accel_tau;
    double LF;
    double vel_command;

    int coeff_cte;
    int coeff_epsi;
    int coeff_delta;
    int coeff_vel;
    int coeff_accel;
    int coeff_vel_delta;
    int coeff_diff_vel;
    int coeff_diff_accel;
    int coeff_diff_delta;
    int coeff_area;
    double kappa_for_vel;
    double kappa_for_diff_delta;
    double kappa_for_delta;
    double kappa_for_area;

    double steer_lowerbound;
    double steer_upperbound;
    double vel_lowerbound;
    double vel_upperbound;
    double accel_lowerbound;
    double accel_upperbound;
    double area_lowerbound;
    double area_upperbound;

    double delta_constraints_lowerbound;
    double delta_constraints_upperbound;
    double vel_constraints_lowerbound;
    double vel_constraints_upperbound;
    double accel_constraints_lowerbound;
    double accel_constraints_upperbound;

    bool is_latency;
    double speed_up;

    double lookingforward_time;
    double lookingforward_distance_lowerbound;
    double lookingforward_distance_upperbound;
    
    int kappa_number;
    double dead_zone;
    double regularized_constant;
    double heading_compensation;

    
    double kappa_threshold;
    double cte_threshold;
    double epsi_threshold;
    
    double kappa_for_vel_upperbound;
    double cte_for_vel_upperbound;
    double epsi_for_vel_upperbound;

  } configMPC;

  struct Purepursuit {
    double coeff_max_brake_acceleration;
    double coeff_brake_distance;
    double coeff_min_turn_radius;
  } configPP;

  /**
   * @brief  Construct the TrajectoryFollowerConfig using default values
   * @warning  if the rosparam server or dynamic_reconfigure are used, the default value will be overwritten;
   * parameter are loade in prior order as 1. rosparam server; 2. dynamic_reconfigure; 3.TrajectoryFollowerConfig
   * Constructor
   */
  TrajectoryFollowerConfig() {
    // MPC controller config
    configMPC.pub_pedals = true;
    configMPC.pub_steering = true;
    configMPC.decel_max = 3.0;
    configMPC.accel_max = 3.0;
    configMPC.max_lat_accel = 8.0;
    configMPC.steer_kp = 0.0;
    configMPC.wheel_radius = 0.33;
    configMPC.vehicle_mass = 1736.35;
    configMPC.fuel_capacity = 13.5;
    configMPC.speed_kp = 2.0;
    configMPC.accel_kp = 0.4;
    configMPC.accel_ki = 0.1;
    configMPC.brake_deadband = 0.1;
    configMPC.accel_tau = 0.5;
    configMPC.LF = 2.849;
    configMPC.vel_command = 2.78;//10Km/h

    // Weight of variables
    configMPC.coeff_cte = 1000;
    configMPC.coeff_epsi = 200;
    configMPC.coeff_vel = 1000;
    configMPC.coeff_delta = 2000;
    configMPC.coeff_accel = 0;
    configMPC.coeff_vel_delta = 250;
    configMPC.coeff_diff_vel = 100;
    configMPC.coeff_diff_delta = 1500;
    configMPC.coeff_area = 1000;
    configMPC.kappa_for_area = 0.2;
    configMPC.kappa_for_vel = 1.0;
    configMPC.kappa_for_diff_delta = 1.0;
    configMPC.kappa_for_delta = 1.0;

    // Boundary of varibles
    configMPC.steer_lowerbound = -0.25;  // rad
    configMPC.steer_upperbound = 0.25;
    configMPC.vel_lowerbound = 0.2;  // m/s
    configMPC.vel_upperbound = 3.0;
    configMPC.accel_lowerbound = -0.05;
    configMPC.accel_upperbound = 0.05;
    configMPC.area_lowerbound = 1;
    configMPC.area_upperbound = -1;
    // Boundary of constrainst
    configMPC.delta_constraints_lowerbound = -0.01;
    configMPC.delta_constraints_upperbound = 0.01;
    configMPC.vel_constraints_lowerbound = -0.1;
    configMPC.vel_constraints_upperbound = 0.1;
    configMPC.accel_constraints_lowerbound = -0.1;
    configMPC.accel_constraints_upperbound = 0.1;
    // Valve and other parameters
    configMPC.is_latency = true;
    configMPC.speed_up = 0;               // 0 m/s
    /*TODO: try to figure out a best config in looking forward mode*/
    configMPC.lookingforward_time = 1.5;  // 1 s
    configMPC.lookingforward_distance_lowerbound = 2.0;//0.8
    configMPC.lookingforward_distance_upperbound = 50.0;//90km/h

    configMPC.kappa_number = 5;//10
    configMPC.dead_zone = 0.02;
    configMPC.regularized_constant = 2.0;
    configMPC.heading_compensation = -0.04;

    configMPC.kappa_threshold = 1.5;
    configMPC.cte_threshold = 3.0;   // 10 cm
    configMPC.epsi_threshold = 1.0;  // 15 degree

    configMPC.kappa_for_vel_upperbound = 0.2;
    configMPC.cte_for_vel_upperbound = 0.5;
    configMPC.epsi_for_vel_upperbound = 0.5;

  }
  /**
   * @brief Reconfigure parameters from dynamic_reconfigure
   * A reconfigure server needs to be instaneiated that calls this method in its callback
   */
  void reconfigure(TrajectoryFollowerReconfigureConfig& cfg);

  //void reconfigure1(dbw_mkz_twist_controller::ControllerConfig& cfg);//for mkz info reconf

  void checkParameters();

  /**
   * @brief  Return the internal config mutex
   */
  boost::mutex& configMutex() {
    return config_mutex_;
  }

 private:
  boost::mutex config_mutex_;  //!< Mutex for config accesses and changes
};
}  // namespace trajectory_follower

#endif  // TRAJECTORY_FOLLOWER_CONFIG_H__
