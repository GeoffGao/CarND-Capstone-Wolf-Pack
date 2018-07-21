/********************************************************
*   Copyright (C) 2018 All rights reserved.
*
*   Filename: mpc_core_mkz.h
*   Author  : junlong.gao
*   Date    : May 26, 2018
*   Describe: header file of MPC controller for MKZ
*
********************************************************/

#ifndef MPC_CORE_MKZ_H__
#define MPC_CORE_MKZ_H__
#define POINTFOLLOW
// ROS includes
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
// user defined includes
#include "autogo_msgs/ControlCommand.h"
#include "autogo_msgs/PathPoint.h"
#include "autogo_msgs/Trajectory.h"
#include "autogo_msgs/TrajectoryPoint.h"
#include "autogo_msgs/localization_status.h"
//mkz related info
#include "mpc_mkz.h"
#include "mpc_viz.h"
#include "mkz_lowpass.h"
#include "mkz_pid.h"
#include <sensor_msgs/Imu.h>
#include <dbw_mkz_msgs/ThrottleCmd.h>
#include <dbw_mkz_msgs/ThrottleReport.h>
#include <dbw_mkz_msgs/BrakeCmd.h>
#include <dbw_mkz_msgs/BrakeReport.h>
#include <dbw_mkz_msgs/SteeringCmd.h>
#include <dbw_mkz_msgs/SteeringReport.h>
#include <dbw_mkz_msgs/FuelLevelReport.h>
#include <dbw_mkz_msgs/TwistCmd.h>
#include <geometry_msgs/TwistStamped.h>
// dynamic reconfigure includes
#include <dynamic_reconfigure/server.h>
//#include <dbw_mkz_twist_controller/ControllerConfig.h>
#include "trajectory_follower/TrajectoryFollowerReconfigureConfig.h"
#include "trajectory_follower/trajectory_follower_config.h"
// boost classes
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Char.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#ifdef POINTFOLLOW
#include "styx_msgs/Lane.h"
#endif

namespace trajectory_follower {
constexpr int32_t TRAJECTORY_RECEIVED = 10;
constexpr int32_t TRAJECTORY_NO_RECEIVED = 11;
constexpr int32_t TRAJECTORY_EMERGENCE_STOP = 12;
constexpr int32_t TRAJECTORY_INIT = 13;
class MpcNode {
 public:
  MpcNode(const ros::NodeHandle& nh, tf::TransformListener& tf);
  //~MpcNode();
  void run();

  void start();

  void RunMpcControl();

 protected:

  const double GAS_DENSITY = 2.858; // kg/gal

  void reconfigureCB(TrajectoryFollowerReconfigureConfig& config, uint32_t level);

  // void reconfigCtrller(dbw_mkz_twist_controller::ControllerConfig& config, uint32_t level) {
// 	cfg = config;                                         //pre-definition,including control parameters and ego car parameters
// 	cfg.vehicle_mass -= cfg.fuel_capacity * GAS_DENSITY; // Subtract weight of full gas tank
// 	cfg.vehicle_mass += 150.0;                            // Account for some passengers
// //	speed_pid_.setGains(cfg_.speed_kp, 0.0, 0.0);          //速度环
// //	accel_pid_.setGains(cfg_.accel_kp, cfg_.accel_ki, 0.0);//加速度环
// //	yaw_control_.setLateralAccelMax(cfg_.max_lat_accel);   //横向yaw_rate限幅
// 	lpf_accel_.setParams(cfg.accel_tau, 0.02);            //低通滤波器初始化
  // }

 private:
  /*-----------handle---------*/

  ros::NodeHandle nh_;

  ros::NodeHandle private_nh_,private_nh_1;

  tf::TransformListener& tf_;

  /*-----------class----------*/
  MPCVisualizationPtr viz_;

  boost::shared_ptr<dynamic_reconfigure::Server<TrajectoryFollowerReconfigureConfig>> dynamic_recfg_0;  // Dynamic reconfigure server to allow config modifications at runtime
  boost::shared_ptr<dynamic_reconfigure::Server<dbw_mkz_twist_controller::ControllerConfig>> dynamic_recfg_1;
  MPCPtr mpc_ptr_;

  TrajectoryFollowerConfig cfg_;  // Config class that stores and manages all related parameters

  // dbw_mkz_twist_controller::ControllerConfig cfg;

  /*-----------variables----------*/
  int loop_rate_;

  double max_odom_time_interval_ = 2.0;

  double max_trajectory_time_interval_ = 1.0;

  double desired_velocity_;

  double wheel_base_ = 2.8498;

  double steer_ratio = 14.8;

  double delta_integration_ = 0.0;

  double keep_heading_;

  bool is_pose_set_,
	   is_velocity_set_,
	   is_config_set_,
	   is_imu_set_,
	   is_odom_set_,
	   is_tracklets_set_,
	   is_trajectory_solved_;

  bool is_localization_status_ = false;

  bool is_sim_status_ = false;

  bool is_wire_enable_ = false;

  int32_t is_trajectory_set_ = TRAJECTORY_INIT;

  std::string global_frame_;

  std::string topic_sub_imu_,
              topic_sub_chassis_,
			  topic_sub_localization_status_,
			  topic_sub_planning_,
			  topic_sub_steering_,
			  topic_sub_throttle_,
			  topic_sub_brake_,
			  topic_sub_fuel_level_,
			  topic_sub_wire_enable_;

  std::string topic_pub_throttle_,
			  topic_pub_brake_,
			  topic_pub_steering_,
			  topic_pub_mid_accel_,
			  topic_pub_mid_rqt_accel_,
			  topic_pub_twist_,
			  topic_pub_debug_;


  MkzLowPass lpf_accel_;

  MkzLowPass lpf_fuel_;

  PidControl accel_pid_;

  geometry_msgs::PoseStamped carcenter_,front_wheel_;

  //TODO:
//  dbw_mkz_msgs::ThrottleReport throttle_info_;
//
//  dbw_mkz_msgs::BrakeInfoReport brake_info_;

  geometry_msgs::Twist actual_;



  std_msgs::Float64 accel_msg;

  float steering_wheel_angle = 0.0;

  std::vector<autogo_msgs::TrajectoryPoint> current_trajectory_;

  std::vector<autogo_msgs::TrajectoryPoint> predicted_trajectory_;

  std::vector<autogo_msgs::TrajectoryPoint> fitted_trajectory_;

  std::vector<autogo_msgs::TrajectoryPoint> fitted_waypoints_;

  std::pair<tf::Vector3, tf::Quaternion> current_pose_;

  std::pair<bool, Eigen::Vector3d> feedback_vel;

  /*--------------ROS related info IO msgs----------------*/
  ros::Publisher pub1_, pub2_, pub3_, pub4_, pub5_;

  ros::Subscriber sub1_, sub2_, sub3_, sub4_, sub5_, sub6_, sub7_, sub8_, sub9_;

  ros::Time latest_odometry_time_,
            latest_tf_time_,
  			latest_trajectory_time_,
  			last_tracklets_time_,
            latest_no_trajectory_time_;

  std_msgs::ColorRGBA color1_;

  std_msgs::ColorRGBA color2_;

  std_msgs::ColorRGBA color3_;

  visualization_msgs::MarkerArray fitted_trajectory_marker;

  visualization_msgs::MarkerArray predicted_trajectory_marker;

  visualization_msgs::MarkerArray fitted_waypoints_marker;

  /*--------------Private call back functions----------------*/

  void paramConfig();

  bool confirmPreparationStatus();

  void registerMpc();

  void updateRobotPose();

  void confirmOdometry();

  void confirmTF();

  void confirmMotionPlan();

  void setTrajectoryForControl(const bool trajectory_set, const vector<autogo_msgs::TrajectoryPoint>& traj,
                               vector<autogo_msgs::TrajectoryPoint>* traj_for_mpc, double desired_velocity_for_mpc);

  void GetPlanningOffline(const styx_msgs::LaneConstPtr &msg);

  void GetPlanningOnline(const autogo_msgs::TrajectoryConstPtr& msg);

  void GetSteeringReport(const dbw_mkz_msgs::SteeringReport::ConstPtr& msg);

  /*-----------callbacks-----real Status------*/
  void GetLocalizationStatus(const autogo_msgs::localization_status& msg);

  void GetChassis(const nav_msgs::OdometryConstPtr& msg);

  void GetChassis();

  void GetImu(const sensor_msgs::ImuConstPtr& msg);

  void GetEnable(const std_msgs::Bool::ConstPtr& msg);

  void GetFuel(const dbw_mkz_msgs::FuelLevelReport::ConstPtr& msg);

  /*------------callbacks-----simulation-----*/

  void GetChassisSim(const geometry_msgs::TwistStampedConstPtr &msg);

  void GetImuSim(const geometry_msgs::TwistStampedConstPtr &msg);

  void GetLocalizationStatusSim(const geometry_msgs::PoseStampedConstPtr &msg);

  void GetEnableSim(const geometry_msgs::PoseStampedConstPtr &msg);

  void GetFuelSim(const geometry_msgs::PoseStampedConstPtr &msg);

  double GetLatDis(const vector<autogo_msgs::TrajectoryPoint>& traj, int num,
  		                  const double& length,const geometry_msgs::PoseStamped& front_wheel);

  /*----------initializer----------*/
  void initForROS();

  void registerDynamicReconfigure();
  /*----------msg publish----------*/

  void publishControlCommand(const double& mpc_v, const double& mpc_delta) const;

  void publishControlCommand(const double& mpc_v, const double& current_v, double mpc_a, const double& mpc_delta);

  void pubDebugInfo(const vector<autogo_msgs::TrajectoryPoint>& traj, const geometry_msgs::PoseStamped& ego_pos);

  double computeLookaheadDistance() const;

  double computeCommandVelocity() const;
  template <class T>
  int Sign (T a) {if (a>0) return 1; else if (a<0) return -1; else return 0;}

};

}  // namespace trajectory_follower

#endif  // MPC_CORE_MKZ_H__
