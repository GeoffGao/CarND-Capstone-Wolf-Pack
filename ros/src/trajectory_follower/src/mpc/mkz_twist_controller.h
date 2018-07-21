/***
 * created by Ye Bo
 * date: 2018-1-29
 */

#ifndef MKZ_TWIST_CONTROLLER_H
#define MKZ_TWIST_CONTROLLER_H

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Float64.h"

#include "dynamic_reconfigure/server.h"
#include "trajectory_follower/MkzControllerConfig.h"

#include "dbw_mkz_msgs/ThrottleCmd.h"
#include "dbw_mkz_msgs/ThrottleReport.h"
#include "dbw_mkz_msgs/BrakeCmd.h"
#include "dbw_mkz_msgs/BrakeReport.h"
#include "dbw_mkz_msgs/SteeringCmd.h"
#include "dbw_mkz_msgs/SteeringReport.h"
#include "dbw_mkz_msgs/FuelLevelReport.h"
#include "dbw_mkz_msgs/TwistCmd.h"

#include "mkz_lowpass.h"
#include "mkz_pid_control.h"
#include "mkz_yaw_control.h"


namespace trajectory_follower{

class MkzTwistController{

public:
	MkzTwistController(ros::NodeHandle &n, ros::NodeHandle &pn);

private:
	
	void reconfig(ControllerConfig& config, uint32_t level);
	
	void controlCallback(const ros::TimerEvent& event);
	
	void recvTwist(const geometry_msgs::Twist::ConstPtr& msg);
	
	void recvTwist2(const dbw_mkz_msgs::TwistCmd::ConstPtr& msg);
	
	void recvTwist3(const geometry_msgs::TwistStamped::ConstPtr& msg);
	
	void recvSteeringReport(const dbw_mkz_msgs::SteeringReport::ConstPtr& msg);
	
	void recvImu(const sensor_msgs::Imu::ConstPtr& msg);
	
	void recvEnable(const std_msgs::Bool::ConstPtr& msg);
	
	void recvFuel(const dbw_mkz_msgs::FuelLevelReport::ConstPtr& msg);

private:
	
	ros::Publisher pub_throttle_;
	ros::Publisher pub_brake_;
	ros::Publisher pub_steering_;
	ros::Publisher pub_accel_;
	ros::Publisher pub_req_accel_;
	ros::Subscriber sub_steering_;
	ros::Subscriber sub_imu_;
	ros::Subscriber sub_enable_;
	ros::Subscriber sub_twist_;
	ros::Subscriber sub_twist2_;
	ros::Subscriber sub_twist3_;
	ros::Subscriber sub_fuel_level_;
	ros::Timer control_timer_;

	dbw_mkz_msgs::TwistCmd cmd_vel_;
	geometry_msgs::Twist actual_;
	ros::Time cmd_stamp_;
	dynamic_reconfigure::Server<ControllerConfig> srv_;

	MkzPidControl speed_pid_;
	MkzPidControl accel_pid_;
	MkzYawControl yaw_control_;
	MkzLowPass lpf_accel_;
	MkzLowPass lpf_fuel_;
	MkzControllerConfig cfg_;
	bool sys_enable_;

	// Parameters
	double control_period_;
	double acker_wheelbase_;
	double acker_track_;
	double steering_ratio_;

	static const double GAS_DENSITY = 2.858; // kg/gal
	static double mphToMps(double mph) { return mph * 0.44704; }
	
};

}//namespace trajectory_follower

#endif