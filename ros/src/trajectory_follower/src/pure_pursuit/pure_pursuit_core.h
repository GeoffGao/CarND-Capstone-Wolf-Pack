/**
 * created by Ye Bo
 * date: 2017-11-30
 */

#ifndef PURE_PURSUIT_CORE_H_
#define PURE_PURSUIT_CORE_H_

//ROS includes
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

//base local planner utilities
#include <base_local_planner/odometry_helper_ros.h>

//User defined includes
#include "pure_pursuit.h"
#include "pure_pursuit_viz.h"

#include "trajectory_follower/ControlCommandStamped.h"


namespace trajectory_follower{
	class PurePursuitNode{
	public:
		PurePursuitNode();
		~PurePursuitNode();
		
		void run();
		
	private:
		//handle
		ros::NodeHandle nh_;
		ros::NodeHandle private_nh_;
		
		//class
		PurePursuit pp_;
		
		//publisher
		ros::Publisher pub1_, pub2_, pub3_;
		ros::Publisher pub11_, pub12_, pub13_, pub14_, pub15_;
		
		//subscriber
		ros::Subscriber sub1_, sub2_, sub3_;
		
		//base_local_planner::OdometryHelperRos odom_helper_;
		
		//constants
		int LOOP_RATE_;
		
		//variables
		double wheel_base_;
		double current_linear_velocity_, current_angular_velocity_;
		double command_linear_velocity_;
		bool is_trajectory_set_, is_pose_set_, is_velocity_set_, is_config_set_;
		bool is_linear_interpolation_, publishes_for_steering_robot_;
		double lookahead_distance_ratio_;
		double max_brake_acceleration_, brake_distance_, min_turn_radius_;
		double minimum_lookahead_distance_;  // the next waypoint must be outside of this threshold.
		
		//callbacks
		void callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
		
		void callbackFromCurrentVelocity(const nav_msgs::OdometryConstPtr& msg);
		
		void callbackFromTrajectory(const trajectory_follower::TrajectoryConstPtr& msg);
		
		//initializer
		void initForROS();
		
		//publish
		void publishTwistStamped(const bool &can_get_curvature, const double &kappa) const;
		void publishControlCommandStamped(const bool &can_get_curvature, const double &kappa) const;
		double computeLookaheadDistance() const;
		double computeCommandVelocity() const;
	};
	
	double convertCurvatureToSteeringAngle(const double &wheel_base, const double &kappa);
	
	inline double kmph2mps(double velocity_kmph)
	{
		return (velocity_kmph * 1000) / (60 * 60);
	}
	
}


#endif //PURE_PURSUIT_CORE_H_