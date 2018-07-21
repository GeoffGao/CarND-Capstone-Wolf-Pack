/**
 * created by Ye Bo
 * date: 2017-11-30
 * modified from https://github.com/CPFL/Autoware
 */

//ROS includes
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
//trajectory msgs
#include <trajectory_follower/Pose.h>
#include <trajectory_follower/Trajectory.h>

#include "trajectory_follower/libwaypoint_follower.h"

namespace trajectory_follower {
	class PurePursuit{
	public:
		PurePursuit();
		~PurePursuit();
		
		//for setting data
		void setLookaheadDistance(const double &ld)
		{
			lookahead_distance_ = ld;
		}
		void setCurrentVelocity(const double &cur_linear_vel, const double &cur_angular_vel)
		{
			current_linear_velocity_ = cur_linear_vel;
			current_angular_velocity_ = cur_angular_vel;
		}
		
		void setCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg)
		{
			current_pose_ = msg->pose;
		}
		
		void setCurrentTrajectory(const std::vector<trajectory_follower::Pose> &traj)
		{
			current_trajectory_ = traj;
		}
		
		void setLinearInterpolationParameter(const bool &param)
		{
			is_linear_interpolation_ = param;
		}
		
		geometry_msgs::Pose getCurrentPose() const
		{
			return current_pose_;
		}
		
		double getLookaheadDistance() const
		{
			return lookahead_distance_;
		}
		
		// for debug and display on ROS
		geometry_msgs::Point getPoseOfNextWaypoint() const
		{
			return current_trajectory_.at(next_waypoint_number_).pose.position;
		}
		geometry_msgs::Point getPoseOfNextTarget() const
		{
			return next_target_position_;
		}
		// processing
		bool canGetCurvature(double *output_kappa);
		
		
	private:
		//constant
		const double RADIUS_MAX_;
		const double KAPPA_MIN_;
		
		//variables
		bool is_linear_interpolation_;
		double current_linear_velocity_, current_angular_velocity_;
		std::vector < trajectory_follower::Pose > current_trajectory_;
		double lookahead_distance_;
		int next_waypoint_number_;
		
		geometry_msgs::Pose current_pose_;
		geometry_msgs::Point next_target_position_;
		
		//function
		double calcCurvature(geometry_msgs::Point target) const;
		bool interpolateNextTarget(int next_waypoint, geometry_msgs::Point *next_target) const;
		void getNextWayPoint();
	
	};
	
}
