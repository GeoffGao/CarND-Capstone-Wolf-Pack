/**
 * created by Ye Bo
 * date: 2017-12-01
 * the pure pursuit basic idea:
 * - Determine the anent location of the vehicle.
 * - Find the path point closest to the vehicle.
 * - Find the goal point
 * - Transform the goal pohl to vehicle coordinates.
 * - Calculate the curvature and request the vehicle to set the steering to that curvature.
 * - Update the vehicle’s position. 
 */

#include "pure_pursuit_core.h"
namespace trajectory_follower {
PurePursuitNode::PurePursuitNode()
	: private_nh_("~")
	, pp_()
	, LOOP_RATE_(10)
	, is_velocity_set_(false)
	, is_trajectory_set_(false)
	, lookahead_distance_ratio_(2.0)
	, minimum_lookahead_distance_(6.0)
	, is_pose_set_(false)
{
	initForROS();
	
	//线性插值,default:true	
	pp_.setLinearInterpolationParameter(is_linear_interpolation_);
}

PurePursuitNode::~PurePursuitNode()
{

}





void PurePursuitNode::initForROS()
{
	//ros parameter settings
	private_nh_.param("max_brake_acceleration", max_brake_acceleration_, double(3.0));
	private_nh_.param("brake_distance", brake_distance_, double(0.2));
	private_nh_.param("min_turn_radius", min_turn_radius_, double(3.0));
	private_nh_.param("vehicle_info/wheel_base", wheel_base_, double(2.5));
	private_nh_.param("is_linear_interpolation", is_linear_interpolation_, bool(true));
	//发布转角命令时设为true
	private_nh_.param("publishes_for_steering_robot", publishes_for_steering_robot_, bool(true));
	sub1_ = nh_.subscribe("final_trajectory", 10, &PurePursuitNode::callbackFromTrajectory, this);
	
	sub2_ = nh_.subscribe("current_pose", 10, &PurePursuitNode::callbackFromCurrentPose, this);
	
	sub3_ = nh_.subscribe("current_velocity",10, &PurePursuitNode::callbackFromCurrentVelocity, this);
	
	//setup publisher
	pub1_ = nh_.advertise<geometry_msgs::Twist>("twist_raw", 10);
	pub2_ = nh_.advertise<trajectory_follower::ControlCommandStamped>("ctrl_cmd", 10);
	
	//for display
	pub11_ = nh_.advertise<visualization_msgs::Marker>("next_waypoint_mark", 0);
	pub12_ = nh_.advertise<visualization_msgs::Marker>("next_target_mark", 0);
	pub13_ = nh_.advertise<visualization_msgs::Marker>("search_circle_mark", 0);
	//pub14_ = nh_.advertise<visualization_msgs::Marker>("line_point_mark", 0);  // debug tool
	pub15_ = nh_.advertise<visualization_msgs::Marker>("trajectory_circle_mark", 0);
	
	//odom_helper_.setOdomTopic("odometry/filted2");
	
}

void PurePursuitNode::run()
{
	ROS_INFO_STREAM(" pure_pursuit_core start.");
	ros::Rate loop_rate(LOOP_RATE_);
	while (ros::ok())
	{
		ros::spinOnce();
		if (!is_velocity_set_ || !is_trajectory_set_)
		{
			ROS_WARN("necessary topics are not subscribed yet ...");
			loop_rate.sleep();
			continue;
		}
		// 求预瞄点距离并传给pp_
		pp_.setLookaheadDistance(computeCommandVelocity());
		
		double kappa = 0;
		bool can_get_curvature = pp_.canGetCurvature(&kappa);
		publishTwistStamped(can_get_curvature, kappa);
		publishControlCommandStamped(can_get_curvature, kappa);
		
		// for visualization with Rviz
		pub11_.publish(displayNextWaypoint(pp_.getPoseOfNextWaypoint()));
		pub13_.publish(displaySearchRadius(pp_.getCurrentPose().position, pp_.getLookaheadDistance()));
		pub12_.publish(displayNextTarget(pp_.getPoseOfNextTarget()));	pub15_.publish(displayTrajectoryCircle(trajectory_follower::generateTrajectoryCircle(pp_.getPoseOfNextTarget(), pp_.getCurrentPose())));
		
		is_pose_set_ = false;
		is_velocity_set_ = false;
		is_trajectory_set_ = false;
		loop_rate.sleep();
		
	}
}

//compute 预瞄点距离
double PurePursuitNode::computeLookaheadDistance() const
{
	//TODO use the velocity acc angular and soon to determine the la!!!
	//  the lookahead distance must be considered within the context of one of two problems:
	// -Regaining a path; i.e. the vehicle is a “large” distance from the path and must attain the path.
	// -Maintaining the path, i.e. the vehicle is on the path and wants to remain on the path
	double maximum_lookahead_distance = current_linear_velocity_ * 10; //最大距离设为当前速度10倍
	
	//ld = A*v^2 + B*v + C
	//A = 1/(2*a_max) 车辆制动距离; B: 遇到异常状况是进行反应的行驶距离, default: 0.2; C: 最小转弯半径
	//TODO need to be dynamic reconfigure
	double ld = 1 / (2 *  max_brake_acceleration_) * current_angular_velocity_ * current_angular_velocity_ + brake_distance_ * current_angular_velocity_ + min_turn_radius_;
	
	//double ld = current_linear_velocity_ * lookahead_distance_ratio_;//default:2.0, 通过lookahead_distance_ratio_调成预瞄距离的值
	
	// 设定上下限制 minimum_lookahead_distance_ < ld < maximum_lookahead_distance
	return ld< minimum_lookahead_distance_ ? minimum_lookahead_distance_ : ld > maximum_lookahead_distance ? maximum_lookahead_distance : ld; 
}


void PurePursuitNode::publishTwistStamped(const bool& can_get_curvature, const double& kappa) const
{
	//以线速度角速度形式下发
	geometry_msgs::TwistStamped ts;
	ts.header.stamp = ros::Time::now();
	ts.twist.linear.x = can_get_curvature ? computeCommandVelocity() : 0;
	ts.twist.angular.z = can_get_curvature ? kappa * ts.twist.linear.x : 0;
	pub1_.publish(ts);
}

void PurePursuitNode::publishControlCommandStamped(const bool& can_get_curvature, const double& kappa) const
{
	if (!publishes_for_steering_robot_)
    return;

	trajectory_follower::ControlCommandStamped ccs;
	ccs.header.stamp = ros::Time::now();
	ccs.cmd.linear_velocity = can_get_curvature ? computeCommandVelocity() : 0;
	ccs.cmd.steering_angle = can_get_curvature ? convertCurvatureToSteeringAngle(wheel_base_, kappa) : 0;

	pub2_.publish(ccs);
}



double PurePursuitNode::computeCommandVelocity() const
{
	return command_linear_velocity_;
}


void PurePursuitNode::callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	pp_.setCurrentPose(msg);
	is_pose_set_ = true;
}


void PurePursuitNode::callbackFromCurrentVelocity(const nav_msgs::OdometryConstPtr& msg)
{
	current_linear_velocity_ = msg->twist.twist.linear.x;
	current_angular_velocity_ = msg->twist.twist.angular.z;
	
	pp_.setCurrentVelocity(current_linear_velocity_, current_angular_velocity_);
	
	is_velocity_set_ = true;
}



void PurePursuitNode::callbackFromTrajectory(const TrajectoryConstPtr& msg)
{
	if (!msg->poses.empty())
	{
		command_linear_velocity_ = msg->poses.at(0).twist.linear.x;
	}
	else
		command_linear_velocity_ = 0;
	
	pp_.setCurrentTrajectory(msg->poses);
	is_trajectory_set_ = true;
	
	//pp_.se
}


double convertCurvatureToSteeringAngle(const double& wheel_base, const double& kappa)
{
	return atan(wheel_base * kappa);
}


}