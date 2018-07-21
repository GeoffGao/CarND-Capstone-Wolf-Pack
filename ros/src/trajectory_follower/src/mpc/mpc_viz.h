/**
 * created by Ye Bo
 * date: 2017-12-11
 */

#ifndef MPC_VIZ_H__
#define MPC_VIZ_H__

// ROS includes
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include "autogo_msgs/PathPoint.h"
#include "autogo_msgs/Trajectory.h"
#include "autogo_msgs/TrajectoryPoint.h"
using namespace std;
namespace trajectory_follower {
class MPCVisualization {
 public:
  MPCVisualization(ros::NodeHandle nh) : nh_(nh) {
    Init();
  };
  virtual ~MPCVisualization() = default;

  void Init() {
    LoadVisualConfig();
    InitializeROS();
  }

  void VisualizeTrajectory(const std_msgs::ColorRGBA color, const double x, const double y,
                           const std::vector<autogo_msgs::TrajectoryPoint> &trajectory,
                           visualization_msgs::MarkerArray &waypoints_marker_array) {
    PathMarker(color, x, trajectory, waypoints_marker_array);
    PointMarker(color, x, y, trajectory, waypoints_marker_array);
  }

  ros::Publisher pub1_, pub2_, pub3_;

 private:
  void LoadVisualConfig() {
    ros::param::get("~mpc/predicted_trajectory", topic_pub_predicted_trajectory_);
    ros::param::get("~mpc/fitted_trajectory", topic_pub_fitted_trajectory_);
    ros::param::get("~mpc/fitted_waypoints", topic_pub_fitted_waypoints_);
  }

  void InitializeROS() {
    pub1_ = nh_.advertise<visualization_msgs::MarkerArray>(topic_pub_fitted_trajectory_, PUBLISH_RATE_);
    pub2_ = nh_.advertise<visualization_msgs::MarkerArray>(topic_pub_predicted_trajectory_, PUBLISH_RATE_);
    pub3_ = nh_.advertise<visualization_msgs::MarkerArray>(topic_pub_fitted_waypoints_, PUBLISH_RATE_);
  }

  void PointMarker(const std_msgs::ColorRGBA color, const double x, const double y,
                   const std::vector<autogo_msgs::TrajectoryPoint> &trajectory,
                   visualization_msgs::MarkerArray &waypoints_marker_array) {
    visualization_msgs::Marker waypoints_maker;
    waypoints_maker.header.frame_id = frame_id_;
    waypoints_maker.header.stamp = ros::Time();
    waypoints_maker.ns = "waypoints";
    waypoints_maker.id = 0;
    waypoints_maker.type = visualization_msgs::Marker::CUBE_LIST;
    waypoints_maker.action = visualization_msgs::Marker::ADD;
    waypoints_maker.scale.x = x;
    waypoints_maker.scale.y = y;
    waypoints_maker.color = color;
    waypoints_maker.color.a = 1.0;
    waypoints_maker.frame_locked = true;
    waypoints_maker.pose.orientation.w = 1.0;
//    waypoints_maker.lifetime = 0.05;
    for (size_t i = 0; i < trajectory.size(); i++) {
      waypoints_maker.points.emplace_back(trajectory[i].path_point.point);
    }
    waypoints_marker_array.markers.emplace_back(waypoints_maker);
  }

  void PathMarker(const std_msgs::ColorRGBA color, const double x,
                  const std::vector<autogo_msgs::TrajectoryPoint> &trajectory,
                  visualization_msgs::MarkerArray &waypoints_marker_array) {
    visualization_msgs::Marker waypoints_maker;
    waypoints_maker.header.frame_id = frame_id_;
    waypoints_maker.header.stamp = ros::Time();
    waypoints_maker.ns = "local_path_marker";
    waypoints_maker.id = 0;
    waypoints_maker.type = visualization_msgs::Marker::LINE_STRIP;
    waypoints_maker.action = visualization_msgs::Marker::ADD;
    waypoints_maker.scale.x = 0.5;
//    waypoints_maker.lifetime = 0.05;
    waypoints_maker.color = color;
    waypoints_maker.color.a = 0.5;
    waypoints_maker.frame_locked = true;
    waypoints_maker.pose.orientation.w = 1.0;

    for (size_t i = 0; i < trajectory.size(); i++) {
      waypoints_maker.points.emplace_back(trajectory[i].path_point.point);
    }
    waypoints_marker_array.markers.emplace_back(waypoints_maker);
  }

  std::string topic_pub_predicted_trajectory_, topic_pub_fitted_trajectory_, topic_pub_fitted_waypoints_;
  std::string frame_id_ = "world";
  ros::NodeHandle nh_;
  int PUBLISH_RATE_ = 10;
};

typedef boost::shared_ptr<MPCVisualization> MPCVisualizationPtr;

// 	MpcVisualization(){
// 			ros::param::get("~mpc/predicted_trajectory", topic_pub_predicted_trajectory_);
// 			ros::param::get("~mpc/fitted_trajectory", topic_pub_fitted_trajectory_);
// 			ros::param::get("~mpc/traj_for_control", topic_pub_traj_for_control_);
// 			pub_mpc_posesequence = nh_.advertise<geometry_msgs::PoseArray>("/mpc_node/posesequence",
// PUBLISH_RATE_); 			pub_mpc_next_target =
// nh_.advertise<visualization_msgs::Marker>("/mpc/next_target", PUBLISH_RATE_);
// pub_robot_plan_trajectory = nh_.advertise<visualization_msgs::Marker>("/mpc/robot_plan_trajectory", PUBLISH_RATE_);
// pub_robot_mpc_trajectory = nh_.advertise<visualization_msgs::Marker>("/mpc/robot_mpc_trajectory",PUBLISH_RATE_);
// pub_controller_trajectory = nh_.advertise<visualization_msgs::MarkerArray>(topic_pub_predicted_trajectory_,
// PUBLISH_RATE_); 			pub_motion_plan_trajectory =
// nh_.advertise<visualization_msgs::MarkerArray>("/mpc/motion_plan_trajectory", PUBLISH_RATE_);
// pub_navigation_status = nh_.advertise<autogo_msgs::navigation_status>("control_status", PUBLISH_RATE_);
// pub_fit_traj = nh_.advertise<visualization_msgs::MarkerArray>(topic_pub_fitted_trajectory_, PUBLISH_RATE_);
// pub_traj_for_control = nh_.advertise<visualization_msgs::MarkerArray>(topic_pub_traj_for_control_, PUBLISH_RATE_);
// 	};

// 	void paramConfig(){
// 		ros::param::get("~mpc/predicted_trajectory", topic_pub_predicted_trajectory_);
// 		ros::param::get("~mpc/fitted_trajectory", topic_pub_fitted_trajectory_);
// 	};

// 	void initForROS(){

// 	};

// 	void publishNextTarget(const geometry_msgs::Pose& pose){
// 		visualization_msgs::Marker marker;
// 		marker.header.frame_id = frame_id_;
// 		marker.header.stamp = ros::Time();
// 		marker.ns = "next_target";
// 		marker.id = 0;
// 		marker.type = visualization_msgs::Marker::SPHERE;
// 		marker.action = visualization_msgs::Marker::ADD;
// 		marker.pose = pose;
// 		std_msgs::ColorRGBA green;
// 		green.a = 1.0;
// 		green.b = 1.0;
// 		green.r = 0.0;
// 		green.g = 1.0;
// 		marker.color = green;
// 		marker.scale.x = 1.0;
// 		marker.scale.y = 1.0;
// 		marker.scale.z = 1.0;
// 		marker.frame_locked = true;

// 		pub_mpc_next_target.publish(marker);
// 	}

// 	void publishPoseSequence(const geometry_msgs::PoseArray& poseseq)
// 	{
// 		geometry_msgs::PoseArray pose_array;
// 		for(unsigned int i = 0; i < poseseq.poses.size(); ++i)
// 		{
// 			geometry_msgs::Pose pose;
// 			pose.position.x  = poseseq.poses[i].position.x;
// 			pose.position.y  = poseseq.poses[i].position.y;
// 			pose.position.z  = 0;
// 			pose.orientation = poseseq.poses[i].orientation;
// 			pose_array.poses.push_back(pose);
// 		}
// 		pose_array.header.frame_id = frame_id_;
// 		pose_array.header.stamp = ros::Time();
// 		pub_mpc_posesequence.publish(pose_array);

// 	}

// 	void publishRobotPlanTrajectory(const vector<double > waypoints_x, const vector<double > waypoints_y)
// 	{
// 		visualization_msgs::Marker vehicle_marker;
// 		vehicle_marker.type = visualization_msgs::Marker::LINE_STRIP;
// 		vehicle_marker.header.frame_id = "base_link";
// 		vehicle_marker.scale.x = 0.1;
// 		vehicle_marker.color.g = 1.0;
// 		vehicle_marker.color.a = 1.0;
// 		vehicle_marker.pose.orientation.w = 1.0;

// 		for (unsigned int i = 0; i < waypoints_x.size(); i++)
// 		{
// 			geometry_msgs::Point robot_state;
// 			robot_state.x = waypoints_x[i];
// 			robot_state.y = waypoints_y[i];
// 			vehicle_marker.points.push_back(robot_state);
// 		}

// 		pub_robot_plan_trajectory.publish(vehicle_marker);
// 	}

// 	void publishRobotMpcTrajectory(const vector<double> result)
// 	{
// 		visualization_msgs::Marker vehicle_marker;
// 		vehicle_marker.type = visualization_msgs::Marker::LINE_STRIP;
// 		vehicle_marker.header.frame_id = "base_link";
// 		vehicle_marker.scale.x = 0.1;
// 		vehicle_marker.color.g = 1.0;
// 		vehicle_marker.color.a = 1.0;
// 		vehicle_marker.color.r = 1.0;
// 		vehicle_marker.color.b = 1.0;
// 		vehicle_marker.pose.orientation.w = 1.0;
// 		int sequence_size = (result.size() - 2) /5;
// 		for (int i = 0; i < sequence_size; i++)
// 		{
// 			geometry_msgs::Point robot_state;
// 			robot_state.x = result[i*5 + 2];
// 			robot_state.y = result[i*5 + 3];
// 			vehicle_marker.points.push_back(robot_state);
// 		}
// 		pub_robot_mpc_trajectory.publish(vehicle_marker);
// 	}

// 	void viusalControllerTrajectory(const has_local_planner::Trajectory& traj){
// 		g_local_waypoints_marker_array.markers.clear();
// 		g_local_color.a = g_local_alpha;
// 		g_local_color.g =1.0;
// 		createLocalWaypointVelocityMarker(g_local_color, traj);
// 		createLocalPathMarker(g_local_color, traj);
// 		createLocalPointMarker(traj);
// 		publishLocalTrajectoryMarker();
// 	}

// 	void viusalFitTraj(const has_local_planner::Trajectory& traj){
// 		g_local_waypoints_marker_array_fit.markers.clear();
// 		g_local_color_fit.a = 1;
// 		g_local_color_fit.g = 1.0;
// 		g_local_color_fit.r = 1.0;
// 		// createLocalWaypointVelocityMarkerFit(g_local_color_fit, traj);
// 		createLocalPathMarkerFit(g_local_color_fit, traj);
// 		createLocalPointMarkerFit(traj);
// 		publishLocalTrajMarker();
// 	}

// 	void createLocalWaypointVelocityMarker(std_msgs::ColorRGBA color, const has_local_planner::Trajectory
// &controller_waypoints){
// 		// display by markers the velocity of each waypoint.
// 		visualization_msgs::Marker velocity;
// 		velocity.header.frame_id = frame_id_;
// 		velocity.header.stamp = ros::Time();
// 		velocity.ns = "local_waypoint_velocity";
// 		velocity.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
// 		velocity.action = visualization_msgs::Marker::ADD;
// 		velocity.scale.z = 0.4;
// 		velocity.color = color;
// 		velocity.frame_locked = true;
// 		velocity.pose.orientation.w = 1.0;
// 		// for (int i =0; i < static_cast<int> (controller_waypoints.poses.size()/5) +1; i++)
// 		for(int i =0; i < 1; i++)
// 		{
// 			velocity.id = i;
// 			geometry_msgs::Point relative_p;
// 			relative_p.y = -1.25;
// 			velocity.pose.position = calcAbsoluteCoordinate(relative_p,
// controller_waypoints.poses[5*i].pose); 			velocity.pose.position.z += 0.4;
// std::ostringstream oss;
// 			//oss << std::fixed << std::setprecision(4) << controller_waypoints.poses[i].twist.linear.x;
// 			oss << "("<<std::setprecision(3) <<controller_waypoints.poses[i].twist.linear.x << "," <<
// std::setprecision(3)<< controller_waypoints.poses[i].twist.angular.z <<")"; 			velocity.text =
// oss.str(); 			g_local_waypoints_marker_array.markers.push_back(velocity);
// 		}
// 	}
// 	void createLocalPathMarker(std_msgs::ColorRGBA color, const has_local_planner::Trajectory
// &controller_waypoints){ 		visualization_msgs::Marker lane_waypoint_marker;
// lane_waypoint_marker.header.frame_id = frame_id_; 		lane_waypoint_marker.header.stamp = ros::Time();
// lane_waypoint_marker.ns = "local_path_marker"; 		lane_waypoint_marker.id = 0;
// lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP; 		lane_waypoint_marker.action =
// visualization_msgs::Marker::ADD; 		lane_waypoint_marker.scale.x = 0.05;
// lane_waypoint_marker.scale.y = 0.05; 		lane_waypoint_marker.color = color;
// lane_waypoint_marker.color.a = 0.5; lane_waypoint_marker.frame_locked = true;
// lane_waypoint_marker.pose.orientation.w = 1.0;

// 		for (int i = 0; i < static_cast<int> (controller_waypoints.poses.size()); i++)
// 		{
// 			geometry_msgs::Point point;
// 			point = controller_waypoints.poses[i].pose.position;
// 			lane_waypoint_marker.points.push_back(point);
// 		}
// 		g_local_waypoints_marker_array.markers.push_back(lane_waypoint_marker);
// 	}
// 	void createLocalPointMarker(const has_local_planner::Trajectory &controller_waypoints){
// 		visualization_msgs::Marker lane_waypoint_marker;
// 		lane_waypoint_marker.header.frame_id = frame_id_;
// 		lane_waypoint_marker.header.stamp = ros::Time();
// 		lane_waypoint_marker.ns = "local_point_marker";
// 		lane_waypoint_marker.id = 0;
// 		lane_waypoint_marker.type = visualization_msgs::Marker::CUBE_LIST;
// 		lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
// 		lane_waypoint_marker.scale.x = 0.1;
// 		lane_waypoint_marker.scale.y = 0.1;
// 		lane_waypoint_marker.color.b = 1.0;
// 		lane_waypoint_marker.color.g = 1.0;
// 		lane_waypoint_marker.color.a = 1.0;
// 		lane_waypoint_marker.frame_locked = true;
// 		lane_waypoint_marker.pose.orientation.w = 1.0;
// 		for (int i = 0; i < static_cast<int> (controller_waypoints.poses.size()); i+= 5)
// 		{
// 			geometry_msgs::Point point;
// 			point = controller_waypoints.poses[i].pose.position;
// 			lane_waypoint_marker.points.push_back(point);
// 		}
// 		g_local_waypoints_marker_array.markers.push_back(lane_waypoint_marker);
// 	}
// 	void createLocalPathMarkerFit(std_msgs::ColorRGBA color, const has_local_planner::Trajectory
// &controller_waypoints){ 		visualization_msgs::Marker lane_waypoint_marker;
// lane_waypoint_marker.header.frame_id = frame_id_; 		lane_waypoint_marker.header.stamp = ros::Time();
// lane_waypoint_marker.ns = "local_path_marker"; 		lane_waypoint_marker.id = 0;
// lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP; 		lane_waypoint_marker.action =
// visualization_msgs::Marker::ADD; 		lane_waypoint_marker.scale.x = 0.05; lane_waypoint_marker.color = color;
// lane_waypoint_marker.color.a = 0.5; 		lane_waypoint_marker.frame_locked = true;
// 		lane_waypoint_marker.pose.orientation.w = 1.0;

// 		for (int i = 0; i < static_cast<int> (controller_waypoints.poses.size()); i++)
// 		{
// 			geometry_msgs::Point point;
// 			point = controller_waypoints.poses[i].pose.position;
// 			lane_waypoint_marker.points.push_back(point);
// 		}
// 		g_local_waypoints_marker_array_fit.markers.push_back(lane_waypoint_marker);
// 	}
// 	void createLocalPointMarkerFit(const has_local_planner::Trajectory &controller_waypoints){
// 		visualization_msgs::Marker lane_waypoint_marker;
// 		lane_waypoint_marker.header.frame_id = frame_id_;
// 		lane_waypoint_marker.header.stamp = ros::Time();
// 		lane_waypoint_marker.ns = "local_point_marker";
// 		lane_waypoint_marker.id = 0;
// 		lane_waypoint_marker.type = visualization_msgs::Marker::CUBE_LIST;
// 		lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
// 		lane_waypoint_marker.scale.x = 0.05;
// 		lane_waypoint_marker.scale.y = 0.05;
// 		lane_waypoint_marker.color.g = 1.0;
// 		lane_waypoint_marker.color.b = 1.0;
// 		lane_waypoint_marker.color.a = 1.0;
// 		lane_waypoint_marker.frame_locked = true;
// 		lane_waypoint_marker.pose.orientation.w = 1.0;
// 		for (int i = 0; i < static_cast<int> (controller_waypoints.poses.size()); i++)
// 		{
// 			geometry_msgs::Point point;
// 			point = controller_waypoints.poses[i].pose.position;
// 			lane_waypoint_marker.points.push_back(point);
// 		}
// 		g_local_waypoints_marker_array_fit.markers.push_back(lane_waypoint_marker);
// 	}
// 	void publishLocalTrajectoryMarker(){
// 		visualization_msgs::MarkerArray marker_array;
// 		marker_array.markers.insert(marker_array.markers.end(), g_local_waypoints_marker_array.markers.begin(),
// g_local_waypoints_marker_array.markers.end());
// 		// ROS_INFO_STREAM(topic_pub_predicted_trajectory_ << "test 666");
// 		pub_controller_trajectory.publish(marker_array);
// 	}

// 	void publishLocalTrajMarker(){
// 		visualization_msgs::MarkerArray marker_array;
// 		marker_array.markers.insert(marker_array.markers.end(),
// g_local_waypoints_marker_array_fit.markers.begin(), g_local_waypoints_marker_array_fit.markers.end());
// 		// ROS_INFO_STREAM(topic_pub_fitted_trajectory_ << "test 666");
// 		pub_fit_traj.publish(marker_array);
// 	}

// 	void viusalPlanningTrajectory(const vector<has_local_planner::Pose>& traj){
// 		g_global_marker_array.markers.clear();
// 		createGlobalWaypointVelocityMarker(traj);
// 		createGlobalPathMarker(traj);
// 		createGlobalPointMarker(traj);
// 		publishPlanningTrajectorylMarker();
// 	}

// 	void viusalpublishTrajForControl(const vector<has_local_planner::Pose>& traj){
// 		g_global_marker_array.markers.clear();
// 		// createGlobalWaypointVelocityMarker(traj);
// 		createGlobalPathMarker(traj);
// 		createGlobalPointMarker(traj);
// 		publishTrajForControlMarker();
// 	}
// 	void createGlobalWaypointVelocityMarker(const vector<has_local_planner::Pose> &plan_waypoints){
// 		visualization_msgs::Marker velocity;
// 		velocity.header.frame_id = frame_id_;
// 		velocity.header.stamp = ros::Time();
// 		velocity.ns = "planning_waypoint_velocity";
// 		velocity.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
// 		velocity.action = visualization_msgs::Marker::ADD;
// 		velocity.scale.z = 0.4;
// 		velocity.color.a = 1.0;
// 		velocity.color.r = 1;
// 		velocity.color.g = 1;
// 		velocity.color.b = 1;
// 		velocity.frame_locked = true;
// 		velocity.pose.orientation.w = 1.0;
// 		for (int i =0; i < static_cast<int> (plan_waypoints.size()/2); i++)
// 		{
// 			velocity.id = i;
// 			geometry_msgs::Point relative_p;
// 			relative_p.y = 0.65;
// 			velocity.pose.position = calcAbsoluteCoordinate(relative_p, plan_waypoints[2*i].pose);
// 			velocity.pose.position.z += 0.2;
// 			std::ostringstream oss;
// 			oss << std::fixed << std::setprecision(4) << plan_waypoints[i].twist.linear.x;
// 			velocity.text = oss.str();
// 			g_global_marker_array.markers.push_back(velocity);
// 		}
// 	}
// 	void createGlobalPathMarker(const vector<has_local_planner::Pose> &plan_waypoints){
// 		visualization_msgs::Marker lane_waypoint_marker;
// 		lane_waypoint_marker.header.frame_id = frame_id_;
// 		lane_waypoint_marker.header.stamp = ros::Time();
// 		lane_waypoint_marker.ns = "planning_path_marker";
// 		lane_waypoint_marker.id = 0;
// 		lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
// 		lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
// 		lane_waypoint_marker.scale.x = 0.05;
// 		lane_waypoint_marker.color.a = 0.5;
// 		lane_waypoint_marker.color.r = 1.0;
// 		lane_waypoint_marker.frame_locked = true;
// 		lane_waypoint_marker.pose.orientation.w = 1.0;

// 		for (int i = 0; i < static_cast<int> (plan_waypoints.size()); i++)
// 		{
// 			geometry_msgs::Point point;
// 			point = plan_waypoints[i].pose.position;
// 			lane_waypoint_marker.points.push_back(point);

// 		}
// 		g_global_marker_array.markers.push_back(lane_waypoint_marker);
// 	}
// 	void createGlobalPointMarker(const vector<has_local_planner::Pose> &plan_waypoints){
// 		visualization_msgs::Marker lane_waypoint_marker;
// 		lane_waypoint_marker.header.frame_id = frame_id_;
// 		lane_waypoint_marker.header.stamp = ros::Time();
// 		lane_waypoint_marker.ns = "planning_point_marker";
// 		lane_waypoint_marker.id = 0;
// 		lane_waypoint_marker.type = visualization_msgs::Marker::CUBE_LIST;
// 		lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
// 		lane_waypoint_marker.scale.x = 0.1;
// 		lane_waypoint_marker.scale.y = 0.1;
// 		lane_waypoint_marker.color.r = 1.0;
// 		lane_waypoint_marker.color.b = 0.5;
// 		lane_waypoint_marker.color.g = 0.2;
// 		lane_waypoint_marker.color.a = 1.0;
// 		lane_waypoint_marker.frame_locked = true;
// 		lane_waypoint_marker.pose.orientation.w = 1.0;
// 		for (int i = 0; i < static_cast<int> (plan_waypoints.size()); i++)
// 		{
// 			geometry_msgs::Point point;
// 			point = plan_waypoints[i].pose.position;
// 			lane_waypoint_marker.points.push_back(point);
// 		}
// 		g_global_marker_array.markers.push_back(lane_waypoint_marker);
// 	}

// 	void publishPlanningTrajectorylMarker(){
// 		visualization_msgs::MarkerArray marker_array;
// 		//insert global marker
// 		marker_array.markers.insert(marker_array.markers.end(), g_global_marker_array.markers.begin(),
// g_global_marker_array.markers.end()); 		pub_motion_plan_trajectory.publish(marker_array);
// 	}

// 	void publishTrajForControlMarker(){
// 		visualization_msgs::MarkerArray marker_array;
// 		//insert global marker
// 		marker_array.markers.insert(marker_array.markers.end(), g_global_marker_array.markers.begin(),
// g_global_marker_array.markers.end()); 		pub_traj_for_control.publish(marker_array);
// 	}

// 	void publishNavigationStatus(const autogo_msgs::navigation_status& status){
// 		pub_navigation_status.publish(status);
// 	}
// public:
// 	std::string topic_pub_predicted_trajectory_, topic_pub_fitted_trajectory_, topic_pub_traj_for_control_;
// 	ros::Publisher pub_mpc_posesequence, pub_mpc_next_target, pub_robot_plan_trajectory, pub_robot_mpc_trajectory,
// pub_controller_trajectory, pub_motion_plan_trajectory, pub_navigation_status, pub_fit_traj, pub_traj_for_control;

//   private:
// std::string frame_id_ = "map";
// ros::NodeHandle nh_;
// int PUBLISH_RATE_ = 10;
// 	visualization_msgs::MarkerArray g_local_waypoints_marker_array, g_global_marker_array;
// 	const double g_global_alpha = 0.2;
// 	const double g_local_alpha = 1.0;
// 	std_msgs::ColorRGBA g_local_color;
// 	std_msgs::ColorRGBA _global_color;

// 	std_msgs::ColorRGBA g_local_color_fit;
// 	std_msgs::ColorRGBA _global_color_fit;

// 	visualization_msgs::MarkerArray g_local_waypoints_marker_array_fit, g_global_marker_array_fit;
// 	};
}  // namespace trajectory_follower

#endif  // MPC_VIZ_H__
