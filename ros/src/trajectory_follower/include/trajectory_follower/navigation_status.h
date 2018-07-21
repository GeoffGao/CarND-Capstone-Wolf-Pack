// Copyright [2017] <Zhuoyang Du>

#ifndef AUTOGO_NAVIGATE_NAVIGATION_STATUS_H_
#define AUTOGO_NAVIGATE_NAVIGATION_STATUS_H_

#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>

#include "ros/ros.h"


#include "autogo_msgs/TrackletArray.h"
#include "autogo_msgs/Tracklet.h"
#include "has_local_planner/Pose.h"
#include "has_local_planner/Trajectory.h"

#include "autogo_msgs/DecisionResult.h"
#include "vector_map_server/Waypoint.h"
#include "vector_map_server/WaypointArray.h"
#include "vector_map_server/GetRoute.h"

#include "costmap_2d/costmap_2d.h"
#include "costmap_2d/costmap_2d_ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include "autogo_msgs/navigation_status.h"
#include "autogo_msgs/localization_status.h"
#include "sensor_msgs/Imu.h"
#include "vector_map_server/WaypointArray.h"
namespace autogo_navigate{
constexpr int32_t TRAJECTORY_RECEIVED = 10;
constexpr int32_t TRAJECTORY_NO_RECEIVED = 11;
constexpr int32_t TRAJECTORY_EMERGENCE_STOP = 12;
constexpr int32_t TRAJECTORY_INIT = 13;
class NavagateStatus{
 public:
    NavagateStatus(const ros::NodeHandle& nh, tf::TransformListener& tf);

    void Start();

    void RunOnce();

 private:

    // Read some params from config file.
    void ParamConfig();

	void UpdateVehicleState();
	void confirmPreparationStatus();
	void confirmOdometry();
	void confirmMotionPlan();
    // Initialize planner.
    void RegisterPlanner();

    // Initialize costmap.
    void RegisterCostmap();

    // Initialize publishers and subscribers.
    void InitPublishersAndSubsribers();


    // Comfirm decision message.
    void ComfirmDecisionMessage();
	
	std::vector<has_local_planner::Pose> TransformTrajectoy(const has_local_planner::Trajectory& trajectory);

    void callbackFromLocalizationStatus(const autogo_msgs::localization_status& msg);
	void callbackFromImu(const sensor_msgs::ImuConstPtr& msg);
	void callbackFromOdometry(const nav_msgs::OdometryConstPtr& msg);
	void callbackFromCurrentPose(const nav_msgs::OdometryConstPtr& msg);
	void callbackFromCurrentVelocity(const nav_msgs::OdometryConstPtr& msg);
	void callbackFromTrajectory(const has_local_planner::TrajectoryConstPtr& msg);
	void callbackFromDecisionResult(const autogo_msgs::DecisionResultConstPtr& msg);
	void callbackFromTracklets(const autogo_msgs::TrackletArrayConstPtr& msg);
	void CallbackWaypoints(const vector_map_server::WaypointArray& waypoint_array);
 private:
    // Set planning loop rate to 200ms.
    double rate_ = 5;

    ros::NodeHandle nh_;

    ros::Publisher pub1_, pub2_, pub3_, pub4_, pub5_, pub6_;
	ros::Publisher pub11_, pub12_, pub13_, pub14_, pub15_;
	//subscriber
	ros::Subscriber sub1_, sub2_, sub3_, sub4_, sub5_, sub6_, sub7_, sub8_;


	
    // Sim time flag.
    bool use_sim_time_;

    // Costmap
    tf::TransformListener& tf_;
    costmap_2d::Costmap2DROS* planner_costmap_ros_, *controller_costmap_ros_;
    bool turn_on_costmap_;

    // Vehicle state from perception
    bool vehicle_state_ready_ = false;
    double vehicle_vel_ = 0.0;
    bool velocity_ready_ = false;
	double max_acc_ = 2.0;
	ros::Time latest_odometry_time_, latest_tf_time_, latest_trajectory_time_, last_tracklets_time_, latest_no_trajectory_time_; 
	bool is_pose_set_, is_velocity_set_, is_config_set_, is_trajectory_solved_;
	bool is_tracklets_set_ = false;
	bool is_imu_set_ = false;
	bool is_odom_set_ =false;
	bool is_localization_status_ = false;
	bool is_decision_result_ = false;
    // Dynamic obstacles from perception.
	int32_t is_trajectory_set_ = TRAJECTORY_INIT;
	// Trajectory
	//std::vector<has_local_planner::Pose> trajectory_;
    // Behavior planning result.
    autogo_msgs::DecisionResult decision_result_;
    bool enable_decision_ = true;
    bool decision_ready_ = false;
    ros::Time callback_decision_time_;
    double max_callback_decision_time_ = 2.0;

    // Global route.
    bool waypoints_ready_ = true;
    bool route_from_file_ = false;

};
} // namespace autogo_navigate

#endif  // AUTOGO_NAVIGATE_NAVIGATION_STATUS_H_

