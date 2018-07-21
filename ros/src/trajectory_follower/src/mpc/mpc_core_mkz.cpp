/***
 * created by Ye Bo, Modified by Junlong Gao
 * date: 2017-12-04
 */

#include "mpc_core_mkz.h"
#include <cmath>
#define DEBUG
//#define REAL
const string STATUS = "PointFollow";//use offline points
const string METHOD = "NMPC";
//const string METHOD = "MPC";
//const string STATUS = "Planning";// use path planning

namespace trajectory_follower {
MpcNode::MpcNode(const ros::NodeHandle& nh, tf::TransformListener& tf)
    : nh_(nh),
      tf_(tf),
      private_nh_("~"),
      loop_rate_(10),
      is_pose_set_(false),
      is_imu_set_(false),
      is_odom_set_(false),
      keep_heading_(0.0) {
  paramConfig();
  registerDynamicReconfigure();
  lpf_fuel_.setParams(60.0, 0.1);
  lpf_accel_.setParams(cfg_.configMPC.accel_tau, 0.02);            //低通滤波器初始化
  accel_pid_.setGains(cfg_.configMPC.accel_kp, cfg_.configMPC.accel_ki, 0.0);//加速度环
  registerMpc();
  initForROS();
  viz_ = MPCVisualizationPtr(new MPCVisualization(nh_));
}
void MpcNode::paramConfig() {
  /*---------------ROS message subscribe-----------*/
  ros::param::get("~mpc/control_rate", loop_rate_);
  ros::param::get("~mpc/max_odom_time_interval", max_odom_time_interval_);
  ros::param::get("~mpc/max_trajectory_time_interval", max_trajectory_time_interval_);
  ros::param::get("~mpc/topic_sub_imu", topic_sub_imu_);
  ros::param::get("~mpc/topic_sub_chassis", topic_sub_chassis_);
  ros::param::get("~mpc/topic_sub_localization_status", topic_sub_localization_status_);
  ros::param::get("~mpc/topic_sub_planning", topic_sub_planning_);
  ros::param::get("~mpc/topic_sub_steering", topic_sub_steering_);
  ros::param::get("~mpc/topic_sub_throttle", topic_sub_throttle_);
  ros::param::get("~mpc/topic_sub_brake", topic_sub_brake_);
  ros::param::get("~mpc/topic_sub_fuel_level", topic_sub_fuel_level_);
  ros::param::get("~mpc/topic_sub_wire_enable", topic_sub_wire_enable_);
  /*---------------ROS message publish-----------*/
  ros::param::get("~mpc/topic_pub_throttle", topic_pub_throttle_);
  ros::param::get("~mpc/topic_pub_brake", topic_pub_brake_);
  ros::param::get("~mpc/topic_pub_steering", topic_pub_steering_);
  ros::param::get("~mpc/topic_pub_mid_accel", topic_pub_mid_accel_);
  ros::param::get("~mpc/topic_pub_mid_rqt_accel", topic_pub_mid_rqt_accel_);
  ros::param::get("~mpc/topic_pub_twist",topic_pub_twist_);
  ros::param::get("~mpc/topic_pub_debuginfo",topic_pub_debug_);
  ros::param::get("~mpc/is_sim_status", is_sim_status_);
  /*---------------line color visual-------------*/
  ros::param::get("~mpc/color_1_a", color1_.a);
  ros::param::get("~mpc/color_1_r", color1_.r);
  ros::param::get("~mpc/color_1_g", color1_.g);
  ros::param::get("~mpc/color_1_b", color1_.b);
  ros::param::get("~mpc/color_2_a", color2_.a);
  ros::param::get("~mpc/color_2_r", color2_.r);
  ros::param::get("~mpc/color_2_g", color2_.g);
  ros::param::get("~mpc/color_2_b", color2_.b);
  ros::param::get("~mpc/color_3_a", color3_.a);
  ros::param::get("~mpc/color_3_r", color3_.r);
  ros::param::get("~mpc/color_3_g", color3_.g);
  ros::param::get("~mpc/color_3_b", color3_.b);

  color1_.a /= 255.0;
  color1_.r /= 255.0;
  color1_.g /= 255.0;
  color1_.b /= 255.0;

  color2_.a /= 255.0;
  color2_.r /= 255.0;
  color2_.g /= 255.0;
  color2_.b /= 255.0;

  color3_.a /= 255.0;
  color3_.r /= 255.0;
  color3_.g /= 255.0;
  color3_.b /= 255.0;
}
void MpcNode::registerMpc() {
  mpc_ptr_ = MPCPtr(new MPC(cfg_));
}

void MpcNode::reconfigureCB(TrajectoryFollowerReconfigureConfig& config, uint32_t level) {
    cfg_.reconfigure(config);
    cfg_.configMPC.vehicle_mass -= cfg_.configMPC.fuel_capacity * GAS_DENSITY; // Subtract weight of full gas tank
    cfg_.configMPC.vehicle_mass += 150.0;                            // Account for some passengers
//  speed_pid_.setGains(cfg_.speed_kp, 0.0, 0.0);          //速度环
//  yaw_control_.setLateralAccelMax(cfg_.max_lat_accel);   //横向yaw_rate限幅


}

void MpcNode::run() {
  ROS_INFO("MPC MKZ node start");
  ros::Rate loop_rate(loop_rate_);
  while (ros::ok()) {
    ros::spinOnce();
    GetChassis();//combine steering and IMU info
#ifdef REAL
    updateRobotPose();
#endif
    if (is_trajectory_set_ == TRAJECTORY_INIT) {
      loop_rate.sleep();
      continue;
    }
    bool status = confirmPreparationStatus();
    double velocity = 0.0;
    double steering = 0.0;
    double acceleration = 0.0;
    double ego_vel = feedback_vel.second.coeffRef(0);
	if (!status || is_trajectory_set_ == TRAJECTORY_NO_RECEIVED || current_trajectory_.size() == 0) {
	  ROS_WARN("mpc_core: trajectory emergency stop, must stop immediately!");
	  velocity = 0.0;
	  steering = keep_heading_;
	  acceleration = 0.0;//TODO: fix emergency acceleration
	  publishControlCommand(velocity, ego_vel, acceleration, steering);
	  mpc_ptr_->Reset();
	  loop_rate.sleep();
	  continue;
	}

#ifdef DEBUG
    std::cout<<"==============MPCRUN CP1,curr_pos.x="<<current_pose_.first.getX()<<" .y="<<current_pose_.first.getY()<<" .z="<<current_pose_.first.getZ()<<std::endl;
    std::cout<<"==============MPCRUN CP2,feedback_vel.speed="<<feedback_vel.second.coeffRef(0)<<", yaw_rate="<<feedback_vel.second.coeffRef(1)<<", acceleration="<<feedback_vel.second.coeffRef(2)<<std::endl;
    std::cout<<"==============MPCRUN CP3, input traj.size="<<current_trajectory_.size()<<std::endl;
#endif
    std::vector<double> result;
    bool is_mpc_resultable = mpc_ptr_->Solve(current_trajectory_,//规划出的轨迹
    		                                 current_pose_,//ego_car_pos, x,y,z, orientation
											 feedback_vel, //底盘数据，包括velocity 与 yaw_rate
											 result);

    ROS_WARN("main after solve CP1");
    if (!is_mpc_resultable) {
      //ROS_INFO("mpc_core: No mpc solution is figured out!");
      mpc_ptr_->Reset();
      ROS_WARN("mpc_core: No solution, soft_brake!");
	  velocity = 0.0;
	  steering = keep_heading_;
	  acceleration = 0.0;//TODO: fix emergency acceleration
	  publishControlCommand(velocity, ego_vel, acceleration, steering);
	  mpc_ptr_->Reset();
	  loop_rate.sleep();
	  continue;

    } else {
    	ROS_WARN("main after solve CP2, steering_raw=%f, acceleration_raw=%f",result[0],result[1]);
    	if(STATUS=="Planning"){
    		velocity = 2.0;
			steering = result[0];
			acceleration = result[1];//TODO: to be modified
			keep_heading_ = result[1];
			ROS_ERROR("MPC done: steering = %f, accel = %f", steering, acceleration);
    	}
    	else if(STATUS=="PointFollow"){
    		if (status && current_trajectory_.size() < int(feedback_vel.second.coeffRef(0)))
			{
				ROS_WARN("mpc_core: meet traj end! Slow down mode Trigged!");
				//使用 v = a×t 来计算刹车时间
				velocity = 2.0;
				steering = result[0];
				acceleration = -1.0;//TODO: fix emergency acceleration
			} else{
				velocity = 2.0;
				steering = result[0];
				acceleration = result[1];//TODO: to be modified
				keep_heading_ = result[1];
				ROS_ERROR("MPC done: steering = %f, accel = %f", steering, acceleration);
			}
    	}
    }
    publishControlCommand(velocity,ego_vel, acceleration, steering);
    front_wheel_.pose.orientation.w = current_pose_.second.getW();
    front_wheel_.pose.orientation.x = current_pose_.second.getX();
    front_wheel_.pose.orientation.y = current_pose_.second.getY();
    front_wheel_.pose.orientation.z = current_pose_.second.getZ();
	front_wheel_.pose.position.x = current_pose_.first.getX() + wheel_base_ * cos(tf::getYaw(current_pose_.second));
	front_wheel_.pose.position.y = current_pose_.first.getY() + wheel_base_ * sin(tf::getYaw(current_pose_.second));
    carcenter_.pose.orientation = front_wheel_.pose.orientation;
    carcenter_.pose.position.x = current_pose_.first.getX() + 1.0 * cos(tf::getYaw(current_pose_.second));
    carcenter_.pose.position.y = current_pose_.first.getY() + 1.0 * sin(tf::getYaw(current_pose_.second));
    pubDebugInfo(current_trajectory_, carcenter_);
    // TODO: need to take care of the order of control input!

    // ros::Time then = ros::Time::now();
    // ROS_INFO("calculation time:%.4f ", (then.toNSec()-now.toNSec()) * 1e-6);
    // visualization
    predicted_trajectory_.clear();
    fitted_trajectory_.clear();
    fitted_waypoints_.clear();

//    visualization_msgs::MarkerArray fitted_trajectory_marker;
//    visualization_msgs::MarkerArray predicted_trajectory_marker;
//    visualization_msgs::MarkerArray fitted_waypoints_marker;
//
//	mpc_ptr_->trajectoryFromRobotToWorld(current_pose_, mpc_ptr_->fitted_trajectory(), &fitted_trajectory_);
//	mpc_ptr_->trajectoryFromRobotToWorld(current_pose_, mpc_ptr_->predicted_trajectory(), &predicted_trajectory_);
//	mpc_ptr_->trajectoryFromRobotToWorld(current_pose_, mpc_ptr_->fitted_waypoints(), &fitted_waypoints_);
//
//	viz_->VisualizeTrajectory(color1_, 0.1, 0.1, fitted_trajectory_, fitted_trajectory_marker);
//    viz_->VisualizeTrajectory(color2_, 0.2, 0.2, predicted_trajectory_, predicted_trajectory_marker);
//    viz_->VisualizeTrajectory(color3_, 0.2, 0.2, fitted_waypoints_, fitted_waypoints_marker);
//
//    viz_->pub1_.publish(fitted_trajectory_marker);
//    viz_->pub2_.publish(predicted_trajectory_marker);
//    viz_->pub3_.publish(fitted_waypoints_marker);

    loop_rate.sleep();
  }

}

void MpcNode::registerDynamicReconfigure() {
  dynamic_recfg_0 = boost::make_shared<dynamic_reconfigure::Server<TrajectoryFollowerReconfigureConfig>>(private_nh_);
  dynamic_reconfigure::Server<TrajectoryFollowerReconfigureConfig>::CallbackType cb1 =
      boost::bind(&MpcNode::reconfigureCB, this, _1, _2);
  dynamic_recfg_0->setCallback(cb1);
   //check:reconfig
  //Tried to advertise a service that is already advertised in this node [/mpc_node/set_parameters]
  // dynamic_recfg_1 = boost::make_shared<dynamic_reconfigure::Server<dbw_mkz_twist_controller::ControllerConfig>>(private_nh_);
  // dynamic_reconfigure::Server<dbw_mkz_twist_controller::ControllerConfig>::CallbackType cb2 =
  //     boost::bind(&MpcNode::reconfigCtrller, this, _1, _2);
  // dynamic_recfg_1->setCallback(cb2);
}

void MpcNode::initForROS() {
  
  sub5_ = nh_.subscribe(topic_sub_steering_, 1, &MpcNode::GetSteeringReport, this);
#ifdef REAL
  if (STATUS == "PointFollow"){
	  sub1_ = nh_.subscribe(topic_sub_planning_, 10, &MpcNode::GetPlanningOffline, this);
  }elseif (STATUS == "Planning"){
      sub1_ = nh_.subscribe(topic_sub_planning_, 10, &MpcNode::GetPlanningOnline, this);
  }
  sub2_ = nh_.subscribe(topic_sub_imu_, 1, &MpcNode::GetImu, this);
  sub3_ = nh_.subscribe(topic_sub_chassis_, 10, &MpcNode::GetChassis, this);
  sub4_ = nh_.subscribe(topic_sub_localization_status_, 10, &MpcNode::GetLocalizationStatus, this);
  sub8_ = nh_.subscribe(topic_sub_wire_enable_, 1, &MpcNode::GetEnable, this);
  sub9_ = nh_.subscribe(topic_sub_fuel_level_, 1, &MpcNode::GetFuel, this);
#else
  sub1_ = nh_.subscribe(topic_sub_planning_, 10, &MpcNode::GetPlanningOffline, this);
  sub2_ = nh_.subscribe(topic_sub_imu_, 1, &MpcNode::GetImuSim, this);
  sub3_ = nh_.subscribe(topic_sub_chassis_, 10, &MpcNode::GetChassisSim, this);//check
  sub4_ = nh_.subscribe(topic_sub_localization_status_, 10, &MpcNode::GetLocalizationStatusSim, this);
  sub8_ = nh_.subscribe(topic_sub_wire_enable_, 1, &MpcNode::GetEnableSim, this);//check
  sub9_ = nh_.subscribe(topic_sub_fuel_level_, 1, &MpcNode::GetFuelSim, this);
#endif
  /*---直接发布控制信息，因mpc的解算速率的限制，无法直接发出50Hz的控制指令，暂时不这么用-----*/
//  pub1_ = nh_.advertise<dbw_mkz_msgs::ThrottleCmd>( topic_pub_throttle_, 1);
//  pub2_ = nh_.advertise<dbw_mkz_msgs::BrakeCmd>( topic_pub_brake_, 1);
//  pub3_ = nh_.advertise<dbw_mkz_msgs::SteeringCmd>( topic_pub_steering_, 1);
  /*---------使用mpc发出twist command命令，封装底层pid产生 50Hz的控制指令-------------*/
  pub1_ = nh_.advertise<geometry_msgs::TwistStamped>(topic_pub_twist_,10);
  pub2_ = nh_.advertise<geometry_msgs::TwistStamped>(topic_pub_debug_,1);
  pub4_ = nh_.advertise<std_msgs::Float64>( topic_pub_mid_accel_, 1);
  pub5_ = nh_.advertise<std_msgs::Float64>( topic_pub_mid_rqt_accel_, 1);
}

void MpcNode::pubDebugInfo(const vector<autogo_msgs::TrajectoryPoint>& traj, const geometry_msgs::PoseStamped& ego_pos)
{
	geometry_msgs::TwistStamped debug_pub;
	double total_length = 0.0, fb_lateral_err = 0.0;
	int num = 0;
	total_length = sqrt(pow(traj[num].path_point.point.x-ego_pos.pose.position.x, 2)+pow(traj[num].path_point.point.y-ego_pos.pose.position.y, 2));
	fb_lateral_err = GetLatDis(traj, num, total_length, ego_pos);
	debug_pub.header.stamp = ros::Time::now();
	debug_pub.twist.linear.x  = fb_lateral_err;
	debug_pub.twist.angular.z = traj[num].path_point.theta - tf::getYaw(ego_pos.pose.orientation);
	pub2_.publish(debug_pub);
}

double MpcNode::GetLatDis(const vector<autogo_msgs::TrajectoryPoint>& traj, int num,
		                  const double& length,const geometry_msgs::PoseStamped& front_wheel)
{
	double v1x=0, v1y=0, v2x=0, v2y=0;//v1 is  vector of p1p2, v2 is vector of pegop2
	double comp1=0.0, comp2=0.0, comp3 = 0.0, comp4=0.0, angle = 0.0;
	int direction = 0;
	autogo_msgs::TrajectoryPoint p1, p2;
	p1 = traj[num], p2 = traj[num+1];
	double yaw = p2.path_point.theta;

	v1x = p2.path_point.point.x-p1.path_point.point.x;               v1y = p2.path_point.point.y-p1.path_point.point.y;
    v2x = p2.path_point.point.x-front_wheel.pose.position.x;         v2y = p2.path_point.point.y-front_wheel.pose.position.y;
    comp1 = (p2.path_point.point.x-front_wheel.pose.position.x)*(p2.path_point.point.x-p1.path_point.point.x);
    comp2 = (p2.path_point.point.y-front_wheel.pose.position.y)*(p2.path_point.point.y-p1.path_point.point.y);
    comp3 = sqrt(pow(v2x,2)+pow(v2y,2));
    comp4 = sqrt(pow(v1x,2)+pow(v1y,2));
    direction = Sign(v1x*v2y - v2x*v1y);
    angle = 1 * direction * acos((comp1 + comp2)/(comp3 * comp4));
    return length*sin(angle);
}



void MpcNode::publishControlCommand(const double& vel_cmd, const double& current_v, double accel_cmd, const double& steer_cmd)
{

	  //动态调整车辆质量
	  double vehicle_mass = cfg_.configMPC.vehicle_mass + lpf_fuel_.get() / 100.0 * cfg_.configMPC.fuel_capacity * GAS_DENSITY;
	   //低速状态下纵向加速度过滤，如果速度指令是0，则加速度指令是min(-0.1m/s2,accecl_cmd)
	   //即紧急状态下的零输出
	  if (vel_cmd <= (double)1e-2) {
		  if(current_v>vel_cmd){
			  accel_cmd = std::min(accel_cmd, -0.5 * 530 / vehicle_mass / cfg_.configMPC.wheel_radius);
		  }
		  else
	          accel_cmd = std::min(accel_cmd, -530 / vehicle_mass / cfg_.configMPC.wheel_radius);
	  }
	  //accel调试信息发布
	  std_msgs::Float64 accel_cmd_msg;
	  accel_cmd_msg.data = accel_cmd;
	  pub5_.publish(accel_cmd_msg);
	  /*============================线控启动情况下的增量式PID控制器行为================================*/
	  if (is_wire_enable_) {
		  /*----------当前MPC发送频率上不去，不直接使用本方法--------------------*/
		/*
	    dbw_mkz_msgs::ThrottleCmd throttle_cmd;
	    dbw_mkz_msgs::BrakeCmd brake_cmd;
	    dbw_mkz_msgs::SteeringCmd steering_cmd;
	    //油门操作
	    throttle_cmd.enable = true;
	    throttle_cmd.pedal_cmd_type = dbw_mkz_msgs::ThrottleCmd::CMD_PERCENT;
	    if (accel_cmd >= 0) {
	      throttle_cmd.pedal_cmd = accel_pid_.step(accel_cmd - lpf_accel_.get(), 30);
	      throttle_cmd.pedal_cmd = throttle_cmd.pedal_cmd > 1 ? 1 : throttle_cmd.pedal_cmd;//TODO
	    } else {
	      accel_pid_.resetIntegrator();
	      throttle_cmd.pedal_cmd = 0;
	    }
	    //刹车操作（按照刹车力矩给出的刹车指令，需要减速度的时候给出）
	    brake_cmd.enable = true;
	    brake_cmd.pedal_cmd_type = dbw_mkz_msgs::BrakeCmd::CMD_TORQUE;
	    if (accel_cmd < -cfg_.configMPC.brake_deadband) {
	      brake_cmd.pedal_cmd = -accel_cmd * vehicle_mass * cfg_.configMPC.wheel_radius;
	    } else {
	      brake_cmd.pedal_cmd = 0;
	    }
	    steering_cmd.enable = true;
	    */
		geometry_msgs::TwistStamped mpc_control_;
		mpc_control_.header.stamp = ros::Time::now();
	    int steer_fact = 1.0;

	    double steer_wheel_cmd = steer_cmd;
	    double steering_wheel_angle_max_ = 0.44;//TODO
	    if (steer_wheel_cmd > steering_wheel_angle_max_) {
	    	steer_wheel_cmd = steering_wheel_angle_max_;
	        } else if (steer_wheel_cmd < -steering_wheel_angle_max_) {
	        	steer_wheel_cmd = -steering_wheel_angle_max_;
	        }
	    mpc_control_.twist.linear.x = accel_cmd;
	    mpc_control_.twist.angular.z = steer_wheel_cmd;
	    pub1_.publish(mpc_control_);
	    /*
	    steering_cmd.steering_wheel_angle_cmd = steer_wheel_cmd;// + cfg_.steer_kp * (steer_cmd - actual_.angular.z);
        steering_cmd.steering_wheel_angle_velocity = 0.148;//TODO:no limit???
	    //控制指令输出
	    if (cfg_.configMPC.pub_pedals) {
	      pub1_.publish(throttle_cmd);
	      pub2_.publish(brake_cmd);

	    }
	    if (cfg_.configMPC.pub_steering) {
	      pub3_.publish(steering_cmd);
	    }
	    */
	  } else {
	    ROS_WARN("pub control command main logic pot");
	  }
}


void MpcNode::confirmOdometry() {
  if (is_odom_set_) {
    double delta_time = ros::Time::now().toSec() - latest_odometry_time_.toSec();
    if (delta_time > max_odom_time_interval_) {
      ROS_ERROR("mpc_core: can not receive the odometry for %.4fs!", delta_time);
      is_odom_set_ = false;
    }
  }
}

void MpcNode::confirmMotionPlan() {
   double delta_time = ros::Time::now().toSec() - latest_trajectory_time_.toSec();
   //ROS_INFO("latest_no_trajectory_time_:=%f",latest_no_trajectory_time_.toSec() );
   //ROS_INFO("latest_trajectory_time_:=%f", latest_trajectory_time_.toSec());
   //ROS_INFO("delta_time=%f",delta_time);
  if (delta_time > max_trajectory_time_interval_) {
    ROS_ERROR("mpc_core: can not receive the trajectory for %.4fs!", delta_time);
    is_trajectory_set_ = TRAJECTORY_NO_RECEIVED;
  }
}

bool MpcNode::confirmPreparationStatus() {
  //ROS_INFO("is_sim_status_=%d, is_odom_set_=%d, is_imu_set_=%d, is_trajectory_set_=%d",is_sim_status_,is_odom_set_,is_imu_set_,is_trajectory_set_);
  bool status = true;
  confirmOdometry();

  confirmMotionPlan();

  if (!is_sim_status_) {
    if (!is_localization_status_) {
      status &= false;
      ROS_WARN("mpc: Localization failed!");
    }
    if (!is_odom_set_) {
      status &= false;
      ROS_WARN("No odometry received or odometry received time out!");
    }
    if (!is_imu_set_) {
      status &= false;
      ROS_WARN("No imu data received!");
    }
  }
  if (is_trajectory_set_ != TRAJECTORY_RECEIVED ){ // || isnan(current_trajectory_[0].path_point.point.x)) {
    status &= false;
    ROS_WARN("No trajectory received or trajectory received time out!");
  }
  return status;
}

void MpcNode::GetPlanningOnline(const autogo_msgs::TrajectoryConstPtr& msg) {
  // 1. decision　result: EMERGENCE_STOP
  //		trajectory_existing= true, 存在pose, type = 0
  // 2. decision　result: EMERGENCE_STOP
  //		trajectory_existing= false, 不存在pose, type = 0
  // 3. decision result: MOVE_FORWARD
  //		trajectory_existing= false, 不存在pose, type = 0
  if (msg->type == 0) {
    current_trajectory_.clear();
    ROS_WARN("mpc_core: Emergency stop!");
    latest_trajectory_time_ = ros::Time::now();
    is_trajectory_set_ = TRAJECTORY_EMERGENCE_STOP;
  } else if (msg->trajectory_points.size() < 6) {
    ROS_WARN("mpc_core: No trajectory!");
    latest_no_trajectory_time_ = ros::Time::now();
    is_trajectory_set_ = TRAJECTORY_NO_RECEIVED;
  } else {
    current_trajectory_.clear();
    current_trajectory_.emplace_back(msg->trajectory_points[0]);
    current_trajectory_[0].path_point.d = 0.0;
    for (size_t i = 1; i < msg->trajectory_points.size(); i++) {
      current_trajectory_.emplace_back(msg->trajectory_points[i]);
      current_trajectory_[i].path_point.d =
          std::hypot(msg->trajectory_points[i].path_point.point.x - msg->trajectory_points[i - 1].path_point.point.x,
                     msg->trajectory_points[i].path_point.point.y - msg->trajectory_points[i - 1].path_point.point.y);
    }
    latest_trajectory_time_ = ros::Time::now();
    is_trajectory_set_ = TRAJECTORY_RECEIVED;
  }
}

void MpcNode::GetPlanningOffline(const styx_msgs::LaneConstPtr &msg) {
#ifdef DEBUG
  ROS_INFO("in GetPlanning waypoints.size=%d",msg->waypoints.size());
#endif
  if (msg->waypoints.size() < 6) {
    ROS_WARN("mpc_core: No trajectory!");
    latest_no_trajectory_time_ = ros::Time::now();
    is_trajectory_set_ = TRAJECTORY_NO_RECEIVED;
  } else {
#ifdef DEBUG1
	  ROS_INFO("first waypoint info=%f",msg->waypoints[0].pose.pose.position.x);
#endif
	  if(isnan(msg->waypoints[0].pose.pose.position.x))
	  {
		  ROS_WARN("mpc_core: No trajectory!");
		  latest_no_trajectory_time_ = ros::Time::now();
		  is_trajectory_set_ = TRAJECTORY_NO_RECEIVED;
	  }
	  else{
		  current_trajectory_.clear();
		  autogo_msgs::TrajectoryPoint temp;
		  for (size_t i = 0; i < msg->waypoints.size(); i++) {
			  temp.path_point.point.x = msg->waypoints[i].pose.pose.position.x;
			  temp.path_point.point.y = msg->waypoints[i].pose.pose.position.y;
			  current_trajectory_.emplace_back(temp);
				if(i == 0) current_trajectory_[i].path_point.d = 0.0;
				else{
					   /*build additional d information*/
					   current_trajectory_[i].path_point.d =
					   std::hypot(msg->waypoints[i].pose.pose.position.x - msg->waypoints[i - 1].pose.pose.position.x,
					   msg->waypoints[i].pose.pose.position.y - msg->waypoints[i - 1].pose.pose.position.y);
					   /*extract vel_cmd on Planning Point*/
					   current_trajectory_[i].v = msg->waypoints[i].twist.twist.linear.x;
				}
#ifdef DEBUG1
		        ROS_INFO("current_trajectory_[%d].x=%f, .y=%f, .d=%f, .v=%f",i,temp.path_point.point.x,temp.path_point.point.y,
		        		current_trajectory_[i].path_point.d, current_trajectory_[i].v);
#endif
		  }

		  latest_trajectory_time_ = ros::Time::now();
#ifdef DEBUG1
		      ROS_INFO("readTraj.size=%d, raw_msg.size=%d",current_trajectory_.size(), msg->waypoints.size());
		      ROS_INFO("latest_trajectory_time_=%f",latest_trajectory_time_.toSec());
#endif
		  is_trajectory_set_ = TRAJECTORY_RECEIVED;
	  }
  }
}

void MpcNode::updateRobotPose() {
  tf::StampedTransform transform;
  try {
    tf_.lookupTransform("/map", "/base_link", ros::Time(0), transform);
    current_pose_.first.setValue(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
    current_pose_.second.setValue(transform.getRotation().getX(), transform.getRotation().getY(),
                                  transform.getRotation().getZ(), transform.getRotation().getW());
    is_pose_set_ = true;
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
  }
}

void  MpcNode::GetSteeringReport(const dbw_mkz_msgs::SteeringReport::ConstPtr& msg)
{
	double raw_accel = 50.0 * (msg->speed - actual_.linear.x); //CAN frequency * (vd-v0)
	lpf_accel_.filt(raw_accel);
	accel_msg.data = lpf_accel_.get();//线加速度滤波
#ifdef DEBUG1
	ROS_INFO("in steering report msg.speed=%.4f, actual_linear.x = %.4f",msg->speed,actual_.linear.x);
	ROS_INFO("in steering report raw_accel=%.4f, accel=%.4f", raw_accel, accel_msg.data);
#endif
#ifdef REAL
	steering_wheel_angle = msg->steering_wheel_angle;//方向盘角度
#else
	steering_wheel_angle = msg->steering_wheel_angle_cmd * 14.8;//方向盘角度
	//ROS_INFO("steering_wheel_angle=%f(rad)",steering_wheel_angle);
#endif
	pub4_.publish(accel_msg);    //滤波后纵向加速度调试信息输出
	actual_.linear.x = msg->speed;    //x方向速度输出
}
/* ---------------------------on Car running------------------------------*/
void MpcNode::GetLocalizationStatus(const autogo_msgs::localization_status& msg) {
  is_localization_status_ = msg.status;
}

void MpcNode::GetChassis(const nav_msgs::OdometryConstPtr& msg) {
  feedback_vel.first = true;
  feedback_vel.second.coeffRef(0) = msg->twist.twist.linear.x;
  feedback_vel.second.coeffRef(1) = msg->twist.twist.angular.z;
  is_odom_set_ = true;
  latest_odometry_time_ = ros::Time::now();
}

void MpcNode::GetChassis() {
  feedback_vel.first = true;
  feedback_vel.second.coeffRef(0) = actual_.linear.x;//velocity
  feedback_vel.second.coeffRef(1) = steering_wheel_angle / 14.8;//steer_angle 弧度
  feedback_vel.second.coeffRef(2) = accel_msg.data;//acceleration
  is_odom_set_ = true;
  latest_odometry_time_ = ros::Time::now();
}

void MpcNode::GetImu(const sensor_msgs::ImuConstPtr& msg) {
  //sensor_msgs::Imu imu_data;
  //imu_data.angular_velocity = msg->angular_velocity;
  //imu_data.orientation = msg->orientation;
  /*this is from MKZ controller,yaw_rate*/
  actual_.angular.z = msg->angular_velocity.z;
  is_imu_set_ = true;

}

void  MpcNode::GetEnable(const std_msgs::Bool::ConstPtr& msg)
{
	 is_wire_enable_ = msg->data;
}

void  MpcNode::GetFuel(const dbw_mkz_msgs::FuelLevelReport::ConstPtr& msg)
{
	 lpf_fuel_.filt(msg->fuel_level);
}

/* ----------------------while in simulation system---------------------------*/
void MpcNode::GetChassisSim(const geometry_msgs::TwistStampedConstPtr &msg) {
  feedback_vel.first = true;
  feedback_vel.second.coeffRef(0) = msg->twist.linear.x;
  feedback_vel.second.coeffRef(1) = steering_wheel_angle / 14.8;//msg->twist.linear.x * tan(steering_wheel_angle / 14.8) / 2.8498;
  actual_.angular.z = msg->twist.linear.x * tan(steering_wheel_angle) / 2.8498;
  is_odom_set_ = true;
  latest_odometry_time_ = ros::Time::now();
}

void MpcNode::GetImuSim(const geometry_msgs::TwistStampedConstPtr &msg) {
  //sensor_msgs::Imu imu_data;
  //imu_data.angular_velocity = msg->angular_velocity;
  //imu_data.orientation = msg->orientation;
  /*this is from MKZ controller,yaw_rate*/
  //actual_.angular.z = msg->angular_velocity.z;
  is_imu_set_ = true;
}

void MpcNode::GetLocalizationStatusSim(const geometry_msgs::PoseStampedConstPtr &msg) {
	is_localization_status_ = true;
	current_pose_.first.setValue(msg->pose.position.x,msg->pose.position.y, 0);
	current_pose_.second.setValue(msg->pose.orientation.x,msg->pose.orientation.y,
			                      msg->pose.orientation.z, msg->pose.orientation.w);
#ifdef DEBUG1
	ROS_INFO("in ego localization pos origin:  pos.x=%f, .y=%f, .z=%f",msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
	ROS_INFO("in ego localization pos after: pos.x=%f, .y=%f, .z=%f",current_pose_.first.getX(),current_pose_.first.getY(),current_pose_.first.getZ());
	ROS_INFO("in ego localization orientation origin:  ori.x=%f, .y=%f, .z=%f, .w=%f",msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
	ROS_INFO("in ego localization orientation after:   pos.x=%f, .y=%f, .z=%f, .w=%f",current_pose_.second.getX(),current_pose_.second.getY(),current_pose_.second.getZ(),current_pose_.second.getW());
#endif
	is_pose_set_ = true;
}

void  MpcNode::GetEnableSim(const geometry_msgs::PoseStampedConstPtr &msg)
{
	 is_wire_enable_ = true;
}

void  MpcNode::GetFuelSim(const geometry_msgs::PoseStampedConstPtr &msg)
{
	 lpf_fuel_.filt(100);
}
/*-----------end of simulation status callback-----------------*/

}  // namespace trajectory_follower

int main(int argc, char** argv) {
  ros::init(argc, argv, "mpc_node");
  ros::NodeHandle nh;
  tf::TransformListener tf_;
  trajectory_follower::MpcNode mpc(nh, tf_);
  mpc.run();
  return 0;
}
