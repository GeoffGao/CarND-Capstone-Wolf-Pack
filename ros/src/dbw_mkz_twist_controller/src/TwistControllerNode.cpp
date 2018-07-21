/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2018, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
/*************************************
 * Note taken by Junlong Gao
 ************************************/

#include "TwistControllerNode.h"

namespace dbw_mkz_twist_controller {

TwistControllerNode::TwistControllerNode(ros::NodeHandle &n, ros::NodeHandle &pn) : srv_(pn)
{
  lpf_fuel_.setParams(60.0, 0.1);
  accel_pid_.setRange(0.0, 1.0);

  // Dynamic reconfigure
  srv_.setCallback(boost::bind(&TwistControllerNode::reconfig, this, _1, _2));

  // Control rate parameter
  double control_rate;
  pn.param("control_rate", control_rate, 50.0);
  control_period_ = 1.0 / control_rate;

  // Ackermann steering parameters
  acker_wheelbase_ = 2.8498; // 112.2 inches
  acker_track_ = 1.5824;     // 62.3 inches
  steering_ratio_ = 14.8;
  //add by junlong for MPC
  pn.getParam("control_method", control_method_);
  ROS_INFO("control_method_ = %d", control_method_);
  pn.getParam("ackermann_wheelbase", acker_wheelbase_);
  pn.getParam("ackermann_track", acker_track_);
  pn.getParam("steering_ratio", steering_ratio_);
  yaw_control_.setWheelBase(acker_wheelbase_);
  yaw_control_.setSteeringRatio(steering_ratio_);

  // Subscribers
  // TODO：三种cmd_vel，具体如何使用见recv调用函数，主要包含纵向速度command，以及横向command（角速度/转弯半径）
  /*
  sub_twist_  = n.subscribe("cmd_vel", 1, &TwistControllerNode::recvTwist, this);
  sub_twist2_ = n.subscribe("cmd_vel_with_limits", 1, &TwistControllerNode::recvTwist2, this);
  sub_twist3_ = n.subscribe("cmd_vel_stamped", 1, &TwistControllerNode::recvTwist3, this);
  */
  sub_current_velocity = n.subscribe("/current_velocity", 1, &TwistControllerNode::current_velocity_cb, this);
  sub_twist_cmd = n.subscribe("/twist_cmd", 1, &TwistControllerNode::recvTwist3, this);

//  // TODO: steering_report(获取当前的线速度, x方向速度&滤波后线加速度）
//  sub_steering_ = n.subscribe("steering_report", 1, &TwistControllerNode::recvSteeringReport, this);
//xxxx
  sub_steering_ = n.subscribe("/vehicle/steering_report", 1, &TwistControllerNode::recvSteeringReport, this);

  // TODO: IMU传感器信息订阅（获得angular_rate,从imu）
//  sub_imu_      = n.subscribe("imu/data_raw", 1, &TwistControllerNode::recvImu, this);
  // TODO: 线控状态查询
  //xxxx
    //sub_enable_   = n.subscribe("dbw_enabled", 1, &TwistControllerNode::recvEnable, this);
  sub_enable_   = n.subscribe("/vehicle/dbw_enabled", 1, &TwistControllerNode::recvEnable, this);

    // TODO: 车辆油量检测
  sub_fuel_level_ = n.subscribe("fuel_level_report", 1, &TwistControllerNode::recvFuel, this);

  // Publishers
  // TODO: 控制量输出
  /*
   pub_throttle_ = n.advertise<dbw_mkz_msgs::ThrottleCmd>("throttle_cmd", 1);
    pub_brake_    = n.advertise<dbw_mkz_msgs::BrakeCmd>("brake_cmd", 1);
    pub_steering_ = n.advertise<dbw_mkz_msgs::SteeringCmd>("steering_cmd", 1);
  */
  //xxxx
   pub_throttle_ = n.advertise<dbw_mkz_msgs::ThrottleCmd>("/vehicle/throttle_cmd", 1);
   pub_brake_    = n.advertise<dbw_mkz_msgs::BrakeCmd>("/vehicle/brake_cmd", 1);
   pub_steering_ = n.advertise<dbw_mkz_msgs::SteeringCmd>("/vehicle/steering_cmd", 1);

  // Debug：中间调试变量
  pub_accel_     = n.advertise<std_msgs::Float64>("filtered_accel", 1);
  pub_req_accel_ = n.advertise<std_msgs::Float64>("req_accel", 1);

  // Timers
  control_timer_ = n.createTimer(ros::Duration(control_period_), &TwistControllerNode::controlCallback, this);
}

void TwistControllerNode::controlCallback(const ros::TimerEvent& event)
{
  /*=========================control初始化环节=====================*/
  if ((event.current_real - cmd_stamp_).toSec() > (10.0 * control_period_)) {
    speed_pid_.resetIntegrator();
    accel_pid_.resetIntegrator();
    steer_pid_.resetIntegrator();
    return;
  }
  //动态调整车辆质量
  double vehicle_mass = cfg_.vehicle_mass + lpf_fuel_.get() / 100.0 * cfg_.fuel_capacity * GAS_DENSITY;
  //velocity tracking error 计算
  double vel_error   = cmd_vel_.twist.linear.x  - actual_.linear.x;
  double yawrt_error = cmd_vel_.twist.angular.z - actual_.angular.z;

  //低速（包括怠速状态）speed_pid重置积分部分
  if ((fabs(cmd_vel_.twist.linear.x) < mphToMps(1.0)) || !cfg_.pub_pedals) {
    speed_pid_.resetIntegrator();
  }
  //过转角零度的时候 steer_pid重置积分部分
//   if ((fabs(cmd_vel_.twist.angular.z) < 0.0001) || !cfg_.pub_steering) {
//     steer_pid_.resetIntegrator();
//   }
  //设置速度环的加减速度最大值（1g）
  speed_pid_.setRange(
      -std::min(fabs(cmd_vel_.decel_limit) > 0.0 ? fabs(cmd_vel_.decel_limit) : 9.8,
                cfg_.decel_max > 0.0 ? cfg_.decel_max : 9.8),
       std::min(fabs(cmd_vel_.accel_limit) > 0.0 ? fabs(cmd_vel_.accel_limit) : 9.8,
                cfg_.accel_max > 0.0 ? cfg_.accel_max : 9.8)
  );
  //TODO: to be determined
  steer_pid_.setRange(-0.44 , 0.44); //-0.44,0.44
  double accel_cmd = 0.0, ori =0.0;
  if(control_method_ == 0 ){
  //手动算所需加速度指令
     accel_cmd = speed_pid_.step(vel_error, control_period_);
  }else if (control_method_ == 1){
	 accel_cmd =  cmd_vel_.twist.linear.x;
  }
  //低速状态下纵向加速度过滤，如果速度指令是0，则加速度指令是min(-0.1m/s2,accecl_cmd)
  if (cmd_vel_.twist.linear.x <= (double)1e-2) {
    accel_cmd = std::min(accel_cmd, -530 / vehicle_mass / cfg_.wheel_radius);
  } 
  //accel调试信息发布
  std_msgs::Float64 accel_cmd_msg;
  accel_cmd_msg.data = accel_cmd;
  pub_req_accel_.publish(accel_cmd_msg);
  /*============================线控启动情况下的增量式PID控制器行为================================*/
  if (sys_enable_) {
    dbw_mkz_msgs::ThrottleCmd throttle_cmd;
    dbw_mkz_msgs::BrakeCmd brake_cmd;
    dbw_mkz_msgs::SteeringCmd steering_cmd;
    //油门操作
    throttle_cmd.enable = true;
    throttle_cmd.pedal_cmd_type = dbw_mkz_msgs::ThrottleCmd::CMD_PERCENT;
    if (accel_cmd >= 0) {
      throttle_cmd.pedal_cmd = accel_pid_.step(accel_cmd - lpf_accel_.get(), control_period_);
    } else {
      accel_pid_.resetIntegrator();
      throttle_cmd.pedal_cmd = 0;
    }
    //刹车操作（按照刹车力矩给出的刹车指令，需要减速度的时候给出）
    brake_cmd.enable = true;
    brake_cmd.pedal_cmd_type = dbw_mkz_msgs::BrakeCmd::CMD_TORQUE;
    if (accel_cmd < -cfg_.brake_deadband) {
      brake_cmd.pedal_cmd = -accel_cmd * vehicle_mass * cfg_.wheel_radius;
    } else {
      brake_cmd.pedal_cmd = 0;
    }

    //转向操作，使用P控制改为PID
    steering_cmd.enable = true;
    double tmp = 0;
		double angular_vel_cmd = steer_pid_.step(cmd_vel_.twist.angular.z, control_period_);
		if (fabsf(cmd_vel_.twist.linear.x) > 0.1){// When moving, use measured speed to compute steering angle to improve accuracy
//			 tmp = steering_ratio_ * angular_vel_cmd;//while real
			 tmp = angular_vel_cmd;  //while simulator
		}else{ tmp = 0;}

		if (tmp > 0.44) { tmp = 0.44;}
		else if (tmp < -0.44) { tmp = -0.44;}
		ori = yaw_control_.getSteeringWheelAngle(cmd_vel_.twist.linear.x, //纵向速度指令
				                                 angular_vel_cmd,         //横向角度？cmd_vel_.twist.angular.z
												 actual_.linear.x);       //实际纵向速度


//    }else if (control_method_ == 0){
//    	tmp = cmd_vel_.twist.angular.z;
////    	lpf_steer_.filt(tmp);
//    }

    steering_cmd.steering_wheel_angle_cmd = ori;
    //控制指令输出
    if (cfg_.pub_pedals) {
      pub_throttle_.publish(throttle_cmd);
      pub_brake_.publish(brake_cmd);
    }
    if (cfg_.pub_steering) {
      pub_steering_.publish(steering_cmd);
    }
    if(control_method_ == 0){
        std::cout<<"recieved cmd.v="<<cmd_vel_.twist.linear.x<<", yawrate_cmd="<<cmd_vel_.twist.angular.z<<", steering_ori="<<ori<<
        		   ", pub_steering="<<steering_cmd.steering_wheel_angle_cmd<<", pub_throttle="<<throttle_cmd.pedal_cmd<<std::endl;
    }else if (control_method_ == 1){
    	std::cout<<"recieved cmd.v="<<cmd_vel_.twist.linear.x<<", yawrate_cmd="<<cmd_vel_.twist.angular.z<<
    	        		   ", pub_steering="<<steering_cmd.steering_wheel_angle_cmd<<", pub_throttle="<<throttle_cmd.pedal_cmd<<std::endl;
    }
  } else {
    speed_pid_.resetIntegrator();
    accel_pid_.resetIntegrator();
    steer_pid_.resetIntegrator();
  }
}

void TwistControllerNode::reconfig(ControllerConfig& config, uint32_t level)
{
  cfg_ = config;                                         //pre-definition,including control parameters and ego car parameters
  cfg_.vehicle_mass -= cfg_.fuel_capacity * GAS_DENSITY; // Subtract weight of full gas tank
  cfg_.vehicle_mass += 150.0;     // Account for some passengers
  //---------------add by junlong for MPC control--------------------
  speed_pid_.setGains(cfg_.speed_kp, 0.0, 0.0);          //速度环
  accel_pid_.setGains(cfg_.accel_kp, cfg_.accel_ki, 0.0);//加速度环
  ROS_WARN("PID controller: speed.kp = %f, accel.kp = %f, accel.ki = %f",cfg_.speed_kp, cfg_.accel_kp, cfg_.accel_ki);
//  steer_pid_.setGains(cfg_.steer_kp, 0.01, 0.005);//方向盘环
  steer_pid_.setGains(cfg_.steer_kp,cfg_.steer_ki, cfg_.steer_kd);//方向盘环
  ROS_WARN("steer.kp = %f, steer.ki = %f, steer.kd = %f",cfg_.steer_kp,cfg_.steer_ki,cfg_.steer_kd);
  //--------------------end interp--------------------------------
  yaw_control_.setLateralAccelMax(cfg_.max_lat_accel);   //横向yaw_rate限幅
  lpf_accel_.setParams(cfg_.accel_tau, 0.02);            //低通滤波器初始化
  lpf_steer_.setParams(cfg_.accel_tau,0.02);
}

void TwistControllerNode::recvTwist(const geometry_msgs::Twist::ConstPtr& msg)
{
  cmd_vel_.twist = *msg;
  cmd_vel_.accel_limit = 0;
  cmd_vel_.decel_limit = 0;
  cmd_stamp_ = ros::Time::now();
}

void TwistControllerNode::recvTwist2(const dbw_mkz_msgs::TwistCmd::ConstPtr& msg)
{
  cmd_vel_ = *msg;
  cmd_stamp_ = ros::Time::now();
}

void TwistControllerNode::recvTwist3(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  cmd_vel_.twist = msg->twist;
  ROS_INFO("twist controller recieved command twist.linear.x=%f, twist.angular.z=%f",cmd_vel_.twist.linear.x,cmd_vel_.twist.angular.z);
  cmd_vel_.accel_limit = 0;
  cmd_vel_.decel_limit = 0;
  cmd_stamp_ = ros::Time::now();
}

void TwistControllerNode::recvFuel(const dbw_mkz_msgs::FuelLevelReport::ConstPtr& msg)
{
  lpf_fuel_.filt(msg->fuel_level);
}

void TwistControllerNode::recvSteeringReport(const dbw_mkz_msgs::SteeringReport::ConstPtr& msg)
{
  double raw_accel = 50.0 * (msg->speed - actual_.linear.x);
  lpf_accel_.filt(raw_accel);

  std_msgs::Float64 accel_msg;
  accel_msg.data = lpf_accel_.get();//纵向加速度滤波
  pub_accel_.publish(accel_msg);    //滤波后纵向加速度调试信息输出

  actual_.linear.x = msg->speed;    //x方向速度输出
}

void TwistControllerNode::recvImu(const sensor_msgs::Imu::ConstPtr& msg)
{
  actual_.angular.z = msg->angular_velocity.z;
//  lpf_steer_.filt(msg->twist.angular.z);
}

void TwistControllerNode::recvEnable(const std_msgs::Bool::ConstPtr& msg)
{
  sys_enable_ = msg->data;
}

void TwistControllerNode::current_velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{

	actual_.angular.z = msg->twist.angular.z;

}


}

