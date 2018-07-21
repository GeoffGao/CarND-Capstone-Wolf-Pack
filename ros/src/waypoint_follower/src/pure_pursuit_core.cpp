/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/*************************************
 * Program modified by Junlong Gao
 ************************************/

#include "ros/ros.h"
#include "pure_pursuit_core.h"
#define DEBUG

namespace waypoint_follower
{

void PurePursuit::callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
  current_pose_.header = msg->header;
  current_pose_.pose = msg->pose;
  pose_set_ = true;
}//processing frequency
//void PurePursuit::callbackFromCurrentPoseR(const autogo_msgs::localization_status& msg)
void PurePursuit::callbackFromCurrentPoseR(const autogo_msgs::TrajectoryConstPtr &msg)
{
  tf::StampedTransform transform;
    try {
      tf_.lookupTransform("/rs_odom", "/base_link", ros::Time(0), transform);
      current_pose_.pose.position.x = transform.getOrigin().x();
      current_pose_.pose.position.y = transform.getOrigin().y();
      current_pose_.pose.position.z = transform.getOrigin().z();
      current_pose_.pose.orientation.x= transform.getRotation().getX();
      current_pose_.pose.orientation.y= transform.getRotation().getY();
      current_pose_.pose.orientation.z= transform.getRotation().getZ();
      current_pose_.pose.orientation.w= transform.getRotation().getW();
      pose_set_ = true;
    } catch (tf::TransformException& ex) {
      pose_set_ = false;
      ROS_ERROR("%s", ex.what());
    }
}//processing frequency

void PurePursuit::callbackFromCurrentVelocity(const geometry_msgs::TwistStampedConstPtr &msg)
{
  current_velocity_ = *msg;
  velocity_set_ = true;
}

void PurePursuit::callbackFromCurrentVelocityR(const geometry_msgs::TwistStampedConstPtr &msg)
{
  current_velocity_ = *msg;
  velocity_set_ = true;
}

void PurePursuit::callbackFromKeyBoard(const std_msgs::Char::ConstPtr msg)
{
  if(msg->data== ' ') autonomous_activate_ =! autonomous_activate_;
}

void PurePursuit::callbackFromWayPoints1(const styx_msgs::LaneConstPtr &msg)
{
  current_waypoints_.setPath(*msg);
  waypoint_set_ = true;
  // ROS_INFO_STREAM("waypoint subscribed");
}

void PurePursuit::callbackFromWayPoints(const autogo_msgs::TrajectoryConstPtr &msg)
{
	current_waypoints_.setPathR(*msg);
    waypoint_set_ = true;
    // ROS_INFO_STREAM("waypoint subscribed");
}

double PurePursuit::getCmdVelocity(int waypoint) const
{
  if (current_waypoints_.isEmpty())
  {
    ROS_INFO_STREAM("waypoint : not loaded path");
    return 0;
  }

  double velocity = current_waypoints_.getWaypointVelocityMPS(waypoint);

  #ifdef DEBUG1
  ROS_INFO("point velocity =%f", velocity);
  #endif
  double speed_limit = speed_limit_ / 3.6;
  if (velocity > speed_limit)
	  velocity = speed_limit;
  // ROS_INFO_STREAM("waypoint : " << mps2kmph(velocity) << " km/h ( " << velocity << "m/s )");
  return velocity;
}

void PurePursuit::calcLookaheadDistance(int waypoint)
{
  double current_velocity_mps = current_velocity_.twist.linear.x;
  double maximum_lookahead_distance =  current_velocity_mps * 10;
   maximum_lookahead_distance = maximum_lookahead_distance < minimum_lookahead_distance_ ? 
                                      minimum_lookahead_distance_ : maximum_lookahead_distance;
  double ld  = current_velocity_mps * lookahead_distance_calc_ratio_;
  double ldd = minimum_lookahead_distance_ + sqrt(current_velocity_mps) * 5;

  lookahead_distance_ = ld < minimum_lookahead_distance_ ? minimum_lookahead_distance_
                      : ld > maximum_lookahead_distance ? maximum_lookahead_distance
                      : ld ;

  feedback_distance_ = ldd > lookahead_distance_ ? lookahead_distance_ : ldd;

//  ROS_INFO("lookahead distance: %f",lookahead_distance_);

  return ;
}

double PurePursuit::calcCurvature(geometry_msgs::Point target) const
{
  double kappa;double b2c_dis = 1.0;
  int type = 2;
  geometry_msgs::Point transpoint;
  transpoint = current_pose_.pose.position;
//  if(type ==1){ // use front wheel as new center
//	  transpoint.x = transpoint.x + wheel_base_ * cos(tf::getYaw(current_pose_.pose.orientation));
//	  transpoint.y = transpoint.y + wheel_base_ * sin(tf::getYaw(current_pose_.pose.orientation));
//  }else if (type == 2){ // use car center as new center
//	  transpoint.x = transpoint.x + b2c_dis * cos(tf::getYaw(current_pose_.pose.orientation));
//	  transpoint.y = transpoint.y + b2c_dis * sin(tf::getYaw(current_pose_.pose.orientation));
//  }

  double denominator = pow(getPlaneDistance(target, transpoint), 2);
  /*==========calcRelativeCoordinate解决的是目标点在当前点坐标系下的横向偏移===============*/
  double numerator = 2 * calcRelativeCoordinate(target, current_pose_.pose).y;
  if (denominator != 0){
    kappa = numerator / denominator;
  }else{
    if(numerator > 0){kappa = KAPPA_MIN_;}
    else             {kappa = -KAPPA_MIN_;}
  }
  ROS_INFO_STREAM("in pure pursuit core, kappa :" << kappa);
  return kappa;
}

double PurePursuit::calcAcceleration() const
{
  double vo = current_velocity_.twist.linear.x;
  double lookahead_seconds = lookahead_distance_calc_ratio_;
  double lookahead_min_meters = minimum_lookahead_distance_;//2.0
  double lookahead = vo * lookahead_seconds;
  double dx = 0.0;
  double vf = 0.0;
  if(lookahead < lookahead_min_meters) { lookahead = lookahead_min_meters; }
  int i = getClosestWaypoint(current_waypoints_.getCurrentWaypoints(),current_pose_.pose);
#ifdef DEBUG
  ROS_INFO("calc calcAcceleration pt i = %d", i);
#endif
  while(i < current_waypoints_.getSize())
  {
    dx = getPlaneDistance(current_waypoints_.getWaypointPosition(i),
                          current_pose_.pose.position);
    //ROS_INFO("dx=%f",dx);
    vf = getCmdVelocity(i);
    if(dx > lookahead)        {break;}
    if(dx > 0.1 && vf < 0.01) {break;}
    i++;
  }
  if(dx < 0.01) {dx = 0.01;}
  double vf2 = pow(vf,2);
  double vo2 = pow(vo,2);
  double a = (vf2 - vo2) / (2 * dx);
  ROS_INFO("calc acceeraltion = %f",a);
  if(a < -10.0) { a = -10.0; }
  /*if(a < -9) {
    ROS_ERROR_STREAM("pure_pursuit: HARD BRAKE i=" << i << "/" << current_waypoints_.getSize() << " dx=" << dx
                     << " vo=" << vo << " vf=" << vf << " CmdVel=" << getCmdVelocity(i));
  } else {
    ROS_ERROR_STREAM("pure_pursuit: wp=" << i << " dx=" << dx << " vo=" << vo << " vf=" << vf << " a=" << a);
  }*/
  return vf;//a
}

/*=========================linear interpolation of next target============================*/
bool PurePursuit::interpolateNextTarget(int next_waypoint, geometry_msgs::Point *next_target) const
{
  constexpr double ERROR = pow(10, -5);  // 0.00001

  int path_size = static_cast<int>(current_waypoints_.getSize());
  if (next_waypoint == path_size - 1)
  {
    *next_target = current_waypoints_.getWaypointPosition(next_waypoint);
    return true;
  }
  double search_radius = lookahead_distance_;
  geometry_msgs::Point zero_p;
  geometry_msgs::Point end = current_waypoints_.getWaypointPosition(next_waypoint);
  geometry_msgs::Point start = current_waypoints_.getWaypointPosition(next_waypoint - 1);

  // let the linear equation be "ax + by + c = 0"
  // if there are two points (x1,y1) , (x2,y2), a = "y2-y1, b = "(-1) * x2 - x1" ,c = "(-1) * (y2-y1)x1 + (x2-x1)y1"
  double a = 0;
  double b = 0;
  double c = 0;
  double get_linear_flag = getLinearEquation(start, end, &a, &b, &c);
  if (!get_linear_flag)
    return false;

  // let the center of circle be "(x0,y0)", in my code , the center of circle is vehicle position
  // the distance  "d" between the foot of a perpendicular line and the center of circle is ...
  //    | a * x0 + b * y0 + c |
  // d = -------------------------------
  //          √( a~2 + b~2)
  double d = getDistanceBetweenLineAndPoint(current_pose_.pose.position, a, b, c);

  // ROS_INFO("a : %lf ", a);
  // ROS_INFO("b : %lf ", b);
  // ROS_INFO("c : %lf ", c);
  // ROS_INFO("distance : %lf ", d);

  if (d > search_radius)
    return false;

  // unit vector of point 'start' to point 'end'
  tf::Vector3 v((end.x - start.x), (end.y - start.y), 0);
  tf::Vector3 unit_v = v.normalize();

  // normal unit vectors of v
  tf::Vector3 unit_w1 = rotateUnitVector(unit_v, 90);   // rotate to counter clockwise 90 degree
  tf::Vector3 unit_w2 = rotateUnitVector(unit_v, -90);  // rotate to counter clockwise 90 degree

  // the foot of a perpendicular line
  geometry_msgs::Point h1;
  h1.x = current_pose_.pose.position.x + d * unit_w1.getX();
  h1.y = current_pose_.pose.position.y + d * unit_w1.getY();
  h1.z = current_pose_.pose.position.z;

  geometry_msgs::Point h2;
  h2.x = current_pose_.pose.position.x + d * unit_w2.getX();
  h2.y = current_pose_.pose.position.y + d * unit_w2.getY();
  h2.z = current_pose_.pose.position.z;

  // ROS_INFO("error : %lf", error);
  // ROS_INFO("whether h1 on line : %lf", h1.y - (slope * h1.x + intercept));
  // ROS_INFO("whether h2 on line : %lf", h2.y - (slope * h2.x + intercept));

  // check which of two foot of a perpendicular line is on the line equation
  geometry_msgs::Point h;
  if (fabs(a * h1.x + b * h1.y + c) < ERROR)
  {
    h = h1;
    //   ROS_INFO("use h1");
  }
  else if (fabs(a * h2.x + b * h2.y + c) < ERROR)
  {
    //   ROS_INFO("use h2");
    h = h2;
  }
  else
  {
    return false;
  }

  // get intersection[s]
  // if there is a intersection
  if (d == search_radius)
  {
    *next_target = h;
    return true;
  }
  else
  {
    // if there are two intersection
    // get intersection in front of vehicle
    double s = sqrt(pow(search_radius, 2) - pow(d, 2));
    geometry_msgs::Point target1;
    target1.x = h.x + s * unit_v.getX();
    target1.y = h.y + s * unit_v.getY();
    target1.z = current_pose_.pose.position.z;

    geometry_msgs::Point target2;
    target2.x = h.x - s * unit_v.getX();
    target2.y = h.y - s * unit_v.getY();
    target2.z = current_pose_.pose.position.z;

    // ROS_INFO("target1 : ( %lf , %lf , %lf)", target1.x, target1.y, target1.z);
    // ROS_INFO("target2 : ( %lf , %lf , %lf)", target2.x, target2.y, target2.z);
    //displayLinePoint(a, b, c, target1, target2, h);  // debug tool

    // check intersection is between end and start
    double interval = getPlaneDistance(end, start);
    if (getPlaneDistance(target1, end) < interval)
    {
      // ROS_INFO("result : target1");
      *next_target = target1;
      return true;
    }
    else if (getPlaneDistance(target2, end) < interval)
    {
      // ROS_INFO("result : target2");
      *next_target = target2;
      return true;
    }
    else
    {
      // ROS_INFO("result : false ");
      return false;
    }
  }
}

bool PurePursuit::verifyFollowing() const
{
  double a = 0;
  double b = 0;
  double c = 0;
  getLinearEquation(current_waypoints_.getWaypointPosition(1), current_waypoints_.getWaypointPosition(2), &a, &b, &c);
  double displacement = getDistanceBetweenLineAndPoint(current_pose_.pose.position, a, b, c);
  double relative_angle = getRelativeAngle(current_waypoints_.getWaypointPose(1), current_pose_.pose);
#ifdef DEBUG1
  ROS_ERROR("side diff : %lf , angle diff : %lf",displacement,relative_angle);
#endif
  if (displacement < displacement_threshold_ && relative_angle < relative_angle_threshold_)
  {
     ROS_INFO("Following : True");
    return true;
  }
  else
  {
     ROS_INFO("Following : False");
    return false;
  }
}
geometry_msgs::Twist PurePursuit::calcTwist(double curvature, double cmd_velocity) const
{
  // verify whether vehicle is following the path
  bool following_flag = verifyFollowing();

  static double prev_angular_velocity = 0;

  geometry_msgs::Twist twist;

  twist.linear.x = cmd_velocity;

  if (!following_flag)
  {
    ROS_ERROR_STREAM("Not following");
    twist.angular.z = current_velocity_.twist.linear.x * curvature;
    ROS_INFO("twist augular=%f",twist.angular.z);
  }
  else
  {
    twist.angular.z = prev_angular_velocity;
  }

  prev_angular_velocity = twist.angular.z;
  return twist;
}

void PurePursuit::getNextWaypoint()
{
  int path_size = static_cast<int>(current_waypoints_.getSize());

  /* ===============if waypoints are not given, do nothing=====================================*/
  if (path_size == 0)
  {
    num_of_next_waypoint_ = -1;
    num_of_feedback_waypoint_ = -1;
    return;
  }
  double length = 0.0;
  float  softbreak_thres = 3.0; //while less than this value, algorithm will trigger soft break action
  // look for the next waypoint.
  for (int i = 0; i < path_size; i++)
  {
    // if search waypoint is the last
	length = getPlaneDistance(current_waypoints_.getWaypointPosition(i), current_pose_.pose.position);
	// while reach the end of traj, if length less than a minimal threshold, means reach the end of the trajectroy, will directly trigger the softbreak logic
    if (i == (path_size - 1))
    {
      ROS_INFO("search waypoint is the last");
      if(length >= softbreak_thres){
    	  num_of_feedback_waypoint_ = i;
    	  num_of_next_waypoint_ = i;
      }else{
    	  num_of_feedback_waypoint_ = -1;
    	  num_of_next_waypoint_ = -1;
      }
      return;
    }
    /*==========if there exists an effective waypoint  ========================================*/
    if (length > feedback_distance_ )
    	num_of_feedback_waypoint_ = i;

    if (length > lookahead_distance_)
    {
      num_of_next_waypoint_ = i;
#ifdef DEBUG1
      ROS_ERROR_STREAM("wp = " << i << " dist = " << getPlaneDistance(current_waypoints_.getWaypointPosition(i), current_pose_.pose.position) );
#endif
      return;
    }
  }
  /*========== if this program reaches here , it means we lost the waypoint!====================*/
  num_of_feedback_waypoint_ = -1;
  num_of_next_waypoint_ = -1;
  return;
}

geometry_msgs::TwistStamped PurePursuit::keepStatus() const
{
  geometry_msgs::TwistStamped twist;
  twist.twist.linear.x = 0.8 * current_velocity_.twist.linear.x;
  twist.twist.linear.y = 1; // keep status
  twist.twist.angular.z = 0.8 * current_velocity_.twist.angular.z;
  twist.header.stamp = ros::Time::now();
  return twist;
}

geometry_msgs::TwistStamped PurePursuit::softBrake() const
{
  geometry_msgs::TwistStamped twist;
  twist.twist.linear.x = 0.5 * current_velocity_.twist.linear.x;
  twist.twist.linear.y = 2; // softbrake status
  twist.twist.angular.z = 0.5 * current_velocity_.twist.angular.z;
  twist.header.stamp = ros::Time::now();
  return twist;
}

geometry_msgs::TwistStamped PurePursuit::outputZero() const
{
  geometry_msgs::TwistStamped twist;
  twist.twist.linear.x = 0;
  twist.twist.linear.z = 1; // emergency status
  twist.twist.angular.z = 0;
  twist.header.stamp = ros::Time::now();
  return twist;
}

geometry_msgs::TwistStamped PurePursuit::outputTwist(geometry_msgs::Twist t) const
{
  double g_lateral_accel_limit = 5.0;
  double ERROR = 1e-8;

  geometry_msgs::TwistStamped twist;
  twist.twist = t;
  twist.header.stamp = ros::Time::now();

  double v = t.linear.x;
  double omega = t.angular.z;

  if(fabs(omega) < ERROR){

    return twist;
  }

  double max_v = fabs(g_lateral_accel_limit / omega);

  double a = v * omega;

  ROS_INFO("lateral accel = %lf", a);

  twist.twist.linear.x = fabs(a) > g_lateral_accel_limit ? max_v
                    : v;
  twist.twist.angular.z = omega;

  ROS_INFO("max_v = %f, vel_cmd=%f,yarrate_cmd=%f", max_v, twist.twist.linear.x,twist.twist.angular.z);

  return twist;
}

double PurePursuit::linearInterp(const double& x, const double& a, const double& b)
{
	double tmp;
	tmp = a * x + b;
	return tmp;
}

double PurePursuit::getLatDis(const double& length,const geometry_msgs::PoseStamped& front_wheel)
{
	geometry_msgs::Pose p1, p2;
	double v1x=0, v1y=0, v2x=0, v2y=0;//v1 is  vector of p1p2, v2 is vector of pegop2
	double comp1=0.0, comp2=0.0, comp3 = 0.0, comp4=0.0, angle = 0.0;
	int direction = 0;
	p2 = current_waypoints_.getWaypointPose(num_of_feedback_waypoint_);
	if(num_of_feedback_waypoint_ != 0){
		p1 = current_waypoints_.getWaypointPose(num_of_feedback_waypoint_ -1);
	}else{ //special case
		double yaw = tf::getYaw(p2.orientation);
		p1.position.x = p2.position.x - 1 * sin(yaw);
		p1.position.y = p2.position.y - 1 * cos(yaw);
		p1.orientation = p2.orientation;
	}
	v1x = p2.position.x-p1.position.x;               v1y = p2.position.y-p1.position.y;
    v2x = p2.position.x-front_wheel.pose.position.x; v2y = p2.position.y-front_wheel.pose.position.y;
    comp1 = (p2.position.x-front_wheel.pose.position.x)*(p2.position.x-p1.position.x);
    comp2 = (p2.position.y-front_wheel.pose.position.y)*(p2.position.y-p1.position.y);
    comp3 = sqrt(pow(v2x,2)+pow(v2y,2));
    comp4 = sqrt(pow(v1x,2)+pow(v1y,2));
    direction = Sign(v1x*v2y - v2x*v1y);
    angle = 1 * direction * acos((comp1 + comp2)/(comp3 * comp4));
#ifdef DEBUG1
//    ROS_ERROR("in Get lat dis, feedback_waypoint_num = %d, p2.x= %f, p2.y= %f, p1.x= %f, p1.y= %f, egof.x= %f, egof.y= %f", num_of_feedback_waypoint_, p2.position.x, p2.position.y,
//    		   p1.position.x, p1.position.y, front_wheel.pose.position.x, front_wheel.pose.position.y);
    ROS_ERROR_STREAM("angle = "<<angle<<", sin(angle)="<<sin(angle));
#endif
    return length*sin(angle);
}

double PurePursuit::getCompSteer(const geometry_msgs::Pose& tarPose, const geometry_msgs::PoseStamped& front_wheel, const geometry_msgs::PoseStamped& car_center,
		                         double& lateral_dis_error1, double& lateral_dis_error2, double& real_ori_error, double& comp1, double& comp2)
{
	/*====Get steer compensate control component==========*/
	  double dFai = 0.0, dDelta = 0.0;
	  double total_length = 0.0;
	  double x1 = 6.9, x2 = 11.1, y1 = 2.4, y2 = 3.0, k1 = 0, b1 = 0;
	/*====factor k should be considered by experiment=====*/
      double kd = 0.0, ky = 1.0, yaw1 =0.0, yaw2= 0.0;
      double v = 0;
      k1 = (y2 - y1) /(x2 - x1);
      b1 = y1 - k1 * x1;
      v = round(current_velocity_.twist.linear.x * 10)/10;
      if( v < x1)
    	  kd = 0.35 * v;
      else if (v >= x1 && v < x2)
    	  kd = k1 * v + b1;
      else
    	  kd = 0.175 * v;
//      kd = 3.0;
	  total_length = sqrt(pow(tarPose.position.x-front_wheel.pose.position.x,2)+pow(tarPose.position.y-front_wheel.pose.position.y,2));
	  lateral_dis_error1 = getLatDis(total_length, front_wheel);
	  lateral_dis_error2 = getLatDis(total_length, car_center);
	  yaw1 = tf::getYaw(tarPose.orientation);
	  yaw2 = tf::getYaw(current_pose_.pose.orientation);
	  dFai = ky * (yaw1 - yaw2);
	  if (dFai > M_PI) { dFai -= 2*M_PI;}
	  else if (dFai < -M_PI) { dFai += 2*M_PI;}
	  real_ori_error = dFai;
	  dDelta = atan2(kd * lateral_dis_error1, current_velocity_.twist.linear.x);
	  double sum = dFai + dDelta;
//	  sum = dFai;
	  if (fabs(sum) > 0.44) sum = Sign(sum) * 0.44;
#ifdef DEBUG1
	  ROS_ERROR("in pure_pursuit_core, dFai = %f, dDelta = %f, total_length = %f", dFai, dDelta, total_length);
//	  ROS_ERROR("in pure_pursuit_core, tar.theta = %f, ego.theta = %f", yaw1, yaw2);
#endif
	  comp1 = dFai;
	  comp2 = dDelta;
	  return sum;
}

void  PurePursuit::getFFFactor(const double& lateral_err, const double& ori_err, double& ff_factor, double& fb_factor)
{
	double lateral_err_buffer = 10.0;
	double lateral_err_thres_up = 3.0;//0.75; //upper bound
	double lateral_err_thres_lo = 0.3; //lower bound
	double angle_thres = M_PI * 10/180.0;
	double max = 1, maxb = 0.5;
	double min = 1 - max; double minb = 1 - maxb;
	double k1 = 0, k2 = 0, b1 = 0, b2 = 0;//1 for ff (pure pursuit), 2 for fb
	double k1b = 0, k2b = 0, b1b = 0, b2b = 0;
	/*==========calc slope factors of ff and fb respectively=========*/
	k1 = (max - min) / (lateral_err_thres_up - lateral_err_thres_lo); k2 = -k1;
	b1 = max - k1 * lateral_err_thres_up;
	b2 = max - k2 * lateral_err_thres_lo;
	k1b = (maxb - minb) / (lateral_err_buffer - lateral_err_thres_up); k2b = -k1b;
	b1b = maxb - k1b * lateral_err_buffer;
	b2b = maxb - k2b * lateral_err_thres_up;
	double x = fabs(lateral_err); double y = fabs(ori_err);
	/*==========calc slope interp of feedforward and feedback factors*/
	if (x > lateral_err_thres_lo){// && y < angle_thres){
		if(x< lateral_err_thres_up){
			ROS_ERROR("trigger region 1.1");
			ff_factor = k1 * x + b1;
			fb_factor = k2 * x + b2;
		}else if (x>= lateral_err_thres_up && x< lateral_err_buffer){
			ROS_ERROR("trigger region 1.2");
			ff_factor = k1b * x + b1b;
			fb_factor = k2b * x + b2b;
	    }else{
	    	ROS_ERROR("trigger region 1.3");
			ff_factor = 0.5; fb_factor = 0.5;
		}
	}//else if (x > lateral_err_thres_lo && y >= angle_thres){
//		ROS_ERROR("trigger region 2");
//		ff_factor = 0; fb_factor = 1;
//	}
	else if (x <= lateral_err_thres_lo){
		ROS_ERROR("trigger region 3");
		ff_factor = 0; fb_factor = 1;
	}
}

geometry_msgs::TwistStamped PurePursuit::go(geometry_msgs::TwistStamped& debug_pub)
{
  /*=============Initial Status Check and bad case handle==================*/
  if(!pose_set_ || !waypoint_set_ || !velocity_set_){
    if(!pose_set_) {
       ROS_WARN("position is missing");
     }
     if(!waypoint_set_) {
       ROS_WARN("waypoint is missing");
     }
     if(!velocity_set_) {
       ROS_WARN("velocity is missing");
    }
    return outputZero();
  }
  /*=============Inner Loop Variable Definition================================================*/
  int emergency_count = 8;
  bool interpolate_flag = false;
  double ori_error = 0.0;
  double comp_steer = 0.0, comp_1 = 0.0, comp_2 = 0.0;
  double comp_kappa = 0.0;
  double ff_factor = 0.0, fb_factor = 0.0, fb_lateral_err = 0.0, fc_lateral_err = 0.0;
  geometry_msgs::PoseStamped frontwheel, carcenter;
  geometry_msgs::Pose tar;

  /*=============Calc lookahead distance and search lookahead waypoint==========================*/
  calcLookaheadDistance(1);
  // search next waypoint
  getNextWaypoint();
  /*=============Find next waypoint bad case handling===========================================*/
  if (num_of_next_waypoint_ == -1)
  {
	  lost_waypt_count +=  1;
      ROS_WARN("lost next waypoint");
      if(lost_waypt_count > emergency_count)
    	  return outputZero();
      else
    	  return softBrake();
  }else{
	  lost_waypt_count = 0;
  }
#ifdef DEBUG1
  ROS_ERROR_STREAM("linear_itp = "<<linear_interpolate_<<", traj size="<<current_waypoints_.getSize()<<", chosen waypoint = " 
                    <<  num_of_next_waypoint_ + 1<<", cmd_velocity ="<<current_waypoints_.getWaypointVelocityMPS(num_of_next_waypoint_));
#endif

  /*===========front wheel pos transfer==========use back wheel instead for testing========================*/
  frontwheel = current_pose_;
  frontwheel.pose.position.x = current_pose_.pose.position.x + wheel_base_ * cos(tf::getYaw(current_pose_.pose.orientation));
  frontwheel.pose.position.y = current_pose_.pose.position.y + wheel_base_ * sin(tf::getYaw(current_pose_.pose.orientation));
  carcenter = current_pose_;
  carcenter.pose.position.x = current_pose_.pose.position.x + 1.0 * cos(tf::getYaw(current_pose_.pose.orientation));
  carcenter.pose.position.y = current_pose_.pose.position.y + 1.0 * sin(tf::getYaw(current_pose_.pose.orientation));

  /* ==========if g_linear_interpolate_mode is false or next waypoint is first or last============*/
  if (!linear_interpolate_ || num_of_next_waypoint_ == 0 || num_of_next_waypoint_ == (static_cast<int>(current_waypoints_.getSize() - 1)))
  {
    ROS_WARN("go into NONINTERP");
    position_of_next_target_ = current_waypoints_.getWaypointPosition(num_of_next_waypoint_);
    position_of_fb_target_ = current_waypoints_.getWaypointPosition(num_of_feedback_waypoint_);
    /*=========Feedback steering calc=============================================================*/
    tar.position = position_of_fb_target_;
    tar.orientation = current_waypoints_.getWaypointOrientation(num_of_feedback_waypoint_);
    comp_steer = getCompSteer(tar, frontwheel, carcenter, fb_lateral_err, fc_lateral_err, ori_error, comp_1, comp_2);
    getFFFactor(fb_lateral_err, ori_error, ff_factor , fb_factor);
    if(comp_steer!=0){
      comp_kappa = Sign(comp_steer)*(tan(fabs(comp_steer))/wheel_base_);//back wheel radius
    }else{comp_kappa = 0;}
    debug_pub.header.stamp=ros::Time::now();
    debug_pub.twist.linear.x  = fb_lateral_err;
    debug_pub.twist.linear.y  = comp_kappa;
    debug_pub.twist.angular.x = comp_1*180/M_PI;
    debug_pub.twist.angular.y = comp_2*180/M_PI;
    debug_pub.twist.angular.z = comp_steer*180/M_PI;
    /*=========Control Output====================================================================*/
//    ff_factor = 1; fb_factor = 0;
    return outputTwist(calcTwist(ff_factor * calcCurvature(position_of_next_target_) + fb_factor * comp_kappa, getCmdVelocity(3)));
//    return outputTwist(calcTwist(calcCurvature(position_of_next_target_), getCmdVelocity(3)));
  }

  /*===========linear interpolation and calculate angular velocity===============================*/
  interpolate_flag = interpolateNextTarget(num_of_next_waypoint_, &position_of_next_target_);
  bool interp_fb_flag = interpolateNextTarget(num_of_feedback_waypoint_, &position_of_fb_target_);
  /*===========interpolation bad case handling===================================================*/
  if (!interpolate_flag)
  {
	lost_tar_count += 1;
    ROS_ERROR_STREAM("lost target! ");
    if(lost_tar_count > emergency_count)
        return outputZero();
	else
		return keepStatus();
  }else{lost_tar_count = 0;}
  /*=========Feedback steering calc================================================================*/
  tar.position = position_of_fb_target_;
  tar.orientation = current_waypoints_.getWaypointOrientation(num_of_feedback_waypoint_); // use this instead of interp from 2 pts
  comp_steer = getCompSteer(tar, frontwheel, carcenter, fb_lateral_err, fc_lateral_err, ori_error, comp_1, comp_2);
  getFFFactor(fb_lateral_err, ori_error, ff_factor , fb_factor);
  if(comp_steer!=0){
	comp_kappa = Sign(comp_steer)*(tan(fabs(comp_steer))/wheel_base_);//back wheel radius
  }else{comp_kappa = 0;}
  ROS_WARN("next_target : ( %lf , %lf , %lf)", position_of_next_target_.x, position_of_next_target_.y,position_of_next_target_.z);
  /*=========Control Output========================================================================*/
  debug_pub.header.stamp=ros::Time::now();
  debug_pub.twist.linear.x  = fb_lateral_err;
  debug_pub.twist.linear.y  = comp_kappa;
  debug_pub.twist.angular.x = comp_1*180/M_PI;
  debug_pub.twist.angular.y = comp_2*180/M_PI;
  debug_pub.twist.angular.z = comp_steer*180/M_PI;
//  ff_factor = 1; fb_factor = 0;
  return outputTwist(calcTwist(ff_factor * calcCurvature(position_of_next_target_) + fb_factor * comp_kappa, calcAcceleration()));
//  return outputTwist(calcTwist(calcCurvature(position_of_next_target_), calcAcceleration()));
#ifdef LOG
  std::ofstream ofs("/tmp/pure_pursuit.log", std::ios::app);
  ofs << _current_waypoints.getWaypointPosition(next_waypoint).x << " "
      << _current_waypoints.getWaypointPosition(next_waypoint).y << " " << next_target.x << " " << next_target.y
      << std::endl;
#endif
}
}
