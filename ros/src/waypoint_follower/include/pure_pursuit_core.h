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

#ifndef PURE_PURSUIT_CORE_H
#define PURE_PURSUIT_CORE_H

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <dbw_mkz_msgs/SteeringReport.h>
#include "autogo_msgs/localization_status.h"
#include <tf/tf.h>
// C++ includes
#include <memory>
#include <std_msgs/Char.h>
#include "libwaypoint_follower.h"

enum class Mode
{
  waypoint,
  dialog
};


namespace waypoint_follower
{
class PurePursuit
{
private:
  tf::TransformListener& tf_;
  //constant
  const double RADIUS_MAX_;
  const double KAPPA_MIN_;
  const double wheel_base_;
  const double speed_limit_;
  bool linear_interpolate_;

  // config topic
  int param_flag_;              // 0 = waypoint, 1 = Dialog
  double const_lookahead_distance_;  // meter
  double initial_velocity_;     // km/h
  double lookahead_distance_calc_ratio_;
  double minimum_lookahead_distance_;  // the next waypoint must be outside of this threshold.
  double displacement_threshold_;
  double relative_angle_threshold_;

  bool waypoint_set_;
  bool pose_set_;
  bool velocity_set_;
  int num_of_next_waypoint_;
  int num_of_feedback_waypoint_;
  geometry_msgs::Point position_of_next_target_;
  geometry_msgs::Point position_of_fb_target_;
  double lookahead_distance_;
  double feedback_distance_;
  int lost_waypt_count = 0;
  int lost_tar_count = 0;


  geometry_msgs::PoseStamped current_pose_;
  geometry_msgs::TwistStamped current_velocity_;
  WayPoints current_waypoints_;


  double getCmdVelocity(int waypoint) const;
  void   calcLookaheadDistance(int waypoint);
  double calcCurvature(geometry_msgs::Point target) const;
  double calcRadius(geometry_msgs::Point target) const;
  bool   interpolateNextTarget(int next_waypoint, geometry_msgs::Point *next_target) const;
  bool   verifyFollowing() const;
  geometry_msgs::Twist calcTwist(double curvature, double cmd_velocity) const;
  void   getNextWaypoint();
  double linearInterp(const double& x, const double& a, const double& b);
  double getLatDis(const double& length,const geometry_msgs::PoseStamped& front_wheel);
  double getCompSteer(const geometry_msgs::Pose& tarPose, const geometry_msgs::PoseStamped& front_wheel, const geometry_msgs::PoseStamped& car_center,
                      double& lateral_dis_error1, double& lateral_dis_error2, double& real_ori_error, double& comp1, double& comp2);
  void   getFFFactor(const double& lateral_err, const double& ori_err, double& ff_factor, double& fb_factor);
  template <class T>
  int Sign (T a) {if (a>0) return 1; else if (a<0) return -1; else return 0;}
  geometry_msgs::TwistStamped outputZero() const;
  geometry_msgs::TwistStamped keepStatus() const;
  geometry_msgs::TwistStamped softBrake() const;
  geometry_msgs::TwistStamped outputTwist(geometry_msgs::Twist t) const;
  double calcAcceleration() const;

public:
  PurePursuit(bool linear_interpolate_mode,  tf::TransformListener& tf)
    : RADIUS_MAX_(9e10)
    , KAPPA_MIN_(1/RADIUS_MAX_)
    , linear_interpolate_(linear_interpolate_mode)
    , param_flag_(0)
    , const_lookahead_distance_(4.0)
    , initial_velocity_(5.0)
    , lookahead_distance_calc_ratio_(1.0)
    , minimum_lookahead_distance_(3.5)
    , displacement_threshold_(0.05)
    , relative_angle_threshold_(0.1)
    , waypoint_set_(false)
    , pose_set_(false)
    , velocity_set_(false)
    , num_of_next_waypoint_(-1)
    , num_of_feedback_waypoint_(-1)
    , lookahead_distance_(0)
    , feedback_distance_(0)
    , tf_(tf)
    , wheel_base_(2.85)
    , speed_limit_(80)
  {
  }
  ~PurePursuit()
  {
  }

  // for ROS
  void callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg);
  void callbackFromCurrentPoseR(const autogo_msgs::TrajectoryConstPtr &msg);
//  void callbackFromCurrentPoseR(const autogo_msgs::localization_status& msg);
  void callbackFromCurrentVelocity(const geometry_msgs::TwistStampedConstPtr &msg);
  void callbackFromCurrentVelocityR(const geometry_msgs::TwistStampedConstPtr &msg);
  void callbackFromWayPoints1(const styx_msgs::LaneConstPtr &msg);
  void callbackFromWayPoints(const autogo_msgs::TrajectoryConstPtr &msg);
  void callbackFromKeyBoard(const std_msgs::Char::ConstPtr msg);
  bool autonomous_activate_ = false;
  bool getAutonomousRight(){ return autonomous_activate_;}
  // for debug
  geometry_msgs::Point getPoseOfNextWaypoint() const
  {
    return current_waypoints_.getWaypointPosition(num_of_next_waypoint_);
  }
  geometry_msgs::Point getPoseOfNextTarget() const
  {
    return position_of_next_target_;
  }
  geometry_msgs::Pose getCurrentPose() const
  {
    return current_pose_.pose;
  }

  double getLookaheadDistance() const
  {
    return lookahead_distance_;
  }
  // processing for ROS
  geometry_msgs::TwistStamped go(geometry_msgs::TwistStamped& debug_pub);
};
}

#endif  // PURE_PURSUIT_CORE_H
