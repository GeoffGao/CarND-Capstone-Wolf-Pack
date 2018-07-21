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
#include "pure_pursuit_core.h"

//#define REAL

#ifdef REAL
constexpr int LOOP_RATE = 10; //processing frequency
#else
constexpr int LOOP_RATE = 10; //processing frequency
#endif


int main(int argc, char **argv)
{


  // set up ros
  ros::init(argc, argv, "pure_pursuit");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  bool linear_interpolate_mode;
  private_nh.param("linear_interpolate_mode", linear_interpolate_mode, bool(true));
  ROS_INFO_STREAM("linear_interpolate_mode : " << linear_interpolate_mode);

  tf::TransformListener tf_;
  waypoint_follower::PurePursuit pp(linear_interpolate_mode,tf_);

  ROS_INFO("set publisher...");
  // publish topic
#ifdef REAL
  ros::Publisher cmd_velocity_publisher = nh.advertise<geometry_msgs::TwistStamped>("/vehicle/cmd_vel_stamped", 10);
#else
  ros::Publisher cmd_velocity_publisher = nh.advertise<geometry_msgs::TwistStamped>("twist_cmd", 10);
#endif
  ros::Publisher debug_info_publisher = nh.advertise<geometry_msgs::TwistStamped>("debug_info_PP",10);

  ROS_INFO("set subscriber...");
  // subscribe topic
  #ifdef REAL
  ros::Subscriber waypoint_subscriber =
          nh.subscribe("/autogo/planning/trajectory", 10, &waypoint_follower::PurePursuit::callbackFromWayPoints, &pp);
    ros::Subscriber ndt_subscriber =
         nh.subscribe("/autogo/planning/trajectory", 10, &waypoint_follower::PurePursuit::callbackFromCurrentPoseR, &pp);//fake subscribe
 	ros::Subscriber est_twist_subscriber =
 	     nh.subscribe("/vehicle/twist", 10, &waypoint_follower::PurePursuit::callbackFromCurrentVelocityR, &pp);
    ros::Subscriber kb_subscriber =
    	 nh.subscribe("/vehicle/trigger_KB",1,&waypoint_follower::PurePursuit::callbackFromKeyBoard, &pp);
#else
    ros::Subscriber waypoint_subscriber =
          nh.subscribe("final_waypoints", 10, &waypoint_follower::PurePursuit::callbackFromWayPoints1, &pp);
    ros::Subscriber ndt_subscriber =
        nh.subscribe("current_pose", 10, &waypoint_follower::PurePursuit::callbackFromCurrentPose, &pp);
	ros::Subscriber est_twist_subscriber =
	    nh.subscribe("current_velocity", 10, &waypoint_follower::PurePursuit::callbackFromCurrentVelocity, &pp);
#endif

  ROS_INFO("pure pursuit start");
  ros::Rate loop_rate(LOOP_RATE);
  geometry_msgs::TwistStamped debug_pub;
  geometry_msgs::TwistStamped final_pub;
  while (ros::ok())
  {
    ROS_INFO("PP cp1");
    ros::spinOnce();
#ifdef REAL
    bool automode = pp.getAutonomousRight();
    if(automode){
    	final_pub = pp.go(debug_pub);
    	debug_info_publisher.publish(debug_pub);
    	cmd_velocity_publisher.publish(final_pub);
    }
#else
    final_pub = pp.go(debug_pub);
	debug_info_publisher.publish(debug_pub);
	cmd_velocity_publisher.publish(final_pub);
#endif
    loop_rate.sleep();
  }

  return 0;
}
