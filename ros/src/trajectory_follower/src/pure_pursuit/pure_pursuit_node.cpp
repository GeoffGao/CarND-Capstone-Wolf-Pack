/**
 * created by Ye Bo
 * date: 2017-11-30
 */

#include <ros/ros.h>

#include "pure_pursuit_core.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pure_pursuit");
	trajectory_follower::PurePursuitNode ppn;
	ppn.run();
	
	ros::spin();
}