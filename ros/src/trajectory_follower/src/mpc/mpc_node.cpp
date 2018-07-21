/**
 * created by Ye Bo
 * date: 2017-12-05
 */
#include "mpc_core_mkz.h"
#include "trajectory_follower/trajectory_follower_config.h"
using namespace trajectory_follower;
int main(int argc, char** argv)
{
	ros::init(argc, argv, "mpc_node");
	ros::NodeHandle nh;
	trajectory_follower::MpcNode mpc_node(nh);
	mpc_node.run();
	//ros::spin();
	return 0;
}
