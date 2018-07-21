/**
 * created by Ye Bo
 * date: 2017-11-30
 */

#include "pure_pursuit.h"

namespace trajectory_follower {
PurePursuit::PurePursuit()
	: next_waypoint_number_(-1)
	, lookahead_distance_(0)
	, current_linear_velocity_(0)
	, RADIUS_MAX_(9e10)
	, KAPPA_MIN_( 1 / RADIUS_MAX_)
	, is_linear_interpolation_(false)
{

}

PurePursuit::~PurePursuit()
{

}

bool PurePursuit::canGetCurvature(double* output_kappa)
{
	//寻找符合要求(大于预瞄点距离)的下一个waypoint
	//return id: next_waypoint_number_
	getNextWayPoint();
	
	if(next_waypoint_number_ == -1)
	{
		ROS_INFO("pure_pursuit :lost next waypoint");
		return false;
	}
	
	//is_linear_interpolation: default: true
	//当非线性插值,或者下一个waypoint是第一个和最后一个点时,直接计算下一个目标位置,并计算k
	if(!is_linear_interpolation_ || next_waypoint_number_ == 0 || next_waypoint_number_ == (static_cast<int>(current_trajectory_.size() - 1)))
	{
		next_target_position_ = current_trajectory_.at(next_waypoint_number_).pose.position;
		
		//利用下一个位置计算曲率
		*output_kappa = calcCurvature(next_target_position_);
		return true;
	}
	
	//如果下一个waypointpoint不是是首尾点,进行线性插值
	bool interpolation = interpolateNextTarget(next_waypoint_number_, &next_target_position_);
	
	if (!interpolation)
	{
		ROS_INFO_STREAM("pure_pursuit: lost target! ");
		return false;
	}
	*output_kappa = calcCurvature(next_target_position_);
	return true;
}

bool PurePursuit::interpolateNextTarget(int next_waypoint, geometry_msgs::Point* next_target) const
{
	const double ERROR = pow(10, -5); //0.00001
	
	int traj_size = static_cast<int>(current_trajectory_.size());
	if (next_waypoint == traj_size -1)
	{
		*next_target = current_trajectory_.at(next_waypoint).pose.position;
		return true;
	}
	
	double search_radius = lookahead_distance_;
	
	geometry_msgs::Point zero_p;
	geometry_msgs::Point end = current_trajectory_.at(next_waypoint).pose.position;
	geometry_msgs::Point start = current_trajectory_.at(next_waypoint - 1).pose.position;
	
	//计算前后两点的线性方程
	// let the linear equation be "ax + by + c = 0"
	// if there are two points (x1,y1) , (x2,y2), a = "y2-y1, b = "(-1) * x2 - x1" ,c = "(-1) * (y2-y1)x1 + (x2-x1)y1"
	double a = 0;
	double b = 0;
	double c = 0;
	double get_linear_flag = getLinearEquation(start, end, &a, &b, &c);
	if (!get_linear_flag)
		return false;
	
	//计算机器人中心到该直线的垂直距离
	// let the center of circle be "(x0,y0)", in my code , the center of circle is vehicle position
	// the distance  "d" between the foot of a perpendicular line and the center of circle is ...
	//    | a * x0 + b * y0 + c |
	// d = -------------------------------
	//          √( a~2 + b~2)
	double d = getDistanceBetweenLineAndPoint(current_pose_.position, a, b, c);
	
	// ROS_INFO("a : %lf ", a);
	// ROS_INFO("b : %lf ", b);
	// ROS_INFO("c : %lf ", c);
	// ROS_INFO("distance : %lf ", d);
	
	//当垂直距离大于预瞄点距离时,线性插值失败
	if (d > search_radius)
		return false;
	
	// unit vector of point 'start' to point 'end'
	tf::Vector3 v((end.x - start.x), (end.y - start.y), 0);
	tf::Vector3 unit_v = v.normalize();

	//计算前后两点单位向量的两个垂直单位向量
	// normal unit vectors of v
	tf::Vector3 unit_w1 = rotateUnitVector(unit_v, 90);   // rotate to counter clockwise 90 degree
	tf::Vector3 unit_w2 = rotateUnitVector(unit_v, -90);  // rotate to counter clockwise 90 degree
	
	//计算机器人中心到该垂线的垂足
	// the foot of a perpendicular line
	geometry_msgs::Point h1;
	h1.x = current_pose_.position.x + d * unit_w1.getX();
	h1.y = current_pose_.position.y + d * unit_w1.getY();
	h1.z = current_pose_.position.z;

	geometry_msgs::Point h2;
	h2.x = current_pose_.position.x + d * unit_w2.getX();
	h2.y = current_pose_.position.y + d * unit_w2.getY();
	h2.z = current_pose_.position.z;
	
	//检查该垂足是否在这条直线上
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
	//当垂直距离等于预瞄点距离时,下一个位置即垂足?
	if (d == search_radius)
	{
		*next_target = h;
		return true;
	}
	//当垂直距离小于预瞄点距离时:
	else
	{
		// if there are two intersection
		// get intersection in front of vehicle
		double s = sqrt(pow(search_radius, 2) - pow(d, 2));
		geometry_msgs::Point target1;
		target1.x = h.x + s * unit_v.getX();
		target1.y = h.y + s * unit_v.getY();
		target1.z = current_pose_.position.z;

		geometry_msgs::Point target2;
		target2.x = h.x - s * unit_v.getX();
		target2.y = h.y - s * unit_v.getY();
		target2.z = current_pose_.position.z;

		// ROS_INFO("target1 : ( %lf , %lf , %lf)", target1.x, target1.y, target1.z);
		// ROS_INFO("target2 : ( %lf , %lf , %lf)", target2.x, target2.y, target2.z);
		// displayLinePoint(a, b, c, target1, target2, h);  // debug tool

		//这块还不太理解???
		// check intersection is between end and start
		//保证插值点落在前后两点的线段之中
		double interval = getPlaneDistance(end, start);
		//当目标点与下一个waypoint的距离小于前后两点距离
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


double PurePursuit::calcCurvature(geometry_msgs::Point target) const
{
	double kappa;
	//参考论文http://www.docin.com/p-1154708865.html
	//车体坐标系车头朝向为x,垂直右侧为y
	//kappa = 2*y / (delta_x^2 + delta_y^2)
	double denominator = pow(getPlaneDistance(target, current_pose_.position), 2);
	//转换到车体坐标系
	double numerator = 2 * calcRelativeCoordinate(target, current_pose_).y;
	
	if(denominator != 0)
		kappa = numerator/denominator;
	else{
		if (numerator > 0)
			kappa = KAPPA_MIN_;
		else
			kappa = -1.0 * KAPPA_MIN_;
	}
	ROS_INFO("pure_pursuit: kappa: %lf", kappa);
}

void PurePursuit::getNextWayPoint()
{
	int traj_size = static_cast<int>(current_trajectory_.size());
	
	if (traj_size == 0)
	{
		next_waypoint_number_ = -1;
	}
	
	for (int i = 0; i < traj_size; i++)
	{
		// if search waypoint size is the last
		if (i == (traj_size - 1))
		{
			ROS_INFO("search waypoint is the last");
			next_waypoint_number_ = i;
			return;
		}
		//从离车辆最近点开始搜索,当满足该点与车辆距离差大于预瞄点距离时,取该点的id
		if(getPlaneDistance(current_trajectory_.at(i).pose.position, current_pose_.position) > lookahead_distance_)
		{
			next_waypoint_number_ = i;
			return;
		}	
	}
	//无法找到符合要求的下一个waypoint
	next_waypoint_number_ = -1;
	return;
}


}