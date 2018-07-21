/**
 * created by Ye Bo
 * date: 2017-12-01
 * modified from Autoware
 */

#ifndef _LIB_WAYPOINT_FOLLOWER_H_
#define _LIB_WAYPOINT_FOLLOWER_H_

// C++ header
#include <iostream>
#include <sstream>
#include <fstream>

// ROS header
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
//#include "autoware_msgs/lane.h"
//#include "trajectory_follower/Trajectory.h"
#include "has_local_planner/Trajectory.h"
class WayPoints
{
protected:
  has_local_planner::Trajectory current_waypoints_;

public:
  void setPath(const has_local_planner::Trajectory &waypoints)
  {
    current_waypoints_ = waypoints;
  }
  int getSize() const;
  bool isEmpty() const
  {
    return current_waypoints_.poses.empty();
  };
  double getInterval() const;
  geometry_msgs::Point getWaypointPosition(int waypoint) const;
  geometry_msgs::Quaternion getWaypointOrientation(int waypoint) const;
  geometry_msgs::Pose getWaypointPose(int waypoint) const;
  double getWaypointVelocityMPS(int waypoint) const;
  has_local_planner::Trajectory getCurrentWaypoints() const
  {
    return current_waypoints_;
  }
  bool isFront(int waypoint, geometry_msgs::Pose current_pose) const;
};

// inline function (less than 10 lines )
inline double kmph2mps(double velocity_kmph)
{
  return (velocity_kmph * 1000) / (60 * 60);
}
inline double mps2kmph(double velocity_mps)
{
  return (velocity_mps * 60 * 60) / 1000;
}
inline double deg2rad(double deg)
{
  return deg * M_PI / 180;
}  // convert degree to radian

tf::Vector3 point2vector(geometry_msgs::Point point);  // convert point to vector
geometry_msgs::Point vector2point(tf::Vector3 vector);  // convert vector to point
tf::Vector3 rotateUnitVector(tf::Vector3 unit_vector, double degree);  // rotate unit vector by degree
geometry_msgs::Point rotatePoint(geometry_msgs::Point point, double degree);  // rotate point vector by degree

double DecelerateVelocity(double distance, double prev_velocity);
geometry_msgs::Point calcRelativeCoordinate(geometry_msgs::Point point,
                                            geometry_msgs::Pose current_pose);  // transform point into the coordinate
                                                                                // of current_pose
geometry_msgs::Point calcAbsoluteCoordinate(geometry_msgs::Point point,
                                            geometry_msgs::Pose current_pose);  // transform point into the global
                                                                                // coordinate
double getPlaneDistance(geometry_msgs::Point target1,
                        geometry_msgs::Point target2);  // get 2 dimentional distance between target 1 and target 2
int getClosestWaypoint(const has_local_planner::Trajectory &current_path, geometry_msgs::Pose current_pose);
bool getLinearEquation(geometry_msgs::Point start, geometry_msgs::Point end, double *a, double *b, double *c);
double getDistanceBetweenLineAndPoint(geometry_msgs::Point point, double sa, double b, double c);
double getRelativeAngle(geometry_msgs::Pose waypoint_pose, geometry_msgs::Pose vehicle_pose);
#endif
