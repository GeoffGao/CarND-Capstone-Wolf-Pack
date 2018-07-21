/********************************************************
*   Copyright (C) 2018 All rights reserved.
*
*   Filename: mpc_components.cpp
*   Author  : junlong.gao
*   Date    : May 24, 2018
*   Describe: function components in MPC
*
********************************************************/
#include "mpc_mkz.h"

using CppAD::AD;
using CppAD::Var2Par;

#define DEBUG

namespace trajectory_follower {

void MPC::GetTrajectoryForControl(const double& x, const double& y, const double& theta,
                                  const std::vector<autogo_msgs::TrajectoryPoint>& planning_published_trajectory,
                                  std::vector<autogo_msgs::TrajectoryPoint>& local_trajectory) {

  std::vector<autogo_msgs::TrajectoryPoint> extract_traj_;
  geometry_msgs::Point ego_pos,zero_pos;
  ego_pos.x = x; ego_pos.y = y; ego_pos.z = 0.0;
  zero_pos.x = 0.0; zero_pos.y = 0.0; zero_pos.z = 0.0;
  FromWorldToPath(x, y, theta, planning_published_trajectory, extract_traj_);
  int path_size = extract_traj_.size();
  // if waypoints are not given, do nothing.
   if (path_size == 0)
   {
     num_of_next_waypoint_ = -1;
     local_trajectory = extract_traj_;
     return;
   }
   // look for the next waypoint.

   for (int i = 0; i < path_size; i++)
   {
	   if(isnan(extract_traj_[i].path_point.point.x))
	   {
			ROS_ERROR("search waypoint wrong");
			num_of_next_waypoint_ = -1;
			return;
	   }
	   local_trajectory.push_back(extract_traj_[i]);
#ifdef DEBUG1
	   cout<<"extracted Pt["<<i<<"].x="<<extract_traj_[i].path_point.point.x<<", .y="<<extract_traj_[i].path_point.point.y<<", .v="<<extract_traj_[i].v<<endl;
#endif
	   // if search waypoint is the last
	   if (i == (path_size - 1))
	   {
		 ROS_INFO("search waypoint is the last");
		 num_of_next_waypoint_ = i;
		 return;
	   }
	   // if there exists an effective waypoint
	   double dis = getPlaneDistance(extract_traj_[i].path_point.point, zero_pos);
#ifdef DEBUG1
	   cout<<"dis["<<i<<"]="<<dis<<", lookingforward_dis="<<lookingforward_distance<<endl;
#endif
	   if (getPlaneDistance(extract_traj_[i].path_point.point, zero_pos) > lookingforward_distance)
	   {
		 num_of_next_waypoint_ = i;
		 return;
	   }
	}

   // if this program reaches here , it means we lost the waypoint!
     num_of_next_waypoint_ = -1;
     return;
}

void MPC::TrajExtraction(std::vector<autogo_msgs::TrajectoryPoint>& extract_traj_, const int& mode, const double& deltaT)
{
   std::vector<autogo_msgs::TrajectoryPoint> temp;
   int count = 0;
   if(mode ==0)
   {
	   for (int i = 0; i< extract_traj_.size();)
	   {
		   temp.push_back(extract_traj_[i]);
		   i = i + 1;
		   count = count + 1;
	   }
   }else if(mode ==1){
	   double time = 0.0;
	   int count = 1;
	   for (int i = 0; i < extract_traj_.size();i++)
	   {
		   if (i == 0){
			   time = extract_traj_[i].relative_time;
			   temp.push_back(extract_traj_[i]);
			   count = count + 1;
		   }else{
			   if (fabs(extract_traj_[i].relative_time - time)/deltaT > 0.9 &&
				   fabs(extract_traj_[i].relative_time - time)/deltaT < 1.1)
			   {
				   time = extract_traj_[i].relative_time;
				   temp.push_back(extract_traj_[i]);
				   count = count + 1;
			   }
		   }

	   }

   }
   //new update
   num_of_next_waypoint_ = count - 1;
   extract_traj_.clear();
   extract_traj_.resize(temp.size());
   extract_traj_ = temp;

}


// distance between target 1 and target2 in 2-D
double  MPC::getPlaneDistance(geometry_msgs::Point target1, geometry_msgs::Point target2)
{
  tf::Vector3 v1 = point2vector(target1);
  v1.setZ(0);
  tf::Vector3 v2 = point2vector(target2);
  v2.setZ(0);
  return tf::tfDistance(v1, v2);
}


tf::Vector3 MPC::point2vector(geometry_msgs::Point point)
{
  tf::Vector3 vector(point.x, point.y, point.z);
  return vector;
}



void MPC::FromWorldToPath(const double x, const double y, const double theta,
                          const std::vector<autogo_msgs::TrajectoryPoint>& planning_published_trajectory,
                          std::vector<autogo_msgs::TrajectoryPoint>& local_trajectory) {
  autogo_msgs::TrajectoryPoint pose_;
  double dx = 0.0;
  double dy = 0.0;
//  local_trajectory.emplace_back(pose_);
  for (size_t i = 0; i < planning_published_trajectory.size(); i++) {
    dx = planning_published_trajectory[i].path_point.point.x - x;
    dy = planning_published_trajectory[i].path_point.point.y - y;
    pose_ = planning_published_trajectory[i];
    pose_.path_point.point.x =  cos(theta) * dx + sin(theta) * dy;
    pose_.path_point.point.y = -sin(theta) * dx + cos(theta) * dy;

#ifdef DEBUG1
    ROS_WARN("ori_traj[%d].x=%f,dx=%f, ori_traj.y=%f, dy=%f, pose_.x=%f,pose_.y=%f, pose_.v=%f",
    		 i,planning_published_trajectory[i].path_point.point.x, dx,planning_published_trajectory[i].path_point.point.y, dy,
			 pose_.path_point.point.x, pose_.path_point.point.y, pose_.v);
#endif
    local_trajectory.emplace_back(pose_);
  }
}

//void MPC::FromPathToWorld(const std::pair<tf::Vector3, tf::Quaternion>& current_pose,
//                          const std::vector<double>& solution, autogo_msgs::Trajectory* mpcTrajectory) {
//  double theta = tf::getYaw(current_pose.second) + cfg_mpc->configMPC.heading_compensation;
//  int sequence_size = (solution.size() - 2) / 5;
//  for (int ri = 0; ri < sequence_size; ++ri) {
//    autogo_msgs::TrajectoryPoint mpcPose;
//    mpcPose.path_point.point.x = cos(theta) * solution[ri * 5 + 2] - sin(theta) * solution[ri * 5 + 3];
//    mpcPose.path_point.point.y = sin(theta) * solution[ri * 5 + 2] + cos(theta) * solution[ri * 5 + 3];
//    mpcPose.path_point.point.z = 0;
//
//    mpcPose.path_point.point.x += current_pose.first.getX();
//    mpcPose.path_point.point.y += current_pose.first.getY();
//    mpcTrajectory->trajectory_points.push_back(mpcPose);
//  }
//  return;
//}

Eigen::VectorXd MPC::GetPolyfitCoeffs(const std::vector<autogo_msgs::TrajectoryPoint>& local_trajectory,
                                      const double constant) {
  size_t order = 3;
  size_t fit_len = local_trajectory.size();
  Eigen::VectorXd coeffs = Eigen::VectorXd::Zero(order + 1);
  std::vector<double> xs;
  std::vector<double> ys;
  for (auto& iter : local_trajectory) {
    xs.emplace_back(iter.path_point.point.x);
    ys.emplace_back(iter.path_point.point.y);
  }
  if (fit_len < 2)
    ROS_WARN("TrajectoryAnalyzer: trajectory has only ONE point!");
  else {
    order = std::min(order, fit_len - 1);
    Eigen::VectorXd tmp = polyfit(xs, ys, order, constant);
    for (int i = 0; i < tmp.size(); i++) coeffs[i] = tmp[i];
  }
#ifdef DEBUG
  ROS_INFO("y = %.4f + %.4f * x + %.4f * x**2 + %.4f * x**3", coeffs[0], coeffs[1], coeffs[2], coeffs[3]);
#endif
  return coeffs;
}

Eigen::VectorXd MPC::polyfit(std::vector<double> xvals, std::vector<double> yvals, const size_t order,
                             const double constant) const {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (size_t i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (size_t j = 0; j < xvals.size(); j++) {
    for (size_t i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals[j];
    }
  }
  Eigen::VectorXd y = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(yvals.data(), yvals.size());
  auto result =
      (A.transpose() * A + constant * Eigen::MatrixXd::Identity(A.cols(), A.cols())).inverse() * A.transpose() * y;
  return result;
}

double MPC::GetAvgKappa(const std::vector<autogo_msgs::TrajectoryPoint>& traj) {
  double kappa_sum = 0.0;
  if (traj.size() == 0) {
    return kappa_sum;
  } else if (traj.size() < cfg_mpc->configMPC.kappa_number) {
    for (size_t i = 0; i < traj.size(); ++i) {
      kappa_sum += traj[i].path_point.kappa;
    }
    return kappa_sum / traj.size() * cfg_mpc->configMPC.kappa_number;
  } else {
    for (int i = 0; i < cfg_mpc->configMPC.kappa_number; ++i) {
      kappa_sum += traj[i].path_point.kappa;
    }
    return kappa_sum;
  }
  ROS_WARN("kappa_sum : %.4f", kappa_sum);
}

/*----------------------在路点跟踪模式下使用-------------------------*/
double MPC::GetCurvature(const autogo_msgs::TrajectoryPoint& target)
{
  double kappa;
  double KAPPA_MIN_ = 1 / 9e10;
  tf::Vector3 temp1, temp2;
  temp1.setValue(target.path_point.point.x, target.path_point.point.y,target.path_point.point.z);
  temp2 = current_pose_.first;
#ifdef DEBUG1
  ROS_INFO("inGetCurvature CP1.1: relativePt.x=%f, .y=%f, .z=%f",target.path_point.point.x,target.path_point.point.y,target.path_point.point.z);
  ROS_INFO("inGetCurvature CP1.2: currentPos.x=%f, .y=%f, .z=%f",current_pose_.first.getX(),current_pose_.first.getY(),current_pose_.first.getZ());
#endif
  double denominator = pow(GetPlaneDistance(temp1, temp2), 2);
  double numerator = 2 * (target.path_point.point.y - current_pose_.first.getY());

  if (denominator != 0)
    kappa = numerator / denominator;
  else
  {
    if(numerator > 0)
     kappa = KAPPA_MIN_;
    else
      kappa = -KAPPA_MIN_;
  }
#ifdef DEBUG1
  ROS_INFO_STREAM("kappa :" << kappa);
#endif
  return kappa;
}

double MPC::GetPlaneDistance(tf::Vector3 v1, tf::Vector3 v2)
{
  v1.setZ(0);
  v2.setZ(0);
  return tf::tfDistance(v1, v2);
}

/*----------------------------------------------------------*/
autogo_msgs::TrajectoryPoint MPC::FromWorldToPath(const double x, const double y, const double theta,
                                                  const autogo_msgs::TrajectoryPoint point) {
  autogo_msgs::TrajectoryPoint pose;
  double dx = point.path_point.point.x - x;
  double dy = point.path_point.point.y - y;
  pose = point;
  pose.path_point.point.x = cos(theta) * dx + sin(theta) * dy;
  pose.path_point.point.y = -sin(theta) * dx + cos(theta) * dy;
  return pose;
}

autogo_msgs::TrajectoryPoint MPC::GetNearestPointByRelativeTime(
    const double t, const std::vector<autogo_msgs::TrajectoryPoint>& planning_published_trajectory) const {
  auto func_comp = [](const autogo_msgs::TrajectoryPoint& point, const double relative_time) {
    return point.relative_time < relative_time;
  };

  auto it_low =
      std::lower_bound(planning_published_trajectory.begin(), planning_published_trajectory.end(), t, func_comp);

  if (it_low == planning_published_trajectory.begin()) {
    return planning_published_trajectory.front();
  }

  if (it_low == planning_published_trajectory.end()) {
    return planning_published_trajectory.back();
  }

  auto it_lower = it_low - 1;
  if (it_low->relative_time - t < t - it_lower->relative_time) {
    return *it_low;
  }
  return *it_lower;
}

double MPC::polyeval(Eigen::VectorXd coeffs, double x) const {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}
/*TODO: GetTrajectoryKappa is not using currently*/
double MPC::GetTrajectoryKappa(const std::vector<autogo_msgs::TrajectoryPoint>& local_trajectory) {
  double kappa = 0.0;
  size_t traj_size = local_trajectory.size();
  if (traj_size == 0) return kappa;

  size_t num = std::min(traj_size, size_t(cfg_mpc->configMPC.kappa_number));
  for (size_t i = 0; i < num; i++) {
    kappa += local_trajectory[i].path_point.kappa;
  }
  kappa *= double(cfg_mpc->configMPC.kappa_number) / double(num);
  return kappa;
}

void MPC::FixKappa(const Eigen::VectorXd& coeffs, std::vector<autogo_msgs::TrajectoryPoint>& local_trajectory) {
  double x = 0.0;
  double df = 0.0;
  double ddf = 0.0;
  size_t num = std::min(local_trajectory.size(), size_t(cfg_mpc->configMPC.kappa_number));
  for (size_t i = 0; i < num; i++) {
    x = local_trajectory[i].path_point.point.x;
    df = coeffs[1] + coeffs[2] * x * 2 + coeffs[3] * x * x * 3;
    ddf = coeffs[2] * 2 + coeffs[3] * x * 6;
    local_trajectory[i].path_point.kappa = std::fabs(ddf) / std::pow(1 + std::pow(df, 2), 1.5);
  }
}

geometry_msgs::PoseArray MPC::waypointsFromRobotToWorld(const std::pair<tf::Vector3, tf::Quaternion>& current_pose,
                                                        const vector<double>& solution) {
  geometry_msgs::PoseArray mpcSequence;
  double theta = tf::getYaw(current_pose.second);
  int sequence_size = (solution.size() - 2) / 5;
  for (int ri = 0; ri < sequence_size; ++ri) {
    geometry_msgs::Pose mpcPose;
    mpcPose.position.x = cos(theta) * solution[ri * 5 + 2] - sin(theta) * solution[ri * 5 + 3];
    mpcPose.position.y = sin(theta) * solution[ri * 5 + 2] + cos(theta) * solution[ri * 5 + 3];
    mpcPose.position.z = 0;

    mpcPose.position.x += current_pose.first.getX();
    mpcPose.position.y += current_pose.first.getY();
    mpcSequence.poses.push_back(mpcPose);
  }

  return mpcSequence;
}
void MPC::trajectoryFromRobotToWorld(const std::pair<tf::Vector3, tf::Quaternion>& current_pose,
                                     const vector<double>& solution, autogo_msgs::Trajectory* mpcTrajectory) {
  double theta = tf::getYaw(current_pose.second);
  int sequence_size = (solution.size() - 2) / 5;
  for (int ri = 0; ri < sequence_size; ++ri) {
    autogo_msgs::TrajectoryPoint mpcPose;
    mpcPose.path_point.point.x = cos(theta) * solution[ri * 5 + 2] - sin(theta) * solution[ri * 5 + 3];
    mpcPose.path_point.point.y = sin(theta) * solution[ri * 5 + 2] + cos(theta) * solution[ri * 5 + 3];
    mpcPose.path_point.point.z = 0;

    mpcPose.path_point.point.x += current_pose.first.getX();
    mpcPose.path_point.point.y += current_pose.first.getY();
    mpcTrajectory->trajectory_points.push_back(mpcPose);
  }
  return;
}

void MPC::trajectoryFromRobotToWorld(const std::pair<tf::Vector3, tf::Quaternion>& current_pose,
                                     const vector<std::pair<double, double>>& fit_traj_,
                                     autogo_msgs::Trajectory* mpcTrajectory) {
  double theta = tf::getYaw(current_pose.second);
  // double max = 0.0;
  for (size_t i = 0; i < fit_traj_.size(); ++i) {
    autogo_msgs::TrajectoryPoint mpcPose;
    mpcPose.path_point.point.x = cos(theta) * fit_traj_[i].first - sin(theta) * fit_traj_[i].second;
    mpcPose.path_point.point.y = sin(theta) * fit_traj_[i].first + cos(theta) * fit_traj_[i].second;
    mpcPose.path_point.point.z = 0;

    mpcPose.path_point.point.x += current_pose.first.getX();
    mpcPose.path_point.point.y += current_pose.first.getY();
    mpcTrajectory->trajectory_points.push_back(mpcPose);
  }
}

void MPC::trajectoryFromRobotToWorld(const std::pair<tf::Vector3, tf::Quaternion>& current_pose,
                                     const std::vector<autogo_msgs::TrajectoryPoint>& fit_traj_,
                                     std::vector<autogo_msgs::TrajectoryPoint>* mpcTrajectory) {
  double theta = tf::getYaw(current_pose.second);
  // double max = 0.0;
  for (size_t i = 0; i < fit_traj_.size(); ++i) {
    autogo_msgs::TrajectoryPoint mpcPose;
    double x = fit_traj_[i].path_point.point.x;
    double y = fit_traj_[i].path_point.point.y;
    mpcPose.path_point.point.x = cos(theta) * x - sin(theta) * y;
    mpcPose.path_point.point.y = sin(theta) * x + cos(theta) * y;
    mpcPose.path_point.point.z = 0;

    mpcPose.path_point.point.x += current_pose.first.getX();
    mpcPose.path_point.point.y += current_pose.first.getY();
    mpcTrajectory->push_back(mpcPose);
  }
}

double MPC::varianceCalc(std::vector<double>& buffer) const {
  double sum = std::accumulate(buffer.begin(), buffer.end(), 0.0);
  double mean = sum / buffer.size();
  double accum = 0.0;
  std::vector<double>::iterator iter = buffer.begin();
  for (; iter != buffer.end(); iter++) {
    accum += (*iter - mean) * (*iter + mean);
  }
  return sqrt(accum / buffer.size());
}

}  // namespace trajectory_follower
