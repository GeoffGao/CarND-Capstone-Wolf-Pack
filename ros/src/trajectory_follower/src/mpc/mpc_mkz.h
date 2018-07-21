#ifndef MPC_MKZ_H
#define MPC_MKZ_H
// ROS includes
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <dbw_mkz_twist_controller/ControllerConfig.h>
#include "trajectory_follower/TrajectoryFollowerReconfigureConfig.h"
// math and opmitimization includes
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/LU>  
// user defined includes
#include <cmath>
#include <vector>
#include "autogo_msgs/PathPoint.h"
#include "autogo_msgs/Trajectory.h"
#include "autogo_msgs/TrajectoryPoint.h"
#include "trajectory_follower/trajectory_follower_config.h"
using namespace std;
using CppAD::AD;

namespace trajectory_follower {

extern size_t last_vws_len;

typedef CPPAD_TESTVECTOR(double) Dvector;

typedef boost::recursive_mutex mutex_t;

// TrajectoryFollowerConfig cfg_; //!< Config class that stores and manages all related parameters

class MPC {
 public:

  MPC(const TrajectoryFollowerConfig& cfg);

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  bool Solve(const vector<autogo_msgs::TrajectoryPoint>& current_trajectory,
             const pair<tf::Vector3, tf::Quaternion>& current_pose, const pair<bool, Eigen::Vector3d>& feedback_vel,
             vector<double>& result);

  double Integration(const double& desire, const double& feedback, const double& ki, const double lowerbound,
                     const double upperbound, double& sum);

  void ResetIntegration(double& sum);

  void Saturation(const double lowerbound, const double upperbound, double& sum);

  void Reset();

  Eigen::VectorXd GetPolyfitCoeffs(const std::vector<autogo_msgs::TrajectoryPoint>& local_trajectory, const double constant);

  void FromWorldToPath(const double x, const double y, const double theta,
						 const std::vector<autogo_msgs::TrajectoryPoint>& planning_published_trajectory,
						 std::vector<autogo_msgs::TrajectoryPoint>& local_trajectory);
//  void FromPathToWorld(const std::pair<tf::Vector3, tf::Quaternion>& current_pose,
//                            const std::vector<double>& solution, autogo_msgs::Trajectory* mpcTrajectory)

  autogo_msgs::TrajectoryPoint FromWorldToPath(const double x, const double y, const double theta,
												 const autogo_msgs::TrajectoryPoint point);

  autogo_msgs::TrajectoryPoint GetNearestPointByRelativeTime(
		  const double t, const std::vector<autogo_msgs::TrajectoryPoint>& planning_published_trajectory) const;


  std::vector<autogo_msgs::TrajectoryPoint>& fitted_trajectory() {
    return fitted_trajectory_;
  };
  std::vector<autogo_msgs::TrajectoryPoint>& predicted_trajectory() {
    return predicted_trajectory_;
  };
  std::vector<autogo_msgs::TrajectoryPoint>& fitted_waypoints() {
    return fitted_waypoints_;
  };

 protected:

  mutex_t* access_;

 private:
  // variables

  size_t delay_time = 2;
  size_t latency_interval_vel = 2;
  size_t latency_interval_delta = 2;

  std::vector<autogo_msgs::TrajectoryPoint> current_trajectory_;

  sensor_msgs::Imu current_imu_;

  std::pair<bool, Eigen::Vector2d> feedback_vel;

  std::pair<tf::Vector3, tf::Quaternion> current_pose_;

  const TrajectoryFollowerConfig* cfg_mpc;  //!< Config class that stores and manages all related parameters


  int kappa_number = 10;

  double kappa_threshold = 0.4;

  double kappa_sum_ = 0.0;

  std::vector<autogo_msgs::TrajectoryPoint> fitted_trajectory_;

  std::vector<autogo_msgs::TrajectoryPoint> predicted_trajectory_;

  std::vector<autogo_msgs::TrajectoryPoint> fitted_waypoints_;

  std::vector<double> past_delta;

  std::vector<double> past_vel;

  std::vector<double> past_acc;

  Eigen::VectorXd coeffs_old;

  std::pair<tf::Vector3, tf::Quaternion> localization_old;

  double mean_max = 0.0;

  double var_cor = 0.0;

  double lookingforward_distance = 25;

  double cte_max = 0.0;

  double area_ = 0.0;

  int num_of_next_waypoint_ = 0;

  Eigen::VectorXd initState(const std::vector<geometry_msgs::Pose>& current_trajectory);

  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, double vel_ref);


  bool SolveFg(const Eigen::VectorXd& state, const Eigen::VectorXd& coeffs, const double vel_ref, const double kappa,
               std::vector<double>& result);

  bool SolveFg(const Eigen::VectorXd& state, const Eigen::VectorXd& coeffs, const double vel_ref, const double kappa,
               std::vector<double>& past_delta, std::vector<double>& past_vel, std::vector<double>& result);
  //nonlinear mpc
  bool SolveFgNL(const autogo_msgs::TrajectoryPoint& pose, const std::vector<autogo_msgs::TrajectoryPoint>& traj,
		         Eigen::VectorXd lastAct, std::vector<double>& result);
  /**
   * @brief  Acording to the motion equations and limits, use the nonlinear opmitimization tools to figure out the
   * control input <b>Note:  whether the trajectory is in the odom frame or map frame is not decided! <b>
   * @param  state  The state variable including e.g. x, y, theta,
   * @param  coeffs  The polyfit coefficients, denoting the trajectory in the frame of odom or map
   * @param  vel_ref  The reference linear velocity input
   * @param  last_ws  The reference angular velocity input
   */
  void ParamConfig();

  void visTrajGet();

  double  getPlaneDistance(geometry_msgs::Point target1, geometry_msgs::Point target2);

  tf::Vector3 point2vector(geometry_msgs::Point point);

  void GetTrajectoryForControl(const double& x, const double& y, const double& theta,
                               const std::vector<autogo_msgs::TrajectoryPoint>& planning_published_trajectory,
                               std::vector<autogo_msgs::TrajectoryPoint>& local_trajectory);

    /**
     * @brief  To polyfit the input trajectory and figure out the coefficients
     * <b>Note:  whether the trajectory is in the odom frame or map frame is not decided! <b>
     * @param  xvals  The x coordinate
     * @param  yvals  The y coordinate
     * @param  order  The order to polyfit
     */
    Eigen::VectorXd polyfit(std::vector<double> xvals, std::vector<double> yvals, const size_t order, const double constant) const;

  /**
   * @brief  Evaluate a polynomial
   * <b>Note:  whether the trajectory is in the odom frame or map frame is not decided! <b>
   * @param  coeffs  The polynomial coefficients
   * @param  x  The x coordinate in the robot frame
   */
  double polyeval(Eigen::VectorXd coeffs, double x) const;

  /**
   * @brief  Acording to the motion equations and limits, use the nonlinear opmitimization tools to figure out the
   * control input <b>Note:  whether the trajectory is in the odom frame or map frame is not decided! <b>
   * @param  state  The state variable including e.g. x, y, theta,
   * @param  coeffs  The polyfit coefficients, denoting the trajectory in the frame of odom or map
   * @param  vel_ref  The reference linear velocity input
   * @param  last_ws  The prediction angular velocity input, to filled with the time delay
   */

  double sqrtVelocity(double vx, double vy) {
    return sqrt(pow(vx, 2) + pow(vy, 2));
  }

  double GetAvgKappa(const vector<autogo_msgs::TrajectoryPoint>& traj);

  void coeffsAlignment(Eigen::VectorXd& coeffs, const Eigen::VectorXd waypoints_x_, const Eigen::VectorXd waypoints_y_,
                       const int order);

  bool isFront(const autogo_msgs::TrajectoryPoint& point, const std::pair<tf::Vector3, tf::Quaternion>& current_pose_);

 public:

  /*TODO: !!!current not being used=================================*/
  double GetTrajectoryKappa(const std::vector<autogo_msgs::TrajectoryPoint>& local_trajectory);
  /*----------------以下函数是在跟踪路点模式下的kappa计算模块--------------------------*/
  double GetCurvature(const autogo_msgs::TrajectoryPoint& local_trajectory);

  void FixKappa(const Eigen::VectorXd& coeffs, std::vector<autogo_msgs::TrajectoryPoint>& local_trajectory);
  // distance between target 1 and target2 in 2-D
  double GetPlaneDistance(tf::Vector3 v1, tf::Vector3 v2);

  /*-------------------------------------------------------------------------*/

  geometry_msgs::PoseArray waypointsFromRobotToWorld(const std::pair<tf::Vector3, tf::Quaternion>& current_pose,
                                                       const vector<double>& solution);

  void trajectoryFromRobotToWorld(const std::pair<tf::Vector3, tf::Quaternion>& current_pose,
                                    const vector<double>& solution, autogo_msgs::Trajectory* mpcTrajectory);



  void trajectoryFromRobotToWorld(const std::pair<tf::Vector3, tf::Quaternion>& current_pose,
                                  const vector<std::pair<double, double>>& fit_traj_,
                                  autogo_msgs::Trajectory* mpcTrajectory);

  void trajectoryFromRobotToWorld(const std::pair<tf::Vector3, tf::Quaternion>& current_pose,
                                  const std::vector<autogo_msgs::TrajectoryPoint>& fit_traj_,
                                  std::vector<autogo_msgs::TrajectoryPoint>* mpcTrajectory);

  double varianceCalc(std::vector<double>& buffer) const;

  autogo_msgs::TrajectoryPoint GetNearestPointByPosition(const double x,
		                                                 const double y,
		                                                 const std::vector<autogo_msgs::TrajectoryPoint>& planning_published_trajectory);


  double SquareEuclideanDistance(const double x, const double y, const autogo_msgs::TrajectoryPoint& point);
  //NMPC component
 public:
  Dvector solution;
  Eigen::VectorXd lastAct;
 private:
  bool hasSolution;
  void TrajExtraction(std::vector<autogo_msgs::TrajectoryPoint>& extract_traj_, const int& mode, const double& deltaT);
};

typedef boost::shared_ptr<MPC> MPCPtr;

const int NUMBER_OF_STATES = 5; 	// px, py, psi, v
const int NUMBER_OF_ACTUATIONS = 2; // steering angle, acceleration

struct NMPCParam {
	double W_dist_lateral;
	double W_dist_vertical;
	double W_epsi;
	double W_delta_acc;
	double W_delta_steer;
	double W_speed;
	double W_steer;
	double num_advanced;
	double acc_argument;
};


}  // namespace trajectory_follower

#endif /* MPC_MKZ_H */
