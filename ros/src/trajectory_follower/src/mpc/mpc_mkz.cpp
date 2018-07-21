/********************************************************
*   Copyright (C) 2018 All rights reserved.
*
*   Filename: mpc_mkz.cpp
*   Author  : junlong.gao
*   Date    : May 24, 2018
*   Describe: nonlinear MPC controller for MKZ
*
********************************************************/
#include <ros/ros.h>
#include "mpc_mkz.h"
using CppAD::AD;
// using CppAD::Value;
using CppAD::Var2Par;

#define POINTFOLLOW
#define DEBUG
const string METHOD="NMPC";
//const string METHOD="MPC";
namespace trajectory_follower {

double dt = 0.1;  //预测时间间隔
size_t N = 10;    //和预瞄长度算在一起了，作为具有上下界的可变量[10,xxx]

/************************NMPC Module****************************************/
class FG_eval {
private:
	double wheel_base_;
	std::vector<autogo_msgs::TrajectoryPoint> _traj;
	Eigen::VectorXd _lastAct;
	const TrajectoryFollowerConfig* cfg;
	NMPCParam _param;
public:

	autogo_msgs::TrajectoryPoint _initPose;

    //TODO：输入初始位置、跟踪轨迹、最后行为、MPC参数
	FG_eval(const autogo_msgs::TrajectoryPoint& initPose, const std::vector<autogo_msgs::TrajectoryPoint>& traj,
			Eigen::VectorXd lastAct, const TrajectoryFollowerConfig* cfg) {
		_initPose = initPose;
		_traj = traj;
		this->cfg = cfg;
		ros::param::get("~mpc/wheel_base",      this->wheel_base_);
		ros::param::get("~mpc/weight_lateral",  _param.W_dist_lateral);
		ros::param::get("~mpc/weight_vertical", _param.W_dist_vertical);
		ros::param::get("~mpc/weight_epsi",     _param.W_epsi);
		ros::param::get("~mpc/weight_delta_acc",_param.W_delta_acc);
		ros::param::get("~mpc/weight_delta_steer", _param.W_delta_steer);
		ros::param::get("~mpc/weight_speed",    _param.W_speed);
		ros::param::get("~mpc/weight_steer",    _param.W_steer);
		_lastAct = lastAct;
	}
	//TODO:输入planning point信息 给cppad::vector<AD<double>>
	void TrajectoryPoint2Dvector(const autogo_msgs::TrajectoryPoint point, CppAD::vector< AD<double> > & vector) {
		vector[0] = point.path_point.point.x;//x
		vector[1] = point.path_point.point.y;//y
		vector[2] = point.path_point.theta;  //朝向
		vector[3] = point.v + 0.001;         //trick for solving the problem of solu 11 and 9
		vector[4] = point.path_point.kappa;  //曲率
	}

	typedef CPPAD_TESTVECTOR( AD<double> ) ADvector;
	void operator()(ADvector& fg, const ADvector& x) {
		CppAD::vector< AD<double>  > act(NUMBER_OF_ACTUATIONS);//steer_angle && acceleration
		AD<double> PI = 3.1415926;
		AD<double> last_lr_cos = 0;
		AD<double> flag = 0;
		AD<double> beta = 0;
		/*Number_of_states includes 5 states in NMPC: pre-definition is px, py, psi, v
		* xt is current state
		* xt1 is next step predicted state
		* xt1_des is next step reference command in trajectory
		*/
		CppAD::vector< AD<double>  > xt(NUMBER_OF_STATES), xt1(NUMBER_OF_STATES), xt1_des(NUMBER_OF_STATES);
		TrajectoryPoint2Dvector(_initPose, xt);
        /*xt[0~4]:x,y,theta,speed,steer*/
		act[0] = xt[4];//steer angle
		act[1] = _lastAct[4];//acceleration, _lastAct is what is to be determined
		fg[0] = 0;
		ROS_ERROR("in MPC trajsize=%d",_traj.size());
		/* NMPC part 1*/
			for(int i = 0; i < N; ++i) {
                /*-----------------predicted model--------------------*/
				beta = CppAD::atan( cfg->configMPC.LF/ 2 * CppAD::tan(x[i]) / cfg->configMPC.LF);
				xt1[0] = xt[0] + xt[3] * CppAD::cos(xt[2] + beta) * dt;
				xt1[1] = xt[1] + xt[3] * CppAD::sin(xt[2] + beta) * dt;
				xt1[2] = xt[2] + xt[3] * CppAD::cos(beta) * CppAD::tan(x[i]) * dt / cfg->configMPC.LF; //moded
				xt1[3] = xt[3] + x[i+ N] * dt;
                /*-----------------destination info------------------*/
				xt1_des[0] = _traj[i].path_point.point.x;
				xt1_des[1] = _traj[i].path_point.point.y;
				xt1_des[2] = _traj[i].path_point.theta;
				xt1_des[3] = _traj[i].v;
                /*---------------calc error psi(theta)--------------*/
				AD<double> epsi = (xt1[2] - xt1_des[2]);
				if(epsi > PI) epsi -= 2*PI;
				if(epsi < -PI) epsi += 2*PI;
                /*-----------calc cost function infactors----------*/
				/*   dist    是下一步轨迹command位置与下一步预测位置之间的距离
				 *   distcar 是下一步预测位置与本车当前位置之间的距离
				 *   _cos    是预测轨迹与本车位置向量与轨迹command与预测轨迹位置向量的夹角cos值
				 *   _lr_cos 是t+2轨迹点与t+1轨迹点向量 与 t+1预测位置与t+1轨迹点向量构成的向量平行四边形面积
				 *   dist_lateral （t+1轨迹点与t+1预测点向量的横向距离）
				 *   dist_vertical（t+1轨迹点与t+1预测点向量的纵向距离）
				 */
				AD<double> dist = CppAD::sqrt((xt1[0] - xt1_des[0]) * (xt1[0] - xt1_des[0]) + (xt1[1] - xt1_des[1]) * (xt1[1] - xt1_des[1]));
				AD<double> distcar = CppAD::sqrt((xt1[0] - xt[0]) * (xt1[0] - xt[0]) + (xt1[1] - xt[1]) * (xt1[1] - xt[1]));
				AD<double> _cos = ((xt1[0] - xt1_des[0]) * (xt1[0] - xt[0]) + (xt1[1] - xt1_des[1]) * (xt1[1] - xt[1])) / (dist * distcar + 1e-8);
				AD<double> _lr_cos = (_traj[i+1].path_point.point.x - _traj[i].path_point.point.x) * (xt1[1] - _traj[i].path_point.point.y)
									- (_traj[i+1].path_point.point.y - _traj[i].path_point.point.y) * (xt1[0] - _traj[i].path_point.point.x);
				AD<double> dist_lateral = dist * CppAD::sqrt(1 - _cos * _cos);
				AD<double> dist_vertical = dist * _cos;
                /*--------横向误差存在控制死区定位错误时的处理------*/
				if (last_lr_cos * _lr_cos < 0) flag = 1;
				if (flag == 1) dist_lateral = 5 * dist_lateral;
                /*------cost function 构建 step1 纵向误差^2+横向误差^2+角度朝向误差^2+steer^2+速度误差^2*/
				fg[0] += _param.W_dist_lateral * dist_lateral * dist_lateral
						+ _param.W_dist_vertical * dist_vertical * dist_vertical
						+ _param.W_epsi * epsi * epsi
						+ _param.W_steer * x[i] * x[i]
						+ _param.W_speed * ((xt1[3] - xt1_des[3]) * (xt1[3] - xt1_des[3]));
#ifdef DEBUG1
				cout<<"fg["<<i<<"], p1="<<_param.W_dist_lateral * dist_lateral * dist_lateral<<",p2="<<_param.W_dist_vertical * dist_vertical * dist_vertical
				    <<",p3="<<_param.W_epsi * epsi * epsi<<",p4="<<_param.W_steer * x[i] * x[i]<<",p5="<<_param.W_speed * ((xt1[3] - xt1_des[3]) * (xt1[3] - xt1_des[3]))<<endl;
				cout<<"fg+="<<fg[0]<<endl;
#endif
				/*-----cost function 构建 step2 加入steer与速度的限制条件-----*/
				fg[0] += _param.W_delta_steer * (x[i] - act[0]) * (x[i] - act[0])
						+ _param.W_delta_acc * (x[i + N] - act[1]) * (x[i + N] - act[1]);
#ifdef DEBUG1
				cout<<"fg["<<i<<"], p6="<<_param.W_delta_steer * (x[i] - act[0]) * (x[i] - act[0])<<",p7="<<_param.W_delta_acc * (x[i + N] - act[1]) * (x[i + N] - act[1])<<endl;
				cout<<"fg+="<<fg[0]<<endl;
#endif
				/*-----update the next step info----*/
				xt = xt1;
				act[0] = x[i];
				act[1] = x[i + N];
				last_lr_cos = _lr_cos;
			}
		    fg[1] = x[0] - xt[4];//steer - steer'
		    cout<<"fg[0]="<<fg[0]<<",fg[1]="<<fg[1]<<endl;
		 /* NMPC part 2 */
			for(int i = 2; i <= N; ++i) {
				fg[i] = x[i-1] - x[i-2];
			}
		return;
		}
};

bool MPC::SolveFgNL(const autogo_msgs::TrajectoryPoint& ego_pose, const std::vector<autogo_msgs::TrajectoryPoint>& traj, Eigen::VectorXd lastAct,  std::vector<double>& result) {
	bool ok = false;
	Dvector xi(NUMBER_OF_ACTUATIONS * N);
	if(!hasSolution)
	for(int i = 0; i < xi.size(); ++i) {
		xi[i] = 0;
	}
	else xi = solution;
	/*----------xl, xu是上下界, steer_angle & acceleration----------*/
	Dvector xl(NUMBER_OF_ACTUATIONS * N), xu(NUMBER_OF_ACTUATIONS * N);
	for(int i = 0; i < N; ++i) {
		xl[i] = -0.44;
		xu[i] = 0.44;
		xl[i + N] = -2.5;
		xu[i + N] = 1.5;
	}
    /*----------gl,gu是什么的上下界？steer_rate constrain-----------------*/
	Dvector gl(N), gu(N);
	for(int i = 0; i < N; ++i) {
		gl[i] = -1.88/10;
		gu[i] = 1.88/10;
	}

	// object that computes objective and constraints
	FG_eval fg_eval(ego_pose, traj, lastAct, cfg_mpc);

	std::string options;
	options += "Integer print_level  0\n"; //12 max show all messages
	options += "String  sb           yes\n";
	options += "Integer max_iter     100000000\n";
	options += "Numeric max_cpu_time 0.5\n";
	options += "Numeric tol          1e-1\n";
	options += "String  derivative_test            second-order\n";
	options += "Numeric point_perturbation_radius  0.\n";
	options += "String print_timing_statistics yes\n";
    // place to return solution
	CppAD::ipopt::solve_result<Dvector> solu;
	clock_t start = clock();
	// solve the problem
	CppAD::ipopt::solve<Dvector, FG_eval>(
		 options, xi, xl, xu, gl, gu, fg_eval, solu
	);
	clock_t end = clock();
    ok = solu.status == CppAD::ipopt::solve_result<Dvector>::success;
	#ifdef DEBUG
    //ROS_INFO("case solved?=%d", ok);
    std::cout<<"time = "<< (double (end - start)) / double (CLOCKS_PER_SEC)<<std::endl;
	//ROS_WARN("SolveFg CP4 after ippod solve");
	#endif

	this->solution = solu.x;

	result.clear();
	result.resize(2);
    result[0]=this->solution[latency_interval_delta - 1];
    result[1]=this->solution[latency_interval_vel -1 + N];
	//-------------------------------
	hasSolution = true;
	return ok;
}
/***************************************************************************/
/*************************MPC module****************************************/
/*---------------------solver-----------------------------*/
/*----------用一个长串序列来做参数定义，8个参数-----------
 *---命名方式后缀"_"和""分别代表：无延迟的变量 & 带延迟的变量
 * */
/* x_start：    起始位置x
 * y_start：    起始位置y
 * psi_start：  起始朝向psi（theta）
 * cte_start：  起始cross tracking error
 * epsi_start： 起始psi error
 * area_start： 起始横向误差积分
 * v_start：    起始速度
 * delta_start：起始轮胎转角
 * a_start:     起始线加速度
 * */
size_t x_start_ = 0;
size_t y_start_ = x_start_ + N;
size_t psi_start_ = y_start_ + N;
size_t cte_start_ = psi_start_ + N;
size_t epsi_start_ = cte_start_ + N;
size_t area_start_ = epsi_start_ + N;
size_t v_start_ = area_start_ + N;
size_t delta_start_ = v_start_ + N;
size_t a_start_ = delta_start_ + N - 1;

//
// MPC class definition implementation.
//
MPC::MPC(const TrajectoryFollowerConfig& cfg) : access_(new mutex_t()), cfg_mpc(&cfg) {
  current_pose_.first.setZero();
  current_pose_.second.normalize();
  localization_old.first.setZero();
  localization_old.second.normalize();
  coeffs_old = Eigen::VectorXd::Zero(4);
  past_delta.assign(latency_interval_delta, 0.0);
  past_vel.assign(latency_interval_vel, 0.0);
  past_acc.assign(latency_interval_vel, 0.0);
  lastAct = Eigen::VectorXd::Zero(8);
  hasSolution = false;
  //ParamConfig();
}

MPC::~MPC() {
  delete access_;
}

void MPC::Reset() {
  area_ = 0.0;
}

void MPC::ParamConfig() {
  // ros::param::get("~mpc/is_check_accuracy", is_check_accuracy);
}

bool MPC::Solve(const std::vector<autogo_msgs::TrajectoryPoint>& planning_published_trajectory,
                const pair<tf::Vector3, tf::Quaternion>& localization, const pair<bool, Eigen::Vector3d>& feedback_vel,
                vector<double>& result) {
  /*--------------------get local traj based on global planning traj and ego car position--------------------*/
  double kappa = 0.0;
#ifdef POINTFOLLOW
  double kappa_pre = 0.0;
#else
  double kappa_pre = GetTrajectoryKappa(planning_published_trajectory);
#endif
  double x = localization.first.getX();
  double y = localization.first.getY();
  double theta = tf::getYaw(localization.second) + cfg_mpc->configMPC.heading_compensation;
#ifdef DEBUG1
  ROS_INFO("solve CP1.1: heading_compensation: %.4f", cfg_mpc->configMPC.heading_compensation);
  ROS_INFO("solve CP1.2: locolization.x:%.4f, .y: %.4f, .theta:%.4f", x, y, theta);
#endif
  double velocity = 0.0;
  double steering = 0.0;
  double filtered_accel = 0.0;
  if (feedback_vel.first) {
    velocity = feedback_vel.second[0];
    steering = feedback_vel.second[1];//wheel steering angle in radius
    filtered_accel = feedback_vel.second[2];
  }
  //-----------------------
  autogo_msgs::TrajectoryPoint ego_pose;
   ego_pose.path_point.point.x=0; ego_pose.path_point.point.y=0; ego_pose.path_point.point.z=0;
   ego_pose.v = velocity;         ego_pose.path_point.theta = 0; ego_pose.path_point.kappa = steering;//trick
  //------------------------
  double t = cfg_mpc->configMPC.lookingforward_time;
  lookingforward_distance = std::min(cfg_mpc->configMPC.lookingforward_distance_upperbound,
	                        std::max(t * velocity / (1 + 0.5 * kappa_pre), cfg_mpc->configMPC.lookingforward_distance_lowerbound));
  cout<<"solve CP: looking_dis="<<lookingforward_distance<<endl;
  int bound_count = int(lookingforward_distance -3 );
  bound_count = bound_count > 30 ? 30 : bound_count;
  //N = bound_count;
#ifdef DEBUG1
  ROS_INFO("solve CP2: lookingforward_distance_lowerbound=%f,t=%f,velocity=%f",cfg_mpc->configMPC.lookingforward_distance_lowerbound,t, velocity);
  ROS_INFO("solve CP3: heading_compensation: %.4f, kappa_pre : %.4f, lookingforward_distance: %.4f",
             cfg_mpc->configMPC.heading_compensation, kappa_pre, lookingforward_distance);
#endif
  std::vector<autogo_msgs::TrajectoryPoint> local_trajectory;
  GetTrajectoryForControl(x, y, theta, planning_published_trajectory, local_trajectory);
  /*--------add trick for traj extraction-------*/
  int extract_mod = 0; //0 for point follow, 1 for time-region trajectory
//  ROS_INFO("xxxx1 local traj.v=%f",local_trajectory[1].v);
#ifdef POINTFOLLOW
  extract_mod = 0;
  TrajExtraction(local_trajectory, extract_mod, dt);
#else
  extract_mod = 1;
  TrajExtraction(local_trajectory,extract_mod, dt);
#endif
#ifdef DEBUG1
  cout<<"x="<<x<<", y="<<y<<",theta="<<theta<<", planning_pubTraj.size="<<planning_published_trajectory.size()<<",local_traj.size="<<local_trajectory.size()<<endl;
#endif
  if (num_of_next_waypoint_ == -1 || local_trajectory.size() < 4){
	  ROS_WARN("lost next waypoint! Soft brake!");
	  Reset();
	  return false;
  }
  //----------------------------------------------
  fitted_waypoints_.clear();
  fitted_trajectory_.clear();
  fitted_waypoints_.assign(local_trajectory.begin(), local_trajectory.end());
  //----------------------------------------------
  Eigen::VectorXd coeffs = GetPolyfitCoeffs(local_trajectory, cfg_mpc->configMPC.regularized_constant);
  FixKappa(coeffs, local_trajectory);
  autogo_msgs::TrajectoryPoint point;
  for (size_t i = 0; i < local_trajectory.size(); i++) {
    point.path_point.point.x = local_trajectory[i].path_point.point.x;
    point.path_point.point.y = polyeval(coeffs, point.path_point.point.x);
    point.v = local_trajectory[i].v;
#ifdef DEBUG
    ROS_ERROR("velocity command [%d]= %f m/s",i, local_trajectory[i].v);
#endif
    fitted_trajectory_.emplace_back(point);
  }
#ifdef DEBUG
  ROS_ERROR("next_waypoint = %d", num_of_next_waypoint_);
#endif
  /*------------------------end of local traj extraction-----------------------------*/
  // Get the preview Traj Point info based on the pre-defined preview time

  //TODO: targetPt->relative_point的工作和上面重复了吧，问桥北？？？
#ifdef POINTFOLLOW
  autogo_msgs::TrajectoryPoint relative_point = fitted_trajectory_[num_of_next_waypoint_];
#else
  autogo_msgs::TrajectoryPoint targetPt = GetNearestPointByRelativeTime(t, planning_published_trajectory);
  autogo_msgs::TrajectoryPoint relative_point = FromWorldToPath(x, y, theta, targetPt);
#endif
#ifdef DEBUG1
  ROS_INFO("solve cp 3.1: targetPt.x=%.4f, .y=%.4f, .z=%.4f, .v=%f",targetPt.path_point.point.x,targetPt.path_point.point.y,targetPt.path_point.point.z,targetPt.v);
  ROS_INFO("solve cp 3.2: relativePt.x=%.4f, .y=%.4f, .z=%.4f, .v=%f",relative_point.path_point.point.x,relative_point.path_point.point.y,relative_point.path_point.point.z,relative_point.v);
#endif
  kappa = GetTrajectoryKappa(fitted_trajectory_);
  double px = 0.1;//0
  double py = 0;
  double psi = 0;
  //TODO: why modify coeffs[0] with deadzone???
  coeffs[0] = coeffs[0] > 0
                  ? coeffs[0] > cfg_mpc->configMPC.dead_zone ? coeffs[0] - cfg_mpc->configMPC.dead_zone : 0.0
                  : coeffs[0] < -cfg_mpc->configMPC.dead_zone ? coeffs[0] + cfg_mpc->configMPC.dead_zone : 0.0;

  double cte = polyeval(coeffs, px);
  double epsi = -atan(coeffs[1]);
  double desired_velocity = relative_point.v + cfg_mpc->configMPC.speed_up;
  //area reset condition?
  area_ += cte * dt;
  double area = std::min(std::max(cfg_mpc->configMPC.area_lowerbound, area_),
		                          cfg_mpc->configMPC.area_upperbound);

#ifdef DEBUG
   ROS_INFO("solve CP1.1  mpc: relative_point.v=%f,cte: %.4f, epsi: %.4f",relative_point.v,cte, epsi);
   ROS_INFO("solve CP1.2  mpc: ego_pos.x=0, relative_point.x=%f, ego_pos.y=0, relative_point.y=%f",relative_point.path_point.point.x,relative_point.path_point.point.y);
   ROS_INFO("solve CP1.3  mpc: desired_velocity: %f, area: %f, size: %lu, velocity: %f, steering: %f, filtered_accel: %f",
            desired_velocity, area, fitted_trajectory_.size(), velocity, steering, filtered_accel);
#endif
   if(METHOD=="NMPC")
   {
	   bool is_solveable = SolveFgNL(ego_pose, fitted_trajectory_, lastAct, result);
	   cout<<"is_solveable ="<<is_solveable<<endl;
	   //ROS_INFO("after solve CP1,ego_x=%f,_y=%f,steer=%f,vel=%f",ego_pose.path_point.point.x,ego_pose.path_point.point.y,ego_pose.path_point.kappa,ego_pose.v);
	   if (!is_solveable) {
		  Reset();
		  return false;
	   }

	   Eigen::Vector4d xt, xt1;
	   xt <<  ego_pose.path_point.point.x,
	       	  ego_pose.path_point.point.y,
	   		  ego_pose.path_point.kappa,
	   		  ego_pose.v;

	   autogo_msgs::TrajectoryPoint gp;

        /* -----------local frame---------------*/
	   	gp.path_point.point.x = xt[0];
	   	gp.path_point.point.y = xt[1];
	   	gp.path_point.point.z = 1;
	    /* -----------map frame---------------*/
//	   	gp.path_point.point.x = cos(theta) * xt[0] - sin(theta) * xt[1];
//	   	gp.path_point.point.y = sin(theta) * xt[0] + cos(theta) * xt[1];
//	   	gp.path_point.point.z = 1;
	   	predicted_trajectory_.push_back(gp);
	   	//ROS_INFO("after solve CP3");
	   	for(int i = 0; i < N; ++i) {
	   		double beta = CppAD::atan(( 2.85/2 + 0.0) * CppAD::tan(this->solution[i]) / 2.85);
	   		xt1[0] = xt[0] + xt[3] * CppAD::cos(xt[2]+beta) * dt;
	   		xt1[1] = xt[1] + xt[3] * CppAD::sin(xt[2]+beta) * dt;
	   		xt1[2] = xt[2] + xt[3] * CppAD::cos(beta) * CppAD::tan(this->solution[i]) * dt / 2.85;
	   		xt1[3] = xt[3] + this->solution[i+N] * dt;
	   		/* -----------local frame---------------*/
	   		gp.path_point.point.x = xt1[0];
	   		gp.path_point.point.y = xt1[1];
	   		gp.path_point.point.z = 1;
	   		/* -----------map frame---------------*/
//	   		gp.path_point.point.x = cos(theta) * xt1[0] - sin(theta) * xt1[1];
//			gp.path_point.point.y = sin(theta) * xt1[0] + cos(theta) * xt1[1];
//			gp.path_point.point.z = 1;
	   		predicted_trajectory_.push_back(gp);
	   		xt = xt1;
	   	}
	   	//ROS_INFO("after solve CP1");
	   	 for (int i = 0; i<=3; i++) {
	   		lastAct[i] = this->solution[i];
	   		lastAct[i+4] = this->solution[i + N];
	   	}

   } else {
	   Eigen::VectorXd state(9);
	   state << px, py, psi, cte, epsi, area, velocity, steering, filtered_accel;
	   bool is_solveable = SolveFg(state, coeffs, desired_velocity, kappa, result);
	   if (!is_solveable) {
	      Reset();
	      return false;
	   }
   }
  return true;
}
/************************************************************************************************************/
class FG_eval_vdelta_no_latency {
 public:
  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  /* FG_eval_vdelta_no_latency
   * 输入包括
   * coeffs:拟合的轨迹abcd系数
   * 速度跟踪指令vel_ref
   * 曲率kappa
   * config参数: 具体：请见trajectory_foller_config.h中的struct定义，较多
   *                 主要包括：coeff_cte,_epsi,_delta,_vel,_accel,_vel_delta,_diff_vel,_diff_delta, _area
   *                         kappa_for_vel, for_diff_delta, for_delta, for_area
   *                         lubound: steer, vel, accel, area, delta_constraints, vel_constraints, accel_constraints
   *                         其他
   * */
  FG_eval_vdelta_no_latency(const TrajectoryFollowerConfig* cfg,
							const std::vector<autogo_msgs::TrajectoryPoint>& traj,
							Eigen::VectorXd coeffs,
							const double& ref_vel,
							const double& ref_kappa)
  {
    this->coeffs_ = coeffs;
    this->cfg = cfg;
    ros::param::get("~mpc/wheel_base", this->wheel_base_);
    this->rvel = ref_vel;
    this->rkappa = ref_kappa;
    this->traj_ = traj;
    //ROS_INFO("FG_eval: kappa: %.4f, wheel_base: %.4f",  this->rkappa, this->wheel_base_);
  }

  void operator()(ADvector& fg, const ADvector& vars) {
	/* fg 是包含cost和constrains的向量
	*  vars是包含变量值的向量（状态与执行器，状态存储的是被跟踪的轨迹）
	*  fg[0]是cost值
	* */
	fg[0] = 0;
    /*================== vars 是超长参数vector=======================*/
    /*--------------cost function 构建--------------------
    * -----Part1: reference state 项： cte(cross tracking error), epsi（方向error）, area项（横向误差积分） */
    for (size_t i = 0; i < N; i++) {
      fg[0] += this->cfg->configMPC.coeff_cte * CppAD::pow(vars[cte_start_ + i], 2);
      fg[0] += this->cfg->configMPC.coeff_epsi * CppAD::pow(vars[epsi_start_ + i], 2);
      fg[0] += this->cfg->configMPC.coeff_area * CppAD::pow(vars[area_start_ + i], 2) *
               std::max(1 - this->rkappa / this->cfg->configMPC.kappa_for_area, 0.1);
      fg[0] += this->cfg->configMPC.coeff_vel * CppAD::pow(vars[v_start_ + i] - this->rvel, 2) * 1.0;
                    //std::max(1 - this->rkappa / this->cfg->configMPC.kappa_for_vel, 0.1);

    }
      /* -----Part2: 执行器项： erorr velocity， 轮转角delta, 加速度a(add by junlong gao)
      * -----TODO:  vel*delta(这个耦合项干什么用？ trade off 速度与轮转角的关系(使用向心加速度不行吗 -_-|||))
      *  */
    for (size_t i = 0; i < N - 1; i++) {
      fg[0] += this->cfg->configMPC.coeff_delta * CppAD::pow(vars[delta_start_ + i], 2) * 1;
               //std::max(1 - this->rkappa / this->cfg->configMPC.kappa_for_delta, 0.1);
      fg[0] += this->cfg->configMPC.coeff_vel_delta * CppAD::pow(vars[v_start_ + i] * vars[delta_start_ + i], 2);
      fg[0] += this->cfg->configMPC.coeff_accel * CppAD::pow(vars[a_start_ + i], 2);
    }
    /*--------------cost function 构建--------------------
	 * -----Part3: 最小化sequantial actuations： d_delta项，d_vel项， dd_vel项(add by Junlong Gao)
	 *  */
    for (size_t i = 0; i < N - 2; i++) {
      fg[0] += this->cfg->configMPC.coeff_diff_delta *
               CppAD::pow(vars[delta_start_ + i + 1] - vars[delta_start_ + i], 2) * 1;
               //std::max(1 - this->rkappa / this->cfg->configMPC.kappa_for_diff_delta, 0.1);
      fg[0] += this->cfg->configMPC.coeff_diff_vel * CppAD::pow(vars[v_start_ + i + 1] - vars[v_start_ + i], 2);
      fg[0] += this->cfg->configMPC.coeff_diff_accel * CppAD::pow(vars[a_start_ + i + 1] - vars[a_start_ + i], 2);
    }
   /*==========================end of cost func==============================*/
    /*-------------------cte,epsi,area是初始的值--调试窗口区--------------------*/
       AD<double> cte  = vars[cte_start_];
       AD<double> epsi = vars[epsi_start_];
       AD<double> area = vars[area_start_];

       AD<double> cte_cost = this->cfg->configMPC.coeff_cte * CppAD::pow(cte, 2);
       AD<double> epsi_cost = this->cfg->configMPC.coeff_epsi * CppAD::pow(epsi, 2);
       AD<double> area_cost = this->cfg->configMPC.coeff_area * CppAD::pow(area, 2) *
                              std::max(1 - this->rkappa / this->cfg->configMPC.kappa_for_area, 0.1);
#ifdef DEBUG
       ROS_INFO_STREAM("cost: cte: " << cte_cost << " epsi: " << epsi_cost << " area: " << area_cost);
#endif
   /*=================start of vehicle differential model====================*/
    fg[1 + x_start_] = vars[x_start_];
    fg[1 + y_start_] = vars[y_start_];
    fg[1 + psi_start_] = vars[psi_start_];
    fg[1 + cte_start_] = vars[cte_start_];
    fg[1 + epsi_start_] = vars[epsi_start_];
    fg[1 + area_start_] = vars[area_start_];
    fg[1 + v_start_] = vars[v_start_];
    for (size_t t = 1; t < N; t++) {
      // State
      AD<double> x1 = vars[x_start_ + t];       AD<double> x0 = vars[x_start_ + t - 1];
      AD<double> y1 = vars[y_start_ + t];       AD<double> y0 = vars[y_start_ + t - 1];
      AD<double> psi1 = vars[psi_start_ + t];   AD<double> psi0 = vars[psi_start_ + t - 1];
      AD<double> cte1 = vars[cte_start_ + t];   AD<double> cte0 = vars[cte_start_ + t - 1];
      AD<double> epsi1 = vars[epsi_start_ + t]; AD<double> epsi0 = vars[epsi_start_ + t - 1];
      AD<double> area1 = vars[area_start_ + t]; AD<double> area0 = vars[area_start_ + t - 1];
      AD<double> v1 =  vars[v_start_ + t ];     AD<double> v0 =  vars[v_start_ + t - 1];
#ifdef DEBUG1
       ROS_INFO("operator CP2 t=%d",t);
#endif
      // Actuator
      // a: accel, delta: steer angle
      AD<double> a0 =  vars[a_start_ + t - 1];
      AD<double> delta0 = vars[delta_start_ + t - 1];

      AD<double> f0 = coeffs_[0];
      AD<double> df0 = coeffs_[1];
      if (coeffs_.size() > 1) {
        for (int i = 1; i < coeffs_.size(); i++) {
          f0 += coeffs_[i] * CppAD::pow(x0, i);
          if (i < coeffs_.size() - 1) df0 += coeffs_[i + 1] * CppAD::pow(x0, i) * (i + 1);
        }
      }
      AD<double> psides0 = CppAD::atan(df0);
      AD<double> beta = CppAD::atan( this->cfg->configMPC.LF / 2 * CppAD::tan(delta0) / this->cfg->configMPC.LF);
      //误差形式的自行车模型
      fg[1 + x_start_ + t] = x1 - (x0 + v0 * CppAD::cos(psi0 + beta) * dt);
      fg[1 + y_start_ + t] = y1 - (y0 + v0 * CppAD::sin(psi0 + beta) * dt);
      fg[1 + psi_start_ + t] = psi1 - (psi0 + v0 * CppAD::cos(beta) * CppAD::tan(delta0) * dt / this->cfg->configMPC.LF);
      fg[1 + v_start_ + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start_ + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start_ + t] = epsi1 - ((psi0 - psides0) + v0 * CppAD::cos(beta) * CppAD::tan(delta0) * dt / this->cfg->configMPC.LF);
      fg[1 + area_start_ + t] = area1 - (area0 + cte0 * dt);
    }
#ifdef DEBUG1
       ROS_INFO("operator done()");
#endif
  }

 private:
  double wheel_base_, rvel, rkappa;
  std::vector<autogo_msgs::TrajectoryPoint> traj_;
  Eigen::VectorXd coeffs_;
  Eigen::VectorXd lastAct_;
  const TrajectoryFollowerConfig* cfg;  //!< Config class that stores and manages all related parameters
};
/************************************************************************************************************/

// control: velocity and steering angle, and no account for latency
bool MPC::SolveFg(const Eigen::VectorXd& state, const Eigen::VectorXd& coeffs, const double vel_ref, const double kappa,
                  std::vector<double>& result) {
  bool ok = true;
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double cte = state[3];
  double epsi = state[4];
  double area = state[5];
  double v = state[6];
  double delta = state[7];
  double acceleration = state[8];
  /*Set the number of model variables (includes both states and inputs).
  * The state is a 6 element vector, the actuators is a 3
  * element vector and there are fixed timesteps. The number of variables is:
  */
  size_t n_vars = 7 * N + 2 * (N - 1);// N-1: acceleration & wheel steer
  /* Set the number of constraints */
  size_t n_constraints = 7 * N; //including v
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  /*-------------设置状态变量（控制变量前）的上下界(不设界)------------*/
	  for (size_t i = 0; i < v_start_; i++) {
		vars_lowerbound[i] = -1.0e19;
		vars_upperbound[i] =  1.0e19;
	  }
  /*------------------细微调整横向跟踪误差积分上下界-----------------*/
      for (size_t i = area_start_; i < v_start_; i++) {
		vars_lowerbound[i] = cfg_mpc->configMPC.area_lowerbound;
		vars_upperbound[i] = cfg_mpc->configMPC.area_upperbound;
      }
  // TODO: 添加 cte 和 epsi 对转角限制的影响
  /**********************控制量限制条件设置************************/
  /*-----------------设置速度上下界限制速度限制----------------------
   * -----不考虑cte和epsi, 除了轨迹弯曲程度较大，不考虑低速修正------ */
#ifdef POINTFOLLOW
      for (size_t i = v_start_; i < delta_start_; i++) {
		vars_lowerbound[i] = cfg_mpc->configMPC.vel_lowerbound;  // m/s
		vars_upperbound[i] = cfg_mpc->configMPC.vel_upperbound;
//				std::max(std::min(cfg_mpc->configMPC.vel_upperbound, vel_ref) -
//										std::max(kappa * cfg_mpc->configMPC.kappa_for_vel_upperbound,
//												 std::max(fabs(cte) * cfg_mpc->configMPC.cte_for_vel_upperbound,
//														  fabs(epsi) * cfg_mpc->configMPC.epsi_for_vel_upperbound)),
//									0.5);
      }
#else
      for (size_t i = v_start_; i < delta_start_; i++) {
      		vars_lowerbound[i] = cfg_mpc->configMPC.vel_lowerbound;  // m/s
      		vars_upperbound[i] =
						std::max(std::min(cfg_mpc->configMPC.vel_upperbound, vel_ref) -
												std::max(kappa * cfg_mpc->configMPC.kappa_for_vel_upperbound,
														 std::max(fabs(cte) * cfg_mpc->configMPC.cte_for_vel_upperbound,
																  fabs(epsi) * cfg_mpc->configMPC.epsi_for_vel_upperbound)),
											0.5);
      }
#endif
	  /*-----------设置轮转角delta上下界（弧度制）------------*/
	  for (size_t i = delta_start_; i < a_start_; i++) {
		// 转角限制为 kappa、cte、epsi除以对应阈值之和
		vars_lowerbound[i] = -0.45;//0.523
//			std::max(cfg_mpc->configMPC.steer_lowerbound,
//					 std::min(cfg_mpc->configMPC.steer_lowerbound *
//								  std::max(kappa / cfg_mpc->configMPC.kappa_threshold + 0.001,
//										   std::max(fabs(cte) / cfg_mpc->configMPC.cte_threshold + 0.001,
//													fabs(epsi) / cfg_mpc->configMPC.epsi_threshold + 0.001)),
//							  -0.01));
		vars_upperbound[i] = 0.45;
//			std::min(cfg_mpc->configMPC.steer_upperbound,
//					 std::max(cfg_mpc->configMPC.steer_upperbound *
//								  std::max(kappa / cfg_mpc->configMPC.kappa_threshold + 0.001,
//										   std::max(fabs(cte) / cfg_mpc->configMPC.cte_threshold + 0.001,
//													fabs(epsi) / cfg_mpc->configMPC.epsi_threshold + 0.001)),
//							  0.01));
	  }
	  for (size_t i =a_start_; i< n_vars; i++)
	  {
		  vars_lowerbound[i] = -2.5;
		  vars_upperbound[i] =  1.0;
	  }


#ifdef DEBUG
  ROS_WARN("SolveFG CP1 kappa/threshold: %.4f | cte/threshold: %.4f | epsi/threshold: %.4f",
           kappa / cfg_mpc->configMPC.kappa_threshold, fabs(cte) / cfg_mpc->configMPC.cte_threshold,
           fabs(epsi) / cfg_mpc->configMPC.epsi_threshold);
//  ROS_WARN_STREAM("SolveFG CP2"<<vars_lowerbound[delta_start_] << " < steering < " << vars_upperbound[delta_start_]
//				  << " | " << vars_lowerbound[v_start_] << " < v < " << vars_upperbound[v_start_]);
#endif
  /*-----------状态&控制变量初值设置--------------*/
  vars[x_start_] = x;
  vars[y_start_] = y;
  vars[psi_start_] = psi;
  vars[cte_start_] = cte;
  vars[epsi_start_] = epsi;
  vars[area_start_] = area;
  vars[v_start_] = v;
  vars[delta_start_] = delta;
  vars[a_start_] = acceleration;

  vars_lowerbound[delta_start_] = delta;
  vars_upperbound[delta_start_] = delta;
  vars_lowerbound[a_start_] = acceleration;
  vars_upperbound[a_start_] = acceleration;

  /*-------------限制条件上下界设置----------------*/
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  /*---------------限制条件初始化------------------*/
  for (size_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start_] = x;
  constraints_upperbound[x_start_] = x;
  constraints_lowerbound[y_start_] = y;
  constraints_upperbound[y_start_] = y;
  constraints_lowerbound[psi_start_] = psi;
  constraints_upperbound[psi_start_] = psi;
  constraints_lowerbound[cte_start_] = cte;
  constraints_upperbound[cte_start_] = cte;
  constraints_lowerbound[epsi_start_] = epsi;
  constraints_upperbound[epsi_start_] = epsi;
  constraints_lowerbound[area_start_] = area;
  constraints_upperbound[area_start_] = area;
  constraints_lowerbound[v_start_] = v;
  constraints_upperbound[v_start_] = v;


  /*TODO：在有轨迹的情况下还重新传入abcd系数是否必要*/
  FG_eval_vdelta_no_latency fg_eval(cfg_mpc, fitted_trajectory_, coeffs, vel_ref, kappa);
#ifdef DEBUG1
  ROS_WARN("SolveFg CP3 before solve");
#endif
  std::string options;
  options += "Integer print_level  1\n";  // Uncomment this if you'd like more print information
  options += "Sparse true forward\n";
  options += "Sparse true reverse\n";     // Setting sparse to true allows the solver to take advantage of sparse
                                          // routines, this makes the computation MUCH FASTER
  options += "Numeric max_cpu_time   0.5\n";  // Currently the solver has a maximum time limit of 0.5 seconds.
//   options += "String  sb           yes\n";
//   options += "Integer max_iter     100\n";
//   options += "Numeric max_cpu_time 0.05\n";
//   options += "Numeric tol          1e-1\n";
//   options += "String  derivative_test  second-order\n";
//   options += "Numeric point_perturbation_radius  0.\n";
//   options += "String print_timing_statistics yes\n";

  CppAD::ipopt::solve_result<Dvector> solution;
  clock_t start = clock();
  CppAD::ipopt::solve<Dvector, FG_eval_vdelta_no_latency>(
		  options, vars, vars_lowerbound, vars_upperbound,
          constraints_lowerbound, constraints_upperbound,
		  fg_eval, solution);
  clock_t end = clock();
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
  if (!ok) {
    ROS_ERROR("mpc: no solution is found!");
    return false;
  }
#ifdef DEBUG
  std::cout<<"time = "<< (double (end - start)) / double (CLOCKS_PER_SEC)<<std::endl;
  ROS_WARN("SolveFg CP4 after ippod solve");
#endif

  result.push_back(solution.x[delta_start_ + latency_interval_delta]);
  result.push_back(solution.x[a_start_ + latency_interval_delta]);

  predicted_trajectory_.clear();
  past_delta.erase(past_delta.begin());
  past_acc.erase(past_acc.begin());
  past_delta.emplace_back(solution.x[delta_start_ + 1 + delay_time]);
  past_acc.emplace_back(solution.x[a_start_ + 1 + delay_time]);

  for (size_t i = 0; i < N - 2; i++) {
    result.push_back(solution.x[x_start_ + i + 1]);
    result.push_back(solution.x[y_start_ + i + 1]);
    result.push_back(solution.x[psi_start_ + i + 1]);
    result.push_back(solution.x[v_start_ + i + 1]);
    result.push_back(solution.x[delta_start_ + i + 1]);
    result.push_back(solution.x[a_start_ + i + 1]);
    autogo_msgs::TrajectoryPoint point;
    point.path_point.point.x = solution.x[x_start_ + i + 1];
    point.path_point.point.y = solution.x[y_start_ + i + 1];
    predicted_trajectory_.emplace_back(point);
  }
//  result.push_back(solution.x[x_start_ + N - 1]);
//  result.push_back(solution.x[y_start_ + N - 1]);
//  result.push_back(solution.x[psi_start_ + N - 1]);
  return true;
}
/*----------------------end of MPC module---------------------------------*/

}  // namespace trajectory_follower
