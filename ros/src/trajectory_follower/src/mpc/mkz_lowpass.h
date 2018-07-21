/***
 * created by Ye Bo
 * date: 2018-1-29
 */

#ifndef MKZ_LOW_PASS_H
#define MKZ_LOW_PASS_H

namespace trajectory_follower{

class MkzLowPass{

public:
//	MkzLowPass() :  ready_(false), last_val_(0){
//		a_ = 1;
//		b_ = 0;
//	}
//
//	MkzLowPass(double tau, double ts) : ready_(false), last_val_(0){
//		setParams(tau, ts);
//	}
//
//	void setParams(double tau, double ts){
//		a_ = 1 / (tau / ts + 1);
//		b_ = tau / ts / (tau / ts + 1);
//	}
//
//	double filt(double val){
//		if(ready_){
//			val = a_ * val + b_ * last_val_;
//		}else{
//			ready_ = true;
//		}
//		last_val_ = val;
//		return val;
//	}
//
//private:
//
//	bool ready_;
//	double a_;
//	double b_;
//	double last_val_;
//
	MkzLowPass() : ready_(false){ a_ = 1; b_ = 0; }
	MkzLowPass(double tau, double ts) : ready_(false) { setParams(tau, ts); }
	void setParams(double tau, double ts) {
	    a_ = 1 / (tau / ts + 1);
	    b_ = tau / ts / (tau / ts + 1);
	    count_l = 0;
	  }
	double get() { return last_val_; }
	double filt(double val) {
	    if (ready_) {
	    	if(count_l == 1){
	    		val = 1 * val;
	    		count_l = count_l + 1;
	    	}
	    	else
	    		val = a_ * val + b_ * last_val_;
	      last_val_ = val;//初始化后的一轮
	      return val;
	    } else {
	      ready_ = true;
	      last_val_ = 0;//输入的第一轮值不准，不做参考，直接输出0
	      count_l = count_l + 1;
	      return last_val_;
	    }
	  }
	private:
	  bool ready_;
	  double a_;
	  double b_;
	  double last_val_ = 0;
	  int count_l = 0;
};	
	
}//namespace trajectory_follower

#endif //MKZ_LOW_PASS_H
