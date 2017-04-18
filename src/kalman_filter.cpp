#include "kalman_filter.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;


KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
	MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
	x_ = x_in;
	P_ = P_in;
	F_ = F_in;
	H_ = H_in;
	R_ = R_in;
	Q_ = Q_in;
}

void KalmanFilter::Predict() {
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
	
	VectorXd z_pred = H_ * x_;

	VectorXd y = z - z_pred;

	MatrixXd Ht = H_.transpose();

	//std::cout << "H_= " << H_ << std::endl;
	//std::cout << "Hr_= " << Ht << std::endl;;
	//std::cout << "P_= " << P_ << std::endl;;
	//std::cout << "R_= " << R_ << std::endl;;

	MatrixXd S = H_ * P_ * Ht + R_;

	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	/**
		* update the state by using Extended Kalman Filter equations
	*/
	float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
	float vy = x_(3);

	if(px ==0 && py==0){
		std::cout << "skipping update due to divide by zero" << std::endl;
		return ;
	}

	
	VectorXd z_pred(3);
	z_pred << sqrt(px*px+py*py)
	,std::atan2(py,px)
	,(px*vx+py*vy)/sqrt(px*px+py*py);

	VectorXd y = z - z_pred;

	//make sure the angle is between -pi and pi 
	float theta = y(1);
	if( theta < -1.0*M_PI){
		y(1)+=2.0*M_PI;
	}else if( theta > M_PI){
		y(1) -= 2.0*M_PI;
	}


	MatrixXd Ht = H_.transpose();

	//std::cout << "H_= " << H_ << std::endl;
	//std::cout << "Hr_= " << Ht << std::endl;;
	//std::cout << "P_= " << P_ << std::endl;;
	//std::cout << "R_= " << R_ << std::endl;;
	
	MatrixXd S = H_ * P_ * Ht + R_;

	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;

}
