#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
	const vector<VectorXd> &ground_truth) {
	VectorXd rmse(4);
	rmse << 0,0,0,0;

	
	if( estimations.size() ==0 || estimations.size()!=ground_truth.size()){
		std::cout << "RMSE failed, no data!";
		return rmse;
	}
	
	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
		VectorXd res = estimations[i] - ground_truth[i];
		res = res.array()*res.array();
		rmse += res;
	}

	//calculate the mean
	rmse/=estimations.size();
	//calculate the squared root
	rmse = rmse.array().sqrt();
	
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//check division by zero
	if(px ==0 && py==0){
		Hj << 0,0,0,0,0,0,0,0,0,0,0,0;
		//std::cout << "divided by zero" << std::endl;
		//TODO relcalculate partials when px==py. for now just return 0
		return Hj;
	}
	float s = px*px+py*py;
	float sq = sqrt(s);
	float s32 = pow(s,3/2.0);
	Hj << 	px/sq , py/sq, 0,0,
			-1.0*py/s,px/s,0,0,
			py*(vx*py-vy*px)/s32,px*(vy*px-vx*py)/s32, px/sq,py/sq; 

	return Hj;
}
