#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
	is_initialized_ = false;

	previous_timestamp_ = 0;

	// initializing matrices
	R_laser_ = MatrixXd(2, 2);
	R_radar_ = MatrixXd(3, 3);
	H_laser_ = MatrixXd(2, 4);
	Hj_ = MatrixXd(3, 4);

	//measurement covariance matrix - laser
	R_laser_ << 0.0225, 0,
							0, 0.0225;

	//measurement covariance matrix - radar
	R_radar_ << 0.09, 0, 0,
							0, 0.0009, 0,
							0, 0, 0.09;

	//laser H measurement matrix
	H_laser_<< 1, 0, 0, 0,
	0, 1, 0, 0;

	//Radar H
	Hj_ << 0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0;
}


/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}


void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


	/*****************************************************************************
	 *  Initialization
	 ****************************************************************************/
	if (!is_initialized_) {
		
		Eigen::VectorXd x_in;

		// state covariance matrix
		Eigen::MatrixXd P_in;

	// state transistion matrix
		Eigen::MatrixXd F_in;

	// process covariance matrix
		Eigen::MatrixXd Q_in;

		x_in = VectorXd(4);
		
		P_in = MatrixXd(4, 4);
		P_in << 1, 0, 0, 0,
						0, 1, 0, 0,
						0, 0, 1000, 0,
						0, 0, 0, 1000;

		F_in = MatrixXd(4, 4);
		F_in << 1, 0, 1, 0,
						0, 1, 0, 1,
						0, 0, 1, 0,
						0, 0, 0, 1;

		Q_in = MatrixXd(4, 4);
		Q_in << 0,0,0,0,
						0,0,0,0,
						0,0,0,0,
						0,0,0,0;
		
		if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
			/**
			Convert radar from polar to cartesian coordinates and initialize state.
			*/

			float p = measurement_pack.raw_measurements_[0];
			float theta = measurement_pack.raw_measurements_[1];
			float v = measurement_pack.raw_measurements_[2];

			float x = p*std::cos(theta);
			float y = p*std::sin(theta);
			float xv = v*std::cos(theta);
			float yv = v*std::sin(theta);

			x_in << x,y,xv,yv;
			
			ekf_.Init(x_in, P_in, F_in, Hj_, R_radar_, Q_in) ;

			previous_timestamp_ = measurement_pack.timestamp_;
			is_initialized_ = true;
		}
		else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
			//set the state with the initial location and zero velocity
			// measurement matrix

			x_in <<  measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;

			ekf_.Init(x_in, P_in, F_in, H_laser_, R_laser_, Q_in) ;

			previous_timestamp_ = measurement_pack.timestamp_;
			is_initialized_ = true;

		}

		// done initializing, no need to predict or update
		
		return;
	}

	/*****************************************************************************
	 *  Prediction
	 ****************************************************************************/

	 //compute the time elapsed between the current and previous measurements
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; //dt - expressed in seconds
	previous_timestamp_ = measurement_pack.timestamp_;


	/**
		 * Update the state transition matrix F according to the new elapsed time.
			- Time is measured in seconds.
		 * Update the process noise covariance matrix.
		 * Use noise_ax = 9 and noise_ay = 9 for Q matrix.
	 */

	//update state transition
	ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ << 	1, 0, dt, 0,
							0, 1, 0, dt,
							0, 0, 1, 0,
							0, 0, 0, 1;

	float noise_ax = 9;
	float noise_ay = 9;
	float dt2 = dt*dt;
	float dt3 = dt2*dt;
	float dt4 = dt3*dt;
	
	ekf_.Q_ = MatrixXd(4, 4);
	ekf_.Q_ <<	 .25*dt4*noise_ax,	0, .5*dt3*noise_ax,	0,
				0,	.25*dt4*noise_ay,	0,	.5*dt3*noise_ay,
				.5*dt3*noise_ax,	0, dt2*noise_ax,	0,
				0,	.5*dt3*noise_ay,	0,	dt2*noise_ay;


	cout << "_________________PREDICT__________________" << endl;
	ekf_.Predict();
	cout << "x_ = " << ekf_.x_ << endl;
	cout << "P_ = " << ekf_.P_ << endl;

	/*****************************************************************************
	 *  Update
	 ****************************************************************************/

	/**
		 * Use the sensor type to perform the update step.
		 * Update the state and covariance matrices.
	 */

	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
		// Radar updates
		cout << "_______________RADAR UPDATE_______________" << endl;
		ekf_.R_ = R_radar_;
		Hj_ = tools.CalculateJacobian(ekf_.x_);
		ekf_.H_ = Hj_;

		ekf_.UpdateEKF(measurement_pack.raw_measurements_);
		
	} else {
		// Laser updates
		cout << "_______________LIDAR UPDATE_______________" << endl;
		ekf_.R_ = R_laser_;
		ekf_.H_ = H_laser_;
		ekf_.Update(measurement_pack.raw_measurements_);

	}

	// print the output
	cout << "x_ = " << ekf_.x_ << endl;
	cout << "P_ = " << ekf_.P_ << endl;
}
