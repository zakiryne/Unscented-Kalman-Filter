#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

#define PI 3.14

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {

  is_initialized_ = false;
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
		0, 0, 0, 0, 1;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.5; //30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.57; //30;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.--------------------
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.---------------------
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  ///* State dimension
  n_x_ = 5;

  ///* Augmented state dimension
  n_aug_ = 7;

  ///* Sigma point spreading parameter
  lambda_ = 3 - n_aug_;


  ///* Weights of sigma points
  // set weights
  //set vector for weights
  weights = VectorXd(2*n_aug_+1);
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights(0) = weight_0;
  for (int i = 1; i < 2*n_aug_ + 1; i++) {  //2n+1 weights
	  double weight = 0.5/(n_aug_ + lambda_);
	  weights(i) = weight;
  }

  //measurement covariance matrix - lidar
  R_lidar_ = MatrixXd(2, 2);
  R_lidar_ << std_laspx_*std_laspx_, 0.0000,
  		  	  0.0000, std_laspy_*std_laspy_;

  //measurement covariance matrix - radar
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << std_radr_*std_radr_, 0.0000, 0.0000,
  		  	  0.0000, std_radphi_*std_radphi_, 0.0000,
  			  0.0000, 0.0000, std_radrd_*std_radrd_;

  //create example matrix with predicted sigma points
  Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);


}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

	long long CurrentTimeStamp;

	float rho, phi, V;
	float Px, Py, Vx, Vy;
	//Tools tool;
	double delta_t;


	CurrentTimeStamp = meas_package.timestamp_;



	if (!is_initialized_) {
		/**
		TODO:
		 * Initialize the state ukf_.x_ with the first measurement.
		 * Create the covariance matrix.
		 * Remember: you'll need to convert radar from polar to cartesian coordinates.
		 */
		// first measurement
		cout << "UKF: " << endl;
		x_ = VectorXd(5);
		x_ << 	1,  // Px
				1,  // Py
				1,  // V
				1,	// ψ
				1;	// ψ'

		if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
			/**
	  	  Convert radar from polar to cartesian coordinates and initialize state.
			 */

			rho = meas_package.raw_measurements_[0];  // range
			phi = meas_package.raw_measurements_[1]; // bearing
			V = meas_package.raw_measurements_[2]; //range velocity

			Px = rho*cos(phi);
			Py = rho*sin(phi);
			Vx = V*cos(phi);
			Vy = V*sin(phi);

			x_ << 	Px,
					Py,
					V,
					phi,
					0;
		}

		else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
			/**
	  	  Initialize state.
			 */
			//set the state with the initial location and zero velocity
			// We don't know velocities from the first measurement of the LIDAR, so, zeros are used
			x_ << 	meas_package.raw_measurements_[0],
					meas_package.raw_measurements_[1],
					0,
					0,
					0;
		}


		PrevTimeStamp = meas_package.timestamp_;

		// done initializing, no need to predict or update
		is_initialized_ = true;
		//return;

	}

	delta_t = (CurrentTimeStamp - PrevTimeStamp) / 1000000.0; // convert micros to s

	//Prediction------------------------------------------
	Prediction(delta_t);


	 // Update-------------------
	if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
		// cout << "Radar " << meas_package.raw_measurements_[0] << " " << meas_package.raw_measurements_[1] << endl;
	     UpdateRadar(meas_package);
	}

	if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
		// cout << "Lidar " << meas_package.raw_measurements_[0] << " " << meas_package.raw_measurements_[1] << endl;
	     UpdateLidar(meas_package);
	}



	PrevTimeStamp = CurrentTimeStamp;

	cout << "x_ = " << x_ << endl;
	cout << "P_ = " << P_ << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

	//create augmented mean vector
	VectorXd x_aug = VectorXd(7);

	//create augmented state covariance
	MatrixXd P_aug = MatrixXd(7, 7);

	//create sigma point matrix
	MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

	//create augmented mean state
	x_aug.fill(0.0);
	x_aug.head(5) = x_;
	x_aug(5) = 0;
	x_aug(6) = 0;

	//create augmented covariance matrix
	P_aug.fill(0.0);
	P_aug.topLeftCorner(5,5) = P_;
	P_aug(5,5) = std_a_*std_a_;
	P_aug(6,6) = std_yawdd_*std_yawdd_;

	//create square root matrix
	MatrixXd L = P_aug.llt().matrixL();

	//create augmented sigma points
	Xsig_aug.col(0)  = x_aug;
	for (int i = 0; i < n_aug_; i++)
	{
	  Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
	  Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
	}


	//--------------------------------------------------------------------------

	//predict sigma points
	for (int i = 0; i< 2*n_aug_+1; i++)
	{
	    //extract values for better readability
	    double p_x = Xsig_aug(0,i);
	    double p_y = Xsig_aug(1,i);
	    double v = Xsig_aug(2,i);
	    double yaw = Xsig_aug(3,i);
	    double yawd = Xsig_aug(4,i);
	    double nu_a = Xsig_aug(5,i);
	    double nu_yawdd = Xsig_aug(6,i);

	    //predicted state values
	    double px_p, py_p;

	    //avoid division by zero
	    if (fabs(yawd) > 0.001) {
	        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
	        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
	    }
	    else {
	        px_p = p_x + v*delta_t*cos(yaw);
	        py_p = p_y + v*delta_t*sin(yaw);
	    }

	    double v_p = v;
	    double yaw_p = yaw + yawd*delta_t;
	    double yawd_p = yawd;

	    //add noise
	    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
	    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
	    v_p = v_p + nu_a*delta_t;

	    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
	    yawd_p = yawd_p + nu_yawdd*delta_t;

	    //write predicted sigma point into right column
	    Xsig_pred(0,i) = px_p;
	    Xsig_pred(1,i) = py_p;
	    Xsig_pred(2,i) = v_p;
	    Xsig_pred(3,i) = yaw_p;
	    Xsig_pred(4,i) = yawd_p;
	}



	//------------------------------------------------------------------------

	//predicted state mean
	x_.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
	   x_ = x_ +  weights(i) * Xsig_pred.col(i);
	}


	//predicted state covariance matrix
	P_.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points


		// state difference
		VectorXd x_diff = Xsig_pred.col(i) - x_;
		//angle normalization
		while (x_diff(3)> PI) x_diff(3) -= 2.*PI;
		while (x_diff(3)<-PI) x_diff(3) += 2.*PI;

		P_ = P_ + weights(i) * x_diff * x_diff.transpose() ;

		//cout << "Inside P_ " << endl;

	}


}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

	double p_x;
	double p_y;
	double v;
	double yaw;
	double v1;
	double v2;

	//set measurement dimension, Lidar can measure Px, Py
	int n_z = 2;


  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

	//create matrix for sigma points in measurement space
	  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
	//transform sigma points into measurement space
	  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

		// extract values for better readibility
		p_x = Xsig_pred(0,i);
		p_y = Xsig_pred(1,i);

		// measurement model
		Zsig(0,i) = p_x;                       //Px
		Zsig(1,i) = p_y;                       //Py

	  }

	  //mean predicted measurement
	  VectorXd z_pred = VectorXd(n_z);
	  z_pred.fill(0.0);
	  for (int i = 0; i < 2*n_aug_ + 1; i++) {
		  z_pred = z_pred + weights(i) * Zsig.col(i);
	  }

	  //innovation covariance matrix S
	  MatrixXd S = MatrixXd(n_z,n_z);
	  S.fill(0.0);
	  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
		//residual
		VectorXd z_diff = Zsig.col(i) - z_pred;

		//angle normalization
		while (z_diff(1)> PI) z_diff(1)-=2.*PI;
		while (z_diff(1)<-PI) z_diff(1)+=2.*PI;

		S = S + weights(i) * z_diff * z_diff.transpose();
	  }

	  //add measurement noise covariance matrix


	  S = S + R_lidar_;


	  //------------------------------------------------------------------------------------

	  //create matrix for cross correlation Tc
	  MatrixXd Tc = MatrixXd(n_x_, n_z);

	  // Measurements
	  VectorXd z = meas_package.raw_measurements_;

	  //calculate cross correlation matrix
	  Tc.fill(0.0);

	  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 sigma points

	   //residual
	   VectorXd z_diff = Zsig.col(i) - z_pred;
	   //angle normalization
	   while (z_diff(1)> PI) z_diff(1)-=2.*PI;
	   while (z_diff(1)<-PI) z_diff(1)+=2.*PI;

	   // state difference
	   VectorXd x_diff = Xsig_pred.col(i) - x_;
	   //angle normalization
	   while (x_diff(3)> PI) x_diff(3)-=2.*PI;
	   while (x_diff(3)<-PI) x_diff(3)+=2.*PI;

	   Tc = Tc + weights(i) * x_diff * z_diff.transpose();
	 }

	 //Kalman gain K;
	 MatrixXd K = Tc * S.inverse();

	 //residual
	 VectorXd z_diff = z - z_pred;

	 //angle normalization
	 while (z_diff(1)> PI) z_diff(1)-=2.*PI;
	 while (z_diff(1)<-PI) z_diff(1)+=2.*PI;

	 //update state mean and covariance matrix
	 x_ = x_ + K * z_diff;
	 P_ = P_ - K*S*K.transpose();

	 NIS_lidar_ = z.transpose() * S.inverse() * z;


}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {

	double p_x;
	double p_y;
	double v;
	double yaw;
	double v1;
	double v2;

	//set measurement dimension, radar can measure r, phi, and r_dot
	int n_z = 3;


  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

	//create matrix for sigma points in measurement space
	  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
	//transform sigma points into measurement space
	  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 sigma points

	    // extract values for better readibility
	    p_x = Xsig_pred(0,i);
	    p_y = Xsig_pred(1,i);
	    v  = Xsig_pred(2,i);
	    yaw = Xsig_pred(3,i);

	    v1 = cos(yaw)*v;
	    v2 = sin(yaw)*v;

	    // measurement model
	    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
	    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
	    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
	  }

	  //mean predicted measurement
	  VectorXd z_pred = VectorXd(n_z);
	  z_pred.fill(0.0);
	  for (int i = 0; i < 2*n_aug_ + 1; i++) {
	      z_pred = z_pred + weights(i) * Zsig.col(i);
	  }

	  //innovation covariance matrix S
	  MatrixXd S = MatrixXd(n_z,n_z);
	  S.fill(0.0);
	  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 sigma points
	    //residual
	    VectorXd z_diff = Zsig.col(i) - z_pred;

	    //angle normalization
	    while (z_diff(1)> PI) z_diff(1)-=2.*PI;
	    while (z_diff(1)<-PI) z_diff(1)+=2.*PI;

	    S = S + weights(i) * z_diff * z_diff.transpose();
	  }

	  //add measurement noise covariance matrix


	  S = S + R_radar_;



	  //------------------------------------------------------------------------------------

	  //create matrix for cross correlation Tc
	  MatrixXd Tc = MatrixXd(n_x_, n_z);

	  // Measurements
	  VectorXd z = meas_package.raw_measurements_;

	  //calculate cross correlation matrix
	  Tc.fill(0.0);

	  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 sigma points

	   //residual
	   VectorXd z_diff = Zsig.col(i) - z_pred;
	   //angle normalization
	   while (z_diff(1)> PI) z_diff(1)-=2.*PI;
	   while (z_diff(1)<-PI) z_diff(1)+=2.*PI;

	   // state difference
	   VectorXd x_diff = Xsig_pred.col(i) - x_;
	   //angle normalization
	   while (x_diff(3)> PI) x_diff(3)-=2.*PI;
	   while (x_diff(3)<-PI) x_diff(3)+=2.*PI;

	   Tc = Tc + weights(i) * x_diff * z_diff.transpose();
	 }

	 //Kalman gain K;
	 MatrixXd K = Tc * S.inverse();

	 //residual
	 VectorXd z_diff = z - z_pred;

	 //angle normalization
	 while (z_diff(1)> PI) z_diff(1)-=2.*PI;
	 while (z_diff(1)<-PI) z_diff(1)+=2.*PI;

	 //update state mean and covariance matrix
	 x_ = x_ + K * z_diff;
	 P_ = P_ - K*S*K.transpose();

	 NIS_radar_ = z.transpose() * S.inverse() * z;



}



