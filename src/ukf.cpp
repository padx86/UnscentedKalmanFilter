#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 6;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = .8;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  //set is_initialized_ bool
  is_initialized_ = false;		

  //get size of state vector
  n_x_ = x_.size();				

  //Initialize XSig_pred with 2*n_x_+1 sigma points(cols)
  //Xsig_pred_ = MatrixXd(n_x_, 2 * n_x_ + 1);				//Non-augmented sigma points
  Xsig_pred_ = MatrixXd(n_x_, 2 * (n_x_+2) + 1);			//Augmented sigma points

  //Initialize weights with one weight per sigma / augmented sigma point
  weights_ = VectorXd(Xsig_pred_.cols());

  //set lambda design parameter to 3-n_x_
  lambda_ = 3 - n_x_;

  //init nis for lidar and radar
  nis_lidar_ = VectorXd::Zero(2);
  nis_radar_ = VectorXd::Zero(2);

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
	if (!is_initialized_) {
		//Init
		if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
			//Initialize state vector if Lidar Meassurement & Lidar shall be used

			x_ << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), 0.0, 0.0, 0.0; //Use Positions given by Lidar measurement

			is_initialized_ = true;
		}
		else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
			//Initalize state vector if Radar Measurement & Radar shall be used
			
			x_ << meas_package.raw_measurements_(0)*cos(meas_package.raw_measurements_(1)), meas_package.raw_measurements_(0)*sin(meas_package.raw_measurements_(1)), 0, 0, 0; //Calculate Position from first redar measurement
			
			is_initialized_ = true;
		}
		
		if (!is_initialized_) {
			//Return if first measurement is of sensor type not to be used to not waist time initializing other variables
			return;
		}

		//store timestamp to calc elapsed time
		time_us_ = meas_package.timestamp_;	

		//Initialize covariance matrix P_
		P_ = MatrixXd::Identity(n_x_, n_x_);

		std::cout << "Init done" << endl;
	}
	else {
		//Predict & Update		
		if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
			//Run on lidar measurement if desired
			double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;	//Calculate time difference
			time_us_ = meas_package.timestamp_;								//save new timestamp
			Prediction(dt);													//Predict state & covariance
			UpdateLidar(meas_package);										//Update lidar measurement
		}
		else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
			//Run on radar measurement if desired
			double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;	//Calculate	time difference
			time_us_ = meas_package.timestamp_;								//Save new timestamp
			Prediction(dt);													//Predict state & covariance
			UpdateRadar(meas_package);										//Update radar measurement
		}
	}
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

	/////////////////////////////////////
	//  Create augmented sigma points  //
	/////////////////////////////////////
	/*
	//Create sigma points non-augmented
	MatrixXd A = P_.llt().matrixL();	//Create A so A^T*A = P using cholesky decomposition

	Xsig_pred_.col(0) = x_;				//First value of Xsig_pred

	double la_nx_sr = sqrt(lambda_ + n_x_);	//Precalc constant arg

	for (unsigned short i = 0; i < n_x_; i++) {
		VectorXd la_nx_sr_Ai = la_nx_sr*A.col(i);	//Precalc argument for subtraction / addition
		Xsig_pred_.col(i + 1) = x_ + la_nx_sr_Ai;
		Xsig_pred_.col(i + n_x_ + 1) = x_ - la_nx_sr_Ai;
	}
	*/
	
	//Create augmented vectors and matrices
	VectorXd x_aug = VectorXd::Zero(n_x_ + 2);			//Create augmented state vector
	int n_aug = x_aug.size();							//Get augmented size
	MatrixXd P_aug = MatrixXd::Zero(n_aug, n_aug);		//Create augmented covariance matrix
	MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);	//Create augmented sigma points matrix
	
	//Augment state vector
	x_aug.head(n_x_) = x_;		//set first five elements to statevector x_
	x_aug(n_x_) = 0;		//Set element 6 to 0
	x_aug(n_x_ + 1) = 0;	//Set element 7 to 0

	//Augment covariance matrix P_aug
	P_aug.topLeftCorner(n_x_, n_x_) = P_;
	P_aug(n_x_, n_x_) = std_a_*std_a_;
	P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_*std_yawdd_;
	
	// Create A so A^T*A = P_aug using cholesky decomposition
	MatrixXd A = P_aug.llt().matrixL();
	
	// Fill Xsig_aug
	Xsig_aug.col(0) = x_aug;								//First value of Xsig_aug


	int lambda_aug = 3 - n_aug;								//calc new lambda for augmented sigma point creation
	double la_nx_sr = sqrt(lambda_aug + n_aug);				//Precalc constant arg

	VectorXd la_nx_sr_Ai = VectorXd::Zero(n_aug);
	for (unsigned short i = 0; i < n_aug; i++) {
		la_nx_sr_Ai = la_nx_sr*A.col(i);			//Precalc argument for subtraction / addition
		Xsig_aug.col(i + 1) = x_aug + la_nx_sr_Ai;			//Create sigma points 1
		Xsig_aug.col(i + n_aug + 1) = x_aug - la_nx_sr_Ai;	//Create sigma points 2

		//Normalize angles
		while (Xsig_aug(3, i) > M_PI) {
			Xsig_aug(3, i) -= 2.0*M_PI;
		}
		while (Xsig_aug(3, i) < -M_PI) {
			Xsig_aug(3, i) += 2.0*M_PI;
		}
		while (Xsig_aug(3, i + n_aug + 1) > M_PI) {
			Xsig_aug(3, i + n_aug + 1) -= 2.0*M_PI;
		}
		while (Xsig_aug(3, i + n_aug + 1) < -M_PI) {
			Xsig_aug(3, i + n_aug + 1) += 2.0*M_PI;
		}
	}

	////////////////////////////
	//  Perdict sigma points  //
	////////////////////////////
	double dtshalf = 0.5*delta_t*delta_t;

	for (unsigned short i = 0; i < 2 * n_aug + 1; i++) {
		//Extract sigma point at position i
		double px_sig = Xsig_aug(0, i);
		double py_sig = Xsig_aug(1, i);
		double v_sig = Xsig_aug(2, i);
		double yaw_sig = Xsig_aug(3, i);
		double yawd_sig = Xsig_aug(4, i);
		double std_a_sig = Xsig_aug(5, i);
		double std_yawdd_sig = Xsig_aug(6, i);

		//Initialize and calculate sigma point prediction
		double px = px_sig;
		double py = py_sig;
		
		if (yawd_sig < 0.00001) {
			//yawrate near zero --> use different set of equations for px,py
			px = px + v_sig*cos(yaw_sig)*delta_t;	//predicted x-position if yaw_rate near 0
			py = py + v_sig*sin(yaw_sig)*delta_t;	//predicted y-position if yaw_rate near 0
		}
		else {
			px = px + v_sig / yawd_sig*(sin(yaw_sig + yawd_sig*delta_t) - sin(yaw_sig));		//predicted x-position if yaw_rate NOT near 0
			py = py + v_sig / yawd_sig*(-cos(yaw_sig + yawd_sig*delta_t) + cos(yaw_sig));	//predicted x-position if yaw_rate NOT near 0
		}
		px = px + dtshalf*cos(yaw_sig)*std_a_sig;	//Add noise to px	
		py = py + dtshalf*sin(yaw_sig)*std_a_sig;	//Add noise to py

		double v = v_sig + 0 + delta_t*std_a_sig;						//predicted velocity (+noise)
		double yaw = yaw_sig + yawd_sig*delta_t + dtshalf*std_yawdd_sig;	//predicted yaw_angle (+noise)
		//Normalize angles
		while (yaw > M_PI) {
			yaw -= 2.0*M_PI;
		}
		while (yaw < -M_PI) {
			yaw += 2.0*M_PI;
		}
		double yawd = yawd_sig + 0 + std_yawdd_sig*delta_t;				//predicted yaw_rate (+noise)
		
		Xsig_pred_.col(i) <<px, py, v, yaw, yawd;		//Fill predicted sigma point matrix
	}

	///////////////////////////////////
	//  Predict mean and covariance  //
	///////////////////////////////////

	//Fill weights
	weights_(0) = lambda_aug / (lambda_aug + n_aug);
	for (unsigned short i = 1; i < weights_.size(); i++) {
		weights_(i) = 0.5 / (n_aug + lambda_aug);
	}
	x_ << 0.0, 0.0, 0.0, 0.0, 0.0;

	//Predict mean
	for (unsigned short i = 0; i < weights_.size(); i++) {
		x_ += weights_(i)*Xsig_pred_.col(i);
	}
	while (x_(3) > M_PI) {
		x_(3) -= 2.0*M_PI;
	}
	while (x_(3) < -M_PI) {
		x_(3) += 2.0*M_PI;
	}

	//Predict covariance
	P_.fill(0.0);
	for (unsigned short i = 0; i < weights_.size(); i++) {
		VectorXd dx = Xsig_pred_.col(i) - x_;
		//Normalize angles
		while (dx(3) > M_PI) {
			dx(3) -= 2.0*M_PI;
		}
		while (dx(3) < -M_PI) {
			dx(3) += 2.0*M_PI;
		}
		//Fill P Matrix
		P_ += weights_(i)*dx*dx.transpose();
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
	VectorXd z = meas_package.raw_measurements_;

	int n_z = z.size();

	VectorXd z_trans = VectorXd(n_z);
	MatrixXd Zsig = MatrixXd::Zero(n_z, Xsig_pred_.cols());

	///////////////////////////////////////////////////
	//  Transform Sigma Points to measurement space  //
	///////////////////////////////////////////////////
	for (unsigned short i = 0; i < Xsig_pred_.cols(); i++) {
		VectorXd x_sig = Xsig_pred_.col(i);
		Zsig(0, i) = x_sig(0);
		Zsig(1, i) = x_sig(1);
	}
	///////////////////////////////////////////////
	//  Predict measurement mean and covariance  //
	///////////////////////////////////////////////
	
	//Predict mean
	VectorXd z_pred = VectorXd::Zero(n_z);
	for (unsigned short i = 0; i < Zsig.cols(); i++) {
		z_pred += weights_(i)*Zsig.col(i);
	}
	//Predict covariance matrix S
	MatrixXd S = MatrixXd::Zero(n_z, n_z);		//Initialize covariance matrix S
	VectorXd dz = VectorXd::Zero(n_z);			//Initialize vector dz;

	for (unsigned short i = 0; i < weights_.size(); i++) {
		dz = Zsig.col(i) - z_pred;
		S += weights_(i)*dz*dz.transpose(); //Fill covariance matrix
	}

	//Create Lidar Noise
	MatrixXd R = MatrixXd(n_z, n_z);
	R << std_laspx_*std_laspx_, 0,
		0, std_laspy_*std_laspy_;

	S = S + R;
	//////////////////////////////////////////
	//  Update state and covariance matrix  //
	//////////////////////////////////////////

	//Calculate crosscorelation matrix
	MatrixXd T = MatrixXd::Zero(n_x_, n_z);
	VectorXd dx = VectorXd::Zero(x_.size());

	for (unsigned short i = 0; i < weights_.size(); i++) {
		dz = Zsig.col(i) - z_pred;
		dx = Xsig_pred_.col(i) - x_;
		//Normalize angles
		while (dx(3) > M_PI) {
			dx(3) -= 2.0*M_PI;
		}
		while (dx(3) < -M_PI) {
			dx(3) += 2.0*M_PI;
		}
		T += weights_(i)*dx*dz.transpose();
	}

	//Calculate S_inv
	MatrixXd Si = S.inverse();

	//calculate Kalman Gain K
	MatrixXd K = T*Si;

	//update state
	dz = z - z_pred;
	x_ = x_ + K*dz;
	//Normalize angles
	while (x_(3) > M_PI) {
		x_(3) -= 2.0*M_PI;
	}
	while (x_(3) < -M_PI) {
		x_(3) += 2.0*M_PI;
	}

	//Update covariance matrix
	P_ = P_ - K*S*K.transpose();

	//Calculate NIS
	double nis = dz.transpose()*Si*dz;
	
	// Calculate and output Percent above X.050
	++nis_lidar_(0);
	if (nis > 5.991) {
		++nis_lidar_(1);
	}
	std::cout << "LIDAR - Percent above X.050 NIS after frame " << nis_lidar_(0) << " = " << nis_lidar_(1)*100 / (nis_lidar_(0)) << endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
	VectorXd z = meas_package.raw_measurements_;
	/////////////////////////////////////////////////
	//  Transform Xsig_pred_ to Measurement Space  //
	/////////////////////////////////////////////////
	int n_z = z.size();
	VectorXd z_trans = VectorXd(n_z);
	MatrixXd Zsig = MatrixXd::Zero(n_z,Xsig_pred_.cols());
	
	for (unsigned short i = 0; i < Xsig_pred_.cols(); i++) {
		VectorXd x_sig = Xsig_pred_.col(i);
		double pxys = x_sig(0)*x_sig(0) + x_sig(1)*x_sig(1);	//calc x^2+y^2
		if (pxys < 0.000001) {
			z_trans(0) = 0.0; //rho
			z_trans(2) = 0.0; //droh
		}
		else {
			pxys = sqrt(pxys);	//calc sqrt(px^2+py^2)
			z_trans(0) = pxys;	//rho
			z_trans(2) = (x_sig(0)*cos(x_sig(3))*x_sig(2) + x_sig(1)*sin(x_sig(3))*x_sig(2)) / pxys;	//droh
		}
		z_trans(1) = atan2(x_sig(1),x_sig(0));
		Zsig.col(i) = z_trans;
	}

	///////////////////////////////////////////////
	//  Predict measurement mean and covariance  //
	///////////////////////////////////////////////
	//Predict mean measurement
	VectorXd z_pred = VectorXd::Zero(n_z);
	for (unsigned short i = 0; i < weights_.size(); i++) {
		z_pred += weights_(i)*Zsig.col(i);
	}
	//Normalize angles
	while (z_pred(1) > M_PI) {
		z_pred(1) -= 2.0*M_PI;
	}
	while (z_pred(1) < -M_PI) {
		z_pred(1) += 2.0*M_PI;
	}
	//Predict covariance matrix S
	MatrixXd S = MatrixXd::Zero(n_z, n_z);		//Initialize covariance matrix S
	VectorXd dz = VectorXd::Zero(n_z);			//Initialize vector dz;
	
	for (unsigned short i = 0; i < weights_.size(); i++) {
		dz = Zsig.col(i) - z_pred;
		//Normalize angles
		while (dz(1) > M_PI) {
			dz(1) -= 2.0*M_PI;
		}
		while (dz(1) < -M_PI) {
			dz(1) += 2.0*M_PI;
		}
		S += weights_(i)*dz*dz.transpose(); //Fill covariance matrix
	}
	
	//Create Radar measurement noise matrix
	MatrixXd R = MatrixXd(n_z, n_z);
	R << std_radr_*std_radr_, 0, 0,
		0, std_radphi_*std_radphi_, 0,
		0, 0, std_radrd_*std_radrd_;

	S = S + R;
	//////////////////////////////////////////
	//  Update state and covariance matrix  //
	//////////////////////////////////////////

	//Calculate crosscorelation matrix
	MatrixXd T = MatrixXd::Zero(n_x_, n_z);
	VectorXd dx = VectorXd::Zero(x_.size());

	for (unsigned short i = 0; i < weights_.size(); i++) {
		dz = Zsig.col(i) - z_pred;
		dx = Xsig_pred_.col(i) - x_;
		//Normalize angles
		while (dx(3) > M_PI) {
			dx(3) -= 2.0*M_PI;
		}
		while (dx(3) < -M_PI) {
			dx(3) += 2.0*M_PI;
		}
		while (dz(1) > M_PI) {
			dz(1) -= 2.0*M_PI;
		}
		while (dz(1) < -M_PI) {
			dz(1) += 2.0*M_PI;
		}
		T += weights_(i)*dx*dz.transpose();
	}
	
	//Calculate S_inv
	MatrixXd Si = S.inverse();

	//calculate Kalman Gain K
	MatrixXd K = T*Si;
	//std::cout << K;

	//update state
	dz = z - z_pred;
	while (dz(1) > M_PI) {
		dz(1) -= 2.0*M_PI;
	}
	while (dz(1) < -M_PI) {
		dz(1) += 2.0*M_PI;
	}
	x_ = x_ + K*dz;
	
	while (x_(3) > M_PI) {
		x_(3) -= 2.0*M_PI;
	}
	while (x_(3) < -M_PI) {
		x_(3) += 2.0*M_PI;
	}

	//Update covariance matrix
	P_ = P_ - K*S*K.transpose();

	//Calculate NIS
	double nis = dz.transpose()*Si*dz;
	++nis_radar_(0);
	if (nis > 7.815) {
		++nis_radar_(1);
	}
	std::cout << "RADAR - Percent above X.050 NIS after frame " << nis_radar_(0) << " = " << nis_radar_(1)*100 / (nis_radar_(0)) << endl;
}
