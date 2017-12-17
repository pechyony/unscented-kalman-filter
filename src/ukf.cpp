#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  
  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.3; 

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = M_PI / 16; 
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.

  R_laser_ = MatrixXd(2, 2);
 
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Laser measurement covariance matrix
  R_laser_ << std_laspx_ * std_laspx_, 0,
	          0, std_laspy_ * std_laspy_;

  R_radar_ = MatrixXd(3, 3);

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  // Radar measurement covariance matrix
  R_radar_ << std_radr_ * std_radr_, 0, 0,
	          0, std_radphi_ * std_radphi_, 0,
	          0, 0, std_radrd_ * std_radrd_;

  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  is_initialized_ = false;
  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_aug_;

  //set vector for weights
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < 2 * n_aug_ + 1; i++) {
	  weights_(i) = 0.5 / (n_aug_ + lambda_);
  }

  radar_nis_file.open("radar_nis_file.txt");
  laser_nis_file.open("laser_nis_file.txt");
}

/**
* Destructor
*/
UKF::~UKF() {
	radar_nis_file.close();
	laser_nis_file.close();
}

/**
* Normalize angle to be in the range [-pi,pi]
* @param phi Measurement angle
*/
double UKF::AdjustAngle(double phi) {
	double adjusted_phi;

	adjusted_phi = phi;
	while (adjusted_phi > M_PI)
		adjusted_phi -= 2 * M_PI;
	while (adjusted_phi < -M_PI) 
		adjusted_phi += 2 * M_PI;

	return adjusted_phi;
}

/**
* Initialize Kalman filter with the first measurement, create the covariance matrix
* @param measurement_pack First measurement
*/
void UKF::InitKF(const MeasurementPackage &measurement_pack) {
	time_us_ = measurement_pack.timestamp_;

	P_.fill(0);
	P_(2, 2) = 1;  // max velocity 3 m/s
	P_(3, 3) = pow(M_PI,2) / 4;  // yaw can be between -pi and pi   
	P_(4, 4) = pow(M_PI, 2) / 64; // max yaw rate could be pi/8

	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

		// normalize angle to be in [-pi,pi] range
		double phi = AdjustAngle(measurement_pack.raw_measurements_(1));

		// convert radar measurement from polar to cartesian coordinates 
		x_ << measurement_pack.raw_measurements_(0) * cos(phi),
			  measurement_pack.raw_measurements_(0) * sin(phi),
			  0, 0, 0;

		P_(0, 0) = R_radar_(0, 0) + R_radar_(1, 1);
		P_(1, 1) = R_radar_(0, 0) + R_radar_(1, 1);
	}
	else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {

		// initialize state and covariance matrix
		x_ << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1), 0, 0, 0;

		P_(0, 0) = R_laser_(0, 0);
		P_(1, 1) = R_laser_(1, 1);
	}

	// done initializing, no need to predict or update
	is_initialized_ = true;
}

/**
* ProcessMeasurement Process Measurement
* @param measurement_pack The latest measurement data of either radar or laser
*/
void UKF::ProcessMeasurement(const MeasurementPackage& measurement_pack) {
	
	// Initialization
	if (!is_initialized_) {
		InitKF(measurement_pack);
		return;
	}

	// Predict
	VectorXd x_pred = VectorXd(n_x_);
	MatrixXd P_pred = MatrixXd(n_x_, n_x_);
	MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);
	Prediction((measurement_pack.timestamp_ - time_us_)/1000000.0, x_pred, P_pred, Xsig_pred);

	// Update
	Update(measurement_pack, x_pred, P_pred, Xsig_pred);

	time_us_ = measurement_pack.timestamp_;
}

/**
* GenerateAugmentedSigmaPoints Generate augmented sigma points
* @param Xsig_aug Returned matrix of augmented sigma points
*/
void UKF::GenerateAugmentedSigmaPoints(MatrixXd& Xsig_aug) {

	//create augmented mean vector
	VectorXd x_aug = VectorXd(n_aug_);

	//create augmented state covariance
	MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

	//create augmented mean state
	x_aug.head(n_x_) = x_;
	x_aug(n_x_) = 0;
	x_aug(n_x_ + 1) = 0;

	//create augmented covariance matrix
	P_aug.fill(0);
	P_aug.topLeftCorner(n_x_, n_x_) = P_;
	P_aug(n_x_, n_x_) = std_a_ * std_a_;
	P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

	//create square root matrix
	MatrixXd A = P_aug.llt().matrixL();

	//create augmented sigma points
	for (int i = 0; i < 2 * n_aug_ + 1; i++)
		Xsig_aug.col(i) = x_aug;

	MatrixXd delta = sqrt(lambda_ + n_aug_) * A;
	Xsig_aug.block(0, 1, n_aug_, n_aug_) += delta;
	Xsig_aug.block(0, n_aug_ + 1, n_aug_, n_aug_) -= delta;
}

/**
* PredictSigmaPoints Predict sigma points according to CTRV model
* @param Xsig_aug Matrix of augmented sigma points
* @param Xsig_pred Returned matrix of predicted sigma points
* @param delta_t Prediction time interval
*/
void UKF::PredictSigmaPoints(const MatrixXd& Xsig_aug, MatrixXd& Xsig_pred, double delta_t) {
	double v, phi, phi_dot, nu_a, nu_phi_dot2;
	VectorXd x(n_x_), a(n_x_), b(n_x_);

	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		x = Xsig_aug.col(i);
		v = x(2);
		phi = x(3);
		phi_dot = x(4);
		nu_a = x(5);
		nu_phi_dot2 = x(6);

		// create vector a
		a.fill(0);
		a(3) = phi_dot * delta_t;
		if (fabs(phi_dot) > 0.00001) {
			a(0) = v * (sin(phi + phi_dot*delta_t) - sin(phi)) / phi_dot;
			a(1) = v * (-cos(phi + phi_dot*delta_t) + cos(phi)) / phi_dot;
		}
		else {
			a(0) = v * cos(phi) * delta_t;
			a(1) = v * sin(phi) * delta_t;
		}

		// create vector b
		b(0) = delta_t*delta_t*cos(phi)*nu_a / 2;
		b(1) = delta_t*delta_t*sin(phi)*nu_a / 2;
		b(2) = delta_t*nu_a;
		b(3) = delta_t*delta_t*nu_phi_dot2 / 2;
		b(4) = delta_t*nu_phi_dot2;

		Xsig_pred.col(i) = x.head(n_x_) + a + b;
	}
}

/**
* PredictMeanStateCovariance Predict mean state and covariance
* @param Xsig_pred Matrix of predicted sigma points
* @param x_pred Returned predicted state
* @param P_pred Returned predicted covariance
* @param angle_coordinate Offset of angle coordinate in a state vector
*/
void UKF::PredictMeanStateCovariance(const MatrixXd& Xsig_pred, VectorXd& x_pred, MatrixXd& P_pred, const int angle_coordinate = -1) {

	//predict state mean
	x_pred.fill(0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++)
		x_pred += weights_(i) * Xsig_pred.col(i);
	if (angle_coordinate >=0 )
	    x_pred(angle_coordinate) = AdjustAngle(x_pred(angle_coordinate));

	//predict state covariance matrix
	VectorXd diff(n_x_);
	P_pred.fill(0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		diff = Xsig_pred.col(i) - x_pred;
		if (angle_coordinate >= 0)
		    diff(angle_coordinate) = AdjustAngle(diff(angle_coordinate));
		P_pred += weights_(i) * (diff * diff.transpose());
	}
}

/**
* Prediction Predicts sigma points, the state, and the state covariance matrix
* @param delta_t Time between k and k+1 in s
* @param x_pred Returned predicted state
* @param P_pred Returned predicted covariance matrix
* @param Xsig_pred Returned predicted sigma points
*/
void UKF::Prediction(double delta_t, VectorXd& x_pred, MatrixXd& P_pred, MatrixXd& Xsig_pred) {

    // Generate augmented sigma points
	MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);                                                                                                                                                                                                                                                                             
	GenerateAugmentedSigmaPoints(Xsig_aug);

	// Predict sigma points
	PredictSigmaPoints(Xsig_aug, Xsig_pred, delta_t);

	// Compute predicted mean state and convariance
	PredictMeanStateCovariance(Xsig_pred, x_pred, P_pred, 3);
}

/**
* Update Update predicted state vector and covariance matrix with the new measurement
* @param measurement_pack New measurement
* @param x_pred Predicted state vector
* @param P_pred Predicted covariance matrix
* @param Xsig_pred Predicted sigma points
*/
void UKF::Update(const MeasurementPackage& measurement_pack, const VectorXd& x_pred, const MatrixXd& P_pred, const MatrixXd& Xsig_pred) {

	// project sigma points into measurement space
	MatrixXd Zsig, R, S;
	VectorXd z, z_pred;
	int n_z, angle_coordinate=-1;

	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
		// project sigma points to radar space
		n_z = 3;
		Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
		ProjectToRadarSpace(Xsig_pred, Zsig);
		R = R_radar_;
		angle_coordinate = 1;
	} 
	else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
		// project sigma points to laser space
		n_z = 2;
		Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
		Zsig = Xsig_pred.topLeftCorner(n_z, 2 * n_aug_ + 1);
		R = R_laser_;
	}

	// compute predicted measurement mean and convariance
	z_pred = VectorXd(n_z);
	S = MatrixXd(n_z, n_z);
	PredictMeanStateCovariance(Zsig, z_pred, S, angle_coordinate);
	S += R;

	// update state and covariance matrix

	// compute cross-correlation matrix Tc
	MatrixXd Tc = MatrixXd(n_x_, n_z);
	VectorXd diff_x(n_x_);
	VectorXd diff_z(n_z);

	Tc.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		diff_x = Xsig_pred.col(i) - x_pred;
		diff_z = Zsig.col(i) - z_pred;
		if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
			diff_x(3) = AdjustAngle(diff_x(3));
			diff_z(1) = AdjustAngle(diff_z(1));
		}
		Tc += weights_(i) * diff_x * diff_z.transpose();
	}

	//calculate Kalman gain K;
	MatrixXd K = Tc * S.inverse();

	//update state mean and covariance matrix
	diff_z = measurement_pack.raw_measurements_ - z_pred;
	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
	    diff_z(1) = AdjustAngle(diff_z(1));
	x_ = x_pred + K * diff_z;
	x_(3) = AdjustAngle(x_(3));
	P_ = P_pred - K * S * K.transpose();

	// compute Normalized Innovation Score (NIS)
	double nis = diff_z.transpose() * S.inverse() * diff_z;

	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
		radar_nis_file << nis << std::endl;

	if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) 
		laser_nis_file << nis << std::endl;
}

/**
* ProjectToRadarSpace Project sigma points to radar measurement space
* @param Xsig_pred Matrix of predicted sigma points
* @param Zsig Matrix of sigma points projected to radar measurement space
*/
void UKF::ProjectToRadarSpace(const MatrixXd& Xsig_pred, MatrixXd& Zsig) {

	//transform sigma points into radar measurement space
	double rho, phi, rho_dot, px, py, psi, v;
	for (int i = 0; i< 2 * n_aug_ + 1; i++) {
		px = Xsig_pred(0, i);
		py = Xsig_pred(1, i);
		v = Xsig_pred(2, i);
		psi = Xsig_pred(3, i);

		rho = sqrt(pow(px, 2) + pow(py, 2));
		phi = atan2(py, px);
		rho_dot = (px * cos(psi) * v + py * sin(psi) * v) / rho;

		Zsig(0, i) = rho;
		Zsig(1, i) = phi;
		Zsig(2, i) = rho_dot;
	}
}

