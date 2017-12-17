#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <iostream>
#include <fstream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:
	/**
	* Constructor
	*/
	UKF();

	/**
	* Destructor
	*/
	virtual ~UKF();

	/**
	* ProcessMeasurement Process Measurement
	* @param measurement_pack The latest measurement data of either radar or laser
	*/
	void ProcessMeasurement(const MeasurementPackage& measurement_pack);

	/**
	* GetState
	*/
	VectorXd GetState() const { return x_; };

private:

	/**
	* AdjustAngle Adjust angle to be in [-pi,+pi] range
    * @param phi Angle
	*/
    double AdjustAngle(double phi);

	/**
	* InitKF Initialize Unscented Kalman Filter
	* @param measurement_pack First measurement
	*/
	void InitKF(const MeasurementPackage &measurement_pack);

	/**
	* GenerateAugmentedSigmaPoints Generate augmented sigma points
	* @param Xsig_aug Returned matrix of augmented sigma points
	*/
	void GenerateAugmentedSigmaPoints(MatrixXd& Xsig_aug);

	/**
	* PredictSigmaPoints Predict sigma points according to CTRV model
	* @param Xsig_aug Matrix of augmented sigma points 
	* @param Xsig_pred Returned matrix of predicted sigma points
	* @param delta_t Prediction time interval
	*/
	void PredictSigmaPoints(const MatrixXd& Xsig_aug, MatrixXd& Xsig_pred, double delta_t);

	/**
	* PredictMeanStateCovariance Predict mean state and covariance
	* @param Xsig_pred Matrix of predicted sigma points 
	* @param x_pred Returned predicted state
	* @param P_pred Returned predicted covariance
	* @param angle_coordinate Offset of angle coordinate in a state vector
	*/
	void PredictMeanStateCovariance(const MatrixXd& Xsig_pred, VectorXd& x_pred, MatrixXd& P_pred, const int angle_coordinate);

	/**
	* Prediction Predicts sigma points, the state, and the state covariance matrix
	* @param delta_t Time between k and k+1 in s
	* @param x_pred Returned predicted state
	* @param P_pred Returned predicted covariance matrix
	* @param Xsig_pred Returned predicted sigma points
	*/
	void Prediction(double delta_t, VectorXd& x_pred, MatrixXd& P_pred, MatrixXd& Xsig_pred);

	/**
	* Update Update predicted state vector and covariance matrix with the new measurement
	* @param measurement_pack New measurement
	* @param x_pred Predicted state vector
	* @param P_pred Predicted covariance matrix
	* @param Xsig_pred Predicted sigma points
	*/
	void Update(const MeasurementPackage& measurement_pack, const VectorXd& x_pred, const MatrixXd& P_pred, const MatrixXd& Xsig_pred);

	/**
	* ProjectToRadarSpace Project sigma points to radar measurement space
	* @param Xsig_pred Matrix of predicted sigma points
	* @param Zsig Matrix of sigma points projected to radar measurement space
	*/
	void ProjectToRadarSpace(const MatrixXd& Xsig_pred, MatrixXd& Zsig);

    // initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_;

    // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    VectorXd x_;

    // state covariance matrix
    MatrixXd P_;

    // predicted sigma points matrix
    MatrixXd Xsig_pred_;

    // time when the state is true, in us
    long long time_us_;

    // Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a_;

    // Process noise standard deviation yaw acceleration in rad/s^2
    double std_yawdd_;

    // Laser measurement noise standard deviation position1 in m
    double std_laspx_;

    // Laser measurement noise standard deviation position2 in m
    double std_laspy_;

    // Radar measurement noise standard deviation radius in m
    double std_radr_;

    // Radar measurement noise standard deviation angle in rad
    double std_radphi_;

    // Radar measurement noise standard deviation radius change in m/s
    double std_radrd_ ;

	// Covariance matrix of radar measurement
    MatrixXd R_radar_;

	// Covariance matrix of laser measurement
    MatrixXd R_laser_;

    // Weights of sigma points
    VectorXd weights_;

    // State dimension
    int n_x_;

    // Augmented state dimension
    int n_aug_;

    // Sigma point spreading parameter
    double lambda_;

    // files to store Normalized Innovation Score values of each sensor
	ofstream radar_nis_file;
	ofstream laser_nis_file;
};

#endif /* UKF_H */
