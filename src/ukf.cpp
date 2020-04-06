#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
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
    std_a_ = 30;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 30;

    /**
     * DO NOT MODIFY measurement noise values below.
     * These are provided by the sensor manufacturer.
     */

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

    /**
     * End DO NOT MODIFY section for measurement noise values
     */

    /**
     * TODO: Complete the initialization. See ukf.h for other member properties.
     * Hint: one or more values initialized above might be wildly off...
     */
    // 30 was wildly off
    n_x_ = 5;
    n_aug_ = 7;
    lambda_ = 3 - n_aug_;
    std_a_ = 1.0;
    std_yawdd_ = 0.5;
    // prediction
    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
    // weights of sigma points
    weights_ = VectorXd(2 * n_aug_ + 1);


}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    /**
     * TODO: Complete this function! Make sure you switch between lidar and radar
     * measurements.
     */
    if (!is_initialized_) {
        P_ = 0.5 * MatrixXd::Identity(5, 5);
        P_(3,3) = std_laspx_*std_laspy_;
        P_(4,4) = std_laspx_*std_laspy_;

        // set the state with the initial location and zero velocity
        x_ << meas_package.raw_measurements_[0],
                meas_package.raw_measurements_[1], 0, 0, 0;

        time_us_ = meas_package.timestamp_;

        if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
            double rho = meas_package.raw_measurements_[0];
            double phi = meas_package.raw_measurements_[1];
            double rho_dot = meas_package.raw_measurements_[2];
            x_ <<   rho * cos(phi),
                    rho * sin(phi),
                                 0,
                                 0,
                                 0;
        } else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
            x_ <<   meas_package.raw_measurements_[0],
                    meas_package.raw_measurements_[1],
                                                    0,
                                                    0,
                                                    0;
        } else throw ("Unknown sensor type to measure");
        is_initialized_ = true;
        return;
    }

    // compute the time elapsed between the current and previous measurements
    // dt - expressed in seconds
    double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
    time_us_ = meas_package.timestamp_;

    // Prediction Step
    Prediction(dt);

    // Update Step
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) UpdateRadar(meas_package);
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) UpdateLidar(meas_package);
    else throw ("Unknown sensor type to update");
}

void UKF::PredictAugmentedSigmaPoints(const MatrixXd &Xsig_aug, const double delta_t) {
    //predict sigma points
    //N.B. tried keep similarity with the equations in lecture in code for better understanding
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        // current augmented state x_k
        VectorXd x_k(n_x_);
        //process model
        VectorXd P_model(n_x_);
        //process noise
        VectorXd Nu(n_x_);

        // extract values for better readability
        double p_x = Xsig_aug(0,i);
        double p_y = Xsig_aug(1,i);
        double v = Xsig_aug(2,i);
        double yaw = Xsig_aug(3,i);
        double yawd = Xsig_aug(4,i);
        double nu_a = Xsig_aug(5,i);
        double nu_yawdd = Xsig_aug(6,i);

        x_k <<  p_x,
                p_y,
                  v,
                yaw,
               yawd;

        // positional offsets
        double px_offset, py_offset;
        //avoid division by zero
        if (fabs(yawd) > 0.001) {
            px_offset = v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
            py_offset = v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
        } else {
            px_offset = v * delta_t * cos(yaw);
            py_offset = v * delta_t * sin(yaw);
        }
        // Process model
        P_model <<  px_offset,
                    py_offset,
                          0.0,
               yawd * delta_t,
                          0.0;
        //noise
        Nu <<   0.5 * nu_a * delta_t * delta_t * cos(yaw), // horizontal noise
                0.5 * nu_a * delta_t * delta_t * sin(yaw), // vertical noise
                                           nu_a * delta_t, // velocity noise
                       0.5 * nu_yawdd * delta_t * delta_t, // angular noise
                                       nu_yawdd * delta_t; // angular velocity noise

        //write predicted sigma point into right column
        // as summation of state, process and noise vectors as shown in the lesson
        Xsig_pred_.col(i) = x_k + P_model + Nu;
    }
}


void UKF::Prediction(double delta_t) {
    /**
     * TODO: Complete this function! Estimate the object's location.
     * Modify the state vector, x_. Predict sigma points, the state,
     * and the state covariance matrix.
     */
    // generate sigma points
    // create augmented mean vector
    VectorXd x_aug = VectorXd(n_aug_);

    // create augmented state covariance
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

    // create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

    // create augmented mean state
    x_aug << x_, 0, 0;
    P_aug.fill(0.0);
    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    P_aug(n_x_, n_x_) = std_a_ * std_a_;
    P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

    // create square root matrix
    MatrixXd L = P_aug.llt().matrixL();
    MatrixXd d = sqrt(lambda_ + n_aug_) * L;

    // create augmented sigma points
    Xsig_aug.col(0) = x_aug;
    Xsig_aug.block(0, 1, n_aug_, n_aug_) = d.colwise() + x_aug;
    Xsig_aug.block(0, n_aug_ + 1, n_aug_, n_aug_) = (-d).colwise() + x_aug;
    // Predict sigma points
    PredictAugmentedSigmaPoints(Xsig_aug, delta_t);
    // Approximate gaussian distribution mean covariance from predicted sigma points
    weights_.fill(0.5 / (n_aug_ + lambda_));
    weights_(0) = lambda_ / (lambda_ + n_aug_);

    // predicted state mean
    x_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // iterate over sigma points
        x_ = x_ + weights_(i) * Xsig_pred_.col(i);
    }


    P_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //angle normalization
        while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;
        P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
    }
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
    /**
     * TODO: Complete this function! Use lidar data to update the belief
     * about the object's position. Modify the state vector, x_, and
     * covariance, P_.
     * You can also calculate the lidar NIS, if desired.
     */

    // The measurement model for LIDAR is linear, we simply extract p_x and p_y from x_
    MatrixXd H = MatrixXd(2, 5);
    H <<    1, 0, 0, 0, 0,
            0, 1, 0, 0, 0;

    // Extract the measurement z
    const int n_z = 2; // A LASER measurement has only two Cartesian coordinates, p_x and p_y
    VectorXd z = VectorXd(n_z);
    z << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1);

    // Define the measurement noise matrix for LIDAR
    MatrixXd  R = MatrixXd(2, 2);
    R.fill(0.0);
    R << std_laspx_ * std_laspx_,    0,
            0, std_laspy_ * std_laspy_;

    // Update y, S, and K
    VectorXd y = z - H * x_;
    MatrixXd PHt = P_ * H.transpose();
    MatrixXd S = H * PHt + R;
    MatrixXd S_inv = S.inverse();
    MatrixXd K = PHt * S_inv;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H) * P_;

    // Compute the normalized innovation squared (NIS)
    const double nis = y.transpose() * S_inv * y;
//    std::cout << "LIDAR NIS: " << nis << std::endl;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
    /**
     * TODO: Complete this function! Use radar data to update the belief
     * about the object's position. Modify the state vector, x_, and
     * covariance, P_.
     * You can also calculate the radar NIS, if desired.
     */

    // set measurement dimension, radar can measure r, phi, and r_dot
    int n_z = 3;
    VectorXd z = meas_package.raw_measurements_;
    // create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

    // mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);

    // measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z,n_z);

    // transform sigma points into measurement space
    double v1, v2, rho, phi, rho_dot;
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
        // extract values for better readability
        double p_x = Xsig_pred_(0,i);
        double p_y = Xsig_pred_(1,i);
        double v  = Xsig_pred_(2,i);
        double yaw = Xsig_pred_(3,i);

        double v1 = cos(yaw)*v;
        double v2 = sin(yaw)*v;

        rho =  sqrt(p_x*p_x + p_y*p_y);
        phi = atan2(p_y, p_x);
        rho_dot = (p_x*v1 + p_y*v2) / std::max(0.001, rho); // avoid division by zero
        // measurement model
        Zsig(0,i) = rho;
        Zsig(1,i) = phi;
        Zsig(2,i) = rho_dot;
    }

    // mean predicted measurement
    z_pred.fill(0.0);
    for (int i=0; i < 2*n_aug_+1; ++i) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }

    // innovation covariance matrix S
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
        // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        // angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

        S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    // add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z,n_z);
    R <<    std_radr_*std_radr_,    0,  0,
            0, std_radphi_*std_radphi_, 0,
            0, 0,   std_radrd_*std_radrd_;
    S = S + R;

    // create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);

    // calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
        // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        // angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        // angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    // Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    // residual
    VectorXd z_diff = z - z_pred;

    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K*S*K.transpose();


    return;
}


