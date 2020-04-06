#include <iostream>
#include "ukf.h"
#include <Eigen/Core>

using Eigen::MatrixXd;
using Eigen::VectorXd;

UKF::UKF() {
    Init();
}

UKF::~UKF() {

}

void UKF::Init() {

}



/**
 * Programming assignment functions:
 */

void UKF::SigmaPointPrediction(MatrixXd* Xsig_out) {

    // set state dimension
    int n_x = 5;

    // set augmented dimension
    int n_aug = 7;

    // create example sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);
    Xsig_aug <<
             5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
            1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,   1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
            2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,   2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
            0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,   0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
            0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528,  0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
            0,        0,        0,        0,        0,        0,  0.34641,        0,         0,        0,        0,        0,        0, -0.34641,        0,
            0,        0,        0,        0,        0,        0,        0,  0.34641,         0,        0,        0,        0,        0,        0, -0.34641;

    // create matrix with predicted sigma points as columns
    MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);

    double delta_t = 0.1; // time diff in sec

    /**
     * Student part begin
     */

    MatrixXd P_model = MatrixXd();
    VectorXd V(n_aug);
    V << 1.0,2.0,30.0,4.0,5.0,6.0,7.0;
//    auto W = Xsig_aug.NullaryExpr(n_aug, [](int i, double j){
//        std::cout<<i<<std::endl;
//    });
//    std::cout << "W" << std::endl << W <<std::endl;
//    std::cout << "X-sig_aug" << std::endl << Xsig_aug <<std::endl;
    // predict sigma points

    
    for (int i = 0; i< 2*n_aug+1; ++i) {
        // extract values for better readability
        //extract values for better readability
        double v = Xsig_aug(2, i);
        double yaw = Xsig_aug(3, i);
        double yawd = Xsig_aug(4, i);
        double nu_a = Xsig_aug(5, i);
        double nu_yawdd = Xsig_aug(6, i);

        // current augmented state x_k
        VectorXd x_k(n_x);
        x_k = Xsig_aug.col(i).head(n_x);
        //process model
        VectorXd Process_model(n_x);
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
        Process_model << px_offset, py_offset, 0, yawd * delta_t, 0;
        //add noise
        //process noise
        VectorXd Noise_model(n_x);
        Noise_model << 0.5 * nu_a * delta_t * delta_t * cos(yaw),
                0.5 * nu_a * delta_t * delta_t * sin(yaw),
                nu_a * delta_t,
                0.5 * nu_yawdd * delta_t * delta_t,
                nu_yawdd * delta_t;

        //write predicted sigma point into right column
        Xsig_pred.col(i) = x_k + Process_model + Noise_model;
    }

    /**
     * Student part end
     */

    // print result
    std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;

    // write result
    *Xsig_out = Xsig_pred;
}

/**
Xsig_pred =
 5.93553  6.06251  5.92217   5.9415  5.92361  5.93516  5.93705  5.93553  5.80832  5.94481  5.92935  5.94553  5.93589  5.93401  5.93553
 1.48939  1.44673  1.66484  1.49719    1.508  1.49001  1.49022  1.48939   1.5308  1.31287  1.48182  1.46967  1.48876  1.48855  1.48939
  2.2049  2.28414  2.24557  2.29582   2.2049   2.2049  2.23954   2.2049  2.12566  2.16423  2.11398   2.2049   2.2049  2.17026   2.2049
 0.53678 0.473387 0.678098 0.554557 0.643644 0.543372  0.53678 0.538512 0.600173 0.395462 0.519003 0.429916 0.530188  0.53678 0.535048
  0.3528 0.299973 0.462123 0.376339  0.48417 0.418721   0.3528 0.387441 0.405627 0.243477 0.329261  0.22143 0.286879   0.3528 0.318159
**/