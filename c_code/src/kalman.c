#include "../include/kalman.h"

void kalman_init(KalmanFilter *kf) {
    kf->Q_angle = 0.001;
    kf->Q_bias = 0.003;
    kf->R_measure = 0.03;

    kf->angle = 0.0; 
    kf->bias = 0.0;

    kf->P[0][0] = 0.0;
    kf->P[0][1] = 0.0;
    kf->P[1][0] = 0.0;
    kf->P[1][1] = 0.0;
}

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
double kalman_get_angle(KalmanFilter *kf, double newAngle, double newRate, double dt) {
    // --- Discrete Kalman filter time update equations - Time Update ("Predict") ---
    
    // Step 1: Update xhat - Project the state ahead
    // Estimate new angle based on gyro rate
    double rate = newRate - kf->bias;
    kf->angle += dt * rate;

    // Step 2: Update estimation error covariance - Project the error covariance ahead
    // Increase uncertainty due to process noise
    kf->P[0][0] += dt * (dt*kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    // --- Discrete Kalman filter measurement update equations - Measurement Update ("Correct") ---
    
    // Step 3: Calculate difference between measurement and prediction
    // Innovation (or residual)
    double y = newAngle - kf->angle; 

    // Step 4: Calculate innovation covariance
    // Estimate error
    double S = kf->P[0][0] + kf->R_measure; 

    // Step 5: Calculate Kalman gain
    // Standard Kalman gain calculation
    double K[2]; 
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    // Step 6: Update estimate with measurement zk (newAngle)
    // Correct the predicted state
    kf->angle += K[0] * y;
    kf->bias += K[1] * y;

    // Step 7: Calculate estimation error covariance
    // Update the error covariance matrix
    double P00_temp = kf->P[0][0];
    double P01_temp = kf->P[0][1];

    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    return kf->angle;
}
