#ifndef KALMAN_H
#define KALMAN_H

typedef struct {
    double angle;   // The angle calculated by the Kalman filter - part of the 2x1 state vector
    double bias;    // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    double P[2][2]; // Error covariance matrix - This is a 2x2 matrix
    
    // Tuning Constants
    double Q_angle; // Process noise variance for the accelerometer
    double Q_bias;  // Process noise variance for the gyro bias
    double R_measure; // Measurement noise variance - this is actually the variance of the measurement noise
} KalmanFilter;

void kalman_init(KalmanFilter *kf);
double kalman_get_angle(KalmanFilter *kf, double newAngle, double newRate, double dt);

#endif
