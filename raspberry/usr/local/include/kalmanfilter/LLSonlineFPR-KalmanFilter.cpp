#include "LLSonlineFPR-KalmanFilter.h"

void LLS_KALMANFILTER::KalmanGain(int measurement) {
  // MatrixXf Matrix(3,3);
  // if (measurement == 1) {
  //   Matrix = H_posl * Pxx * H_posl.transpose() + R;
  //   Matrix = Matrix.inverse();
  //   K = Pxx * H_posl.transpose() * Matrix;
  // }
  // if (measurement == 2) {
  //   Matrix = H_velol * Pxx * H_velol.transpose() + R;
  //   Matrix = Matrix.inverse();
  //   K = Pxx * H_velol.transpose() * Matrix;
  // }
}

void LLS_KALMANFILTER::EstiState(int measurement, Vector3f y) {
  // if (measurement == 1) {
  //   x = x + K * (y - H_posnl * x);
  // }
  // if (measurement == 2) {
  //   x = x + K * (y - H_velonl * x);
  // }
}

void LLS_KALMANFILTER::EstiCovariance(int measurement){
  // if (measurement == 1) {
  //   Pxx = Pxx - K * H_posl * Pxx;
  // }
  // if (measurement == 2) {
  //   Pxx = Pxx - K * H_velol * Pxx;
  // }
}

void LLS_KALMANFILTER::Estimation(int measurement, Vector3f y) {
  // KalmanGain(measurement);
  // EstiState(measurement, y);
  // EstiCovariance(measurement);
}

void LLS_KALMANFILTER::PropaState(void) {
  // x = phi_nl * x;
}

void LLS_KALMANFILTER::PropaCovariance(void) {
  // Pxx = phi_l * Pxx * phi_l.transpose();
}

void LLS_KALMANFILTER::Propagation(void) {
  // PropaState();
  // PropaCovariance();
}
