#ifndef _LLS_ONLINE_FPR_KALMANFILTER_H_
#define _LLS_ONLINE_FPR_KALMANFILTER_H_

#include <eigen3/Eigen/Eigen>
using namespace Eigen;


#define LLSKALMANFILTER

class LLS_KALMANFILTER{
public:
  VectorXf x;
  MatrixXf phi_nl;
  MatrixXf Pxx;
  MatrixXf H_posnl;
  MatrixXf H_velonl;
  MatrixXf R;

  void KalmanGain(int measurement);

  void EstiState(int measurement, Vector3f y);

  void EstiCovariance(int measurement);

  void Estimation(int measurement, Vector3f y);

  void PropaState(void);

  void PropaCovariance(void);

  void Propagation(void);

private:
  MatrixXf K;
  MatrixXf phi_l;
  MatrixXf H_posl;
  MatrixXf H_velol;
};

#endif //_LLS_ONLINE_FPR_KALMANFILTER_H_
