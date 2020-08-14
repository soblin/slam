#include <Eigen/SVD>
#include <slam/fuser/PoseFuser.h>

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

static double normalize(double x) {
  if (x > M_PI)
    return (x - 2 * M_PI);
  else if (x <= -M_PI)
    return (x + 2 * M_PI);
  return x;
}

static Matrix3d svdInverse(const Eigen::Matrix3d &A) {
  size_t m = A.rows();
  size_t n = A.cols();

  Eigen::JacobiSVD<MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

  MatrixXd eU = svd.matrixU();
  MatrixXd eV = svd.matrixV();
  VectorXd eS = svd.singularValues();

  MatrixXd M1(m, n);
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < n; ++j) {
      M1(i, j) = eU(i, j) / eS[i];
    }
  }

  Matrix3d IA;
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < n; ++j) {
      IA(i, j) = 0;
      for (size_t k = 0; k < n; ++k) {
        IA(i, j) += eV(i, k) * M1(k, j);
      }
    }
  }

  return IA;
}

namespace slam {

void PoseFuser::Initialize() { m_cov_calculator.Initialize(); }

double PoseFuser::CalcICPCovariance(const Pose2D &estMotion,
                                    const Scan2D *curScan,
                                    Eigen::Matrix3d &cov) {
  m_data_associator_ptr->FindCorrespondence(curScan, estMotion);

  double ratio = m_cov_calculator.CalcICPCovariance(
      estMotion, m_data_associator_ptr->cur_points_ref(),
      m_data_associator_ptr->ref_points_ref(), cov);

  return ratio;
}

double PoseFuser::FusePose(Scan2D *curScan, const Pose2D &estPose,
                           const Pose2D &odoMotion, const Pose2D &lastPose,
                           Pose2D &fusedPose, Eigen::Matrix3d &fusedCov) {
  m_data_associator_ptr->FindCorrespondence(curScan, estPose);
  double ratio = m_cov_calculator.CalcICPCovariance(
      estPose, m_data_associator_ptr->cur_points_ref(),
      m_data_associator_ptr->ref_points_ref(), m_ecov);

  Pose2D predPose;
  Pose2D::CalcGlobalPose(odoMotion, lastPose, predPose);

  Matrix3d mcovL;
  double dT = 0.1;
  m_cov_calculator.CalcMotionCovarianceSimple(odoMotion, dT, mcovL);
  CovarianceCalculator::RotateCovariance(estPose, mcovL, m_mcov);

  Vector3d mu1(estPose.tx(), estPose.ty(), normalize(estPose.th()));
  Vector3d mu2(predPose.tx(), predPose.ty(), normalize(predPose.th()));
  Vector3d mu;

  Fuse(mu1, m_ecov, mu2, m_mcov, mu, fusedCov);

  fusedPose.SetVal(mu[0], mu[1], normalize(mu[2]));

  m_totalcov = fusedCov;

  return ratio;
}

void PoseFuser::CalcOdometryCovariance(const Pose2D &odoMotion,
                                       const Pose2D &lastPose,
                                       Eigen::Matrix3d &mcov) {
  Matrix3d mcovL;
  double dT = 0.1;
  m_cov_calculator.CalcMotionCovarianceSimple(odoMotion, dT, mcovL);

  CovarianceCalculator::RotateCovariance(lastPose, mcovL, mcov);
}

double PoseFuser::Fuse(const Eigen::Vector3d &mu1, const Eigen::Matrix3d &cv1,
                       const Eigen::Vector3d &mu2, const Eigen::Matrix3d &cv2,
                       Eigen::Vector3d &mu, Eigen::Matrix3d &cv) {
  Matrix3d IC1 = svdInverse(cv1);
  Matrix3d IC2 = svdInverse(cv2);
  Matrix3d IC = IC1 + IC2;
  cv = svdInverse(IC);

  Vector3d mu11 = mu1;
  double da = mu2(2) - mu1(2);
  if (da > M_PI)
    mu11(2) += 2 * M_PI;
  else if (da <= -M_PI)
    mu11(2) -= 2 * M_PI;

  Vector3d nu1 = IC1 * mu11;
  Vector3d nu2 = IC2 * mu2;
  Vector3d nu3 = nu1 + nu2;
  mu = cv * nu3;

  if (mu(2) > M_PI)
    mu(2) -= 2 * M_PI;
  else if (mu(2) <= -M_PI)
    mu(2) += 2 * M_PI;

  Vector3d W1 = IC1 * mu11;
  Vector3d W2 = IC2 * mu2;
  Vector3d W = IC * mu;

  double A1 = mu1.dot(W1);
  double A2 = mu2.dot(W2);
  double A = mu.dot(W);
  double K = A1 + A2 - A;

  return K;
}

} // namespace slam
