#ifndef COVARIANCE_CALCULATOR_H
#define COVARIANCE_CALCULATOR_H

#include <vector>
#include <slam/geometry/ScanPoint2D.h>
#include <slam/geometry/Pose2D.h>

#include <Eigen/Eigen>

class CovarianceCalculator {
 public:
  CovarianceCalculator() {}
  ~CovarianceCalculator() {}

  void Initialize();
  double CalcICPCovariance(const Pose2D &pose, std::vector<const ScanPoint2D *> &curPoints, std::vector<const ScanPoint2D *> &refPoints, Eigen::Matrix3d &cov);
  double CalcPDistance(const ScanPoint2D *curPoint, const ScanPoint2D *refPoint, double tx, double ty, double th);
  void CalcMotionCovarianceSimple(const Pose2D &motion, double dT, Eigen::Matrix3d &cov);
  void CalcMotionCovariance(double th, double dx, double dy, double dth, double dt, Eigen::Matrix3d &cov, bool accum = false);
  void CalcUk(double vt, double wt, Eigen::Matrix2d &Uk);
  void CalcJxk(double th, double vt, double dt, Eigen::Matrix3d &Jxk);
  void CalcJuk(double th, double dt, Eigen::Matrix<double, 3, 2> &Juk);
  double CalcEigen(const Eigen::Matrix3d &cov, double *vals, double *vec1, double *vec2);

  static void AccumulateCovariance(const Pose2D &curPose, const Pose2D &prevPose, const Eigen::Matrix3d &mcov, Eigen::Matrix3d &curCov);
  static void RotateCovariance(const Pose2D &pose, const Eigen::Matrix3d &cov, Eigen::Matrix3d &icov, bool reverse = false);

 private:
  double m_dd = 0;
  double m_da = 0;
};

#endif /* COVARIANCE_CALCULATOR_H */
