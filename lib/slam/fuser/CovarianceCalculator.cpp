#include <slam/fuser/CovarianceCalculator.h>
#include <slam/geometry/ScanPoint2D.h>
#include <slam/manager/ParamServer.h>

static double normalize(double th) {
  if (th > M_PI)
    return (th - 2 * M_PI);
  else if (th <= -M_PI)
    return (th + 2 * M_PI);

  return th;
}

static Eigen::Matrix3d svdInverse(const Eigen::Matrix3d &A) {
  size_t m = A.rows();
  size_t n = A.cols();

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU |
                                               Eigen::ComputeThinV);

  Eigen::MatrixXd eU = svd.matrixU();
  Eigen::MatrixXd eV = svd.matrixV();
  Eigen::VectorXd eS = svd.singularValues();

  Eigen::MatrixXd M1(m, n);
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < n; ++j) {
      M1(i, j) = eU(i, j) / eS[i];
    }
  }

  Eigen::Matrix3d IA;
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

static void CalcEigen2D(double (*mat)[2], double *vals, double *vec1,
                        double *vec2) {
  double a = mat[0][0];
  double b = mat[0][1];
  double c = mat[1][0];
  double d = mat[1][1];

  double B = std::sqrt((a + d) * (a + d) - 4 * (a * d - b * c));
  double x1 = ((a + d) + B) / 2;
  double x2 = ((a + d) - B) / 2;
  vals[0] = x1;
  vals[1] = x2;

  double m00 = a - x1;
  double m01 = b;
  double L = std::sqrt(m00 * m00 + m01 * m01);
  vec1[0] = m01 / L;
  vec1[1] = -m00 / L;

  m00 = a - x2;
  m01 = b;
  L = std::sqrt(m00 * m00 + m01 * m01);
  vec2[0] = m01 / L;
  vec2[1] = -m00 / L;
}

namespace slam {

void CovarianceCalculator::Initialize() {
  m_dd = ParamServer::Get("CovarianceCalculator_TICK_DIST");
  m_da = ParamServer::Get("CovarianceCalculator_TICK_THETA");
  m_alpha1 = ParamServer::Get("CovarianceCalculator_ALPHA1");
  m_alpha2 = ParamServer::Get("CovarianceCalculator_ALPHA2");
}

double CovarianceCalculator::CalcICPCovariance(
    const Pose2D &pose, const std::vector<const ScanPoint2D *> &curPoints,
    const std::vector<const ScanPoint2D *> &refPoints, Eigen::Matrix3d &cov) {
  double tx = pose.tx();
  double ty = pose.ty();
  double th = pose.th();
  double a = normalize(th);

  std::vector<double> Jx, Jy, Jt;

  for (unsigned i = 0; i < curPoints.size(); ++i) {
    const ScanPoint2D *cur_point = curPoints[i];
    const ScanPoint2D *ref_point = refPoints[i];

    if (ref_point->type() == ScanPoint2D::PointType::ISOLATE)
      continue;

    double pd0 = CalcPDistance(cur_point, ref_point, tx, ty, a);
    double pdx = CalcPDistance(cur_point, ref_point, tx + m_dd, ty, a);
    double pdy = CalcPDistance(cur_point, ref_point, tx, ty + m_dd, a);
    double pdt = CalcPDistance(cur_point, ref_point, tx, ty, a + m_da);

    Jx.emplace_back((pdx - pd0) / m_dd);
    Jy.emplace_back((pdy - pd0) / m_dd);
    Jt.emplace_back((pdt - pd0) / m_da);
  }

  Eigen::Matrix3d hes = Eigen::Matrix3d::Zero(3, 3);
  for (unsigned i = 0; i < Jx.size(); ++i) {
    hes(0, 0) += Jx[i] * Jx[i];
    hes(0, 1) += Jx[i] * Jy[i];
    hes(0, 2) += Jx[i] * Jt[i];
    hes(1, 1) += Jy[i] * Jy[i];
    hes(1, 2) += Jy[i] * Jt[i];
    hes(2, 2) += Jt[i] * Jt[i];
  }

  hes(1, 0) = hes(0, 1);
  hes(2, 0) = hes(0, 2);
  hes(2, 1) = hes(1, 2);

  cov = svdInverse(hes);

  double vals[2], vec1[2], vec2[2];
  double ratio = CalcEigen(cov, vals, vec1, vec2);

  double kk = ParamServer::Get("CovarianceCalculator_ICP_COV_SCALE1");

  cov *= kk;

  return ratio;
}

double CovarianceCalculator::CalcPDistance(const ScanPoint2D *curPoint,
                                           const ScanPoint2D *refPoint,
                                           double tx, double ty, double th) {
  double x = cos(th) * curPoint->x() - sin(th) * curPoint->y() + tx;
  double y = sin(th) * curPoint->x() + cos(th) * curPoint->y() + ty;
  double pdis = (x - refPoint->x()) * refPoint->nx() +
                (y - refPoint->y()) * refPoint->ny();

  return pdis;
}

void CovarianceCalculator::CalcMotionCovarianceSimple(const Pose2D &motion,
                                                      double dT,
                                                      Eigen::Matrix3d &cov) {
  double dis = std::hypot(motion.tx(), motion.ty());
  double vt = dis / dT;
  double wt = normalize(motion.th()) / dT;
  double vthre = ParamServer::Get("CovarianceCalculator_VEL_THRESH1");
  double wthre = ParamServer::Get("CovarianceCalculator_OMEGA_THRESH1");

  if (vt < vthre)
    vt = vthre;
  if (wt < wthre)
    wt = wthre;

  double dx = vt;
  double dy = vt;
  double da = wt;

  Eigen::Matrix3d C1;
  C1.setZero();
  double mcov_coeff_x = ParamServer::Get("CovarianceCalculator_MCOV_COEFF_X");
  double mcov_coeff_y = ParamServer::Get("CovarianceCalculator_MCOV_COEFF_Y");
  double mcov_coeff_th = ParamServer::Get("CovarianceCalculator_MCOV_COEFF_TH");
  C1(0, 0) = mcov_coeff_x * dx * dx;
  C1(1, 1) = mcov_coeff_y * dy * dy;
  C1(2, 2) = mcov_coeff_th * da * da;

  double kk = ParamServer::Get("CovarianceCalculator_ICP_COV_SCALE2");
  cov = kk * C1;
}

void CovarianceCalculator::CalcMotionCovariance(double th, double dx, double dy,
                                                double dth, double dt,
                                                Eigen::Matrix3d &cov,
                                                bool accum) {
  double dis = std::hypot(dx, dy);
  double vt = dis / dt;
  double wt = dth / dt;
  double vthre = ParamServer::Get("CovarianceCalculator_VEL_THRESH2");
  double wthre = ParamServer::Get("CovarianceCalculator_OMEGA_THRESH2");

  if (vt < vthre)
    vt = vthre;
  if (wt < wthre)
    wt = wthre;

  Eigen::Matrix3d A = Eigen::Matrix3d::Zero(3, 3);
  if (accum) {
    Eigen::Matrix3d Jxk;
    CalcJxk(th, vt, dt, Jxk);
    A = Jxk * cov * Jxk.transpose();
  }

  Eigen::Matrix2d Uk;
  CalcUk(vt, wt, Uk);

  Eigen::Matrix<double, 3, 2> Juk;
  CalcJuk(th, dt, Juk);
  Eigen::Matrix3d B = Juk * Uk * Juk.transpose();

  cov = A + B;
}

void CovarianceCalculator::CalcUk(double vt, double wt, Eigen::Matrix2d &Uk) {
  Uk << m_alpha1 * vt * vt, 0, 0, m_alpha2 * wt * wt;
}

void CovarianceCalculator::CalcJxk(double th, double vt, double dt,
                                   Eigen::Matrix3d &Jxk) {
  double cs = cos(th);
  double sn = sin(th);
  Jxk << 1, 0, -vt * dt * sn, 0, 1, vt * dt * cs, 0, 0, 1;
}

void CovarianceCalculator::CalcJuk(double th, double dt,
                                   Eigen::Matrix<double, 3, 2> &Juk) {
  double cs = cos(th);
  double sn = sin(th);
  Juk << dt * cs, 0, dt * sn, 0, 0, dt;
}

double CovarianceCalculator::CalcEigen(const Eigen::Matrix3d &cov, double *vals,
                                       double *vec1, double *vec2) {
  double cv2[2][2];
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      cv2[i][j] = cov(i, j);
    }
  }

  CalcEigen2D(cv2, vals, vec1, vec2);
  double ratio = vals[0] / vals[1];

  return ratio;
}

void CovarianceCalculator::AccumulateCovariance(const Pose2D &curPose,
                                                const Pose2D &prevPose,
                                                const Eigen::Matrix3d &prevCov,
                                                const Eigen::Matrix3d &mcov,
                                                Eigen::Matrix3d &curCov) {
  Eigen::Matrix3d J1, J2;
  J1 << 1, 0, -(curPose.ty() - prevPose.ty()), 0, 1,
      curPose.tx() - prevPose.tx(), 0, 0, 1;

  double prevCos = cos(normalize(prevPose.th()));
  double prevSin = sin(normalize(prevPose.th()));

  J2 << prevCos, -prevSin, 0, prevSin, prevCos, 0, 0, 0, 1;

  curCov = J1 * prevCov * J1.transpose() + J2 * mcov * J2.transpose();
}

void CovarianceCalculator::RotateCovariance(const Pose2D &pose,
                                            const Eigen::Matrix3d &cov,
                                            Eigen::Matrix3d &icov,
                                            bool reverse) {
  double cs = cos(normalize(pose.th()));
  double sn = sin(normalize(pose.th()));

  Eigen::Matrix3d J;
  J << cs, -sn, 0, sn, cs, 0, 0, 0, 1;

  Eigen::Matrix3d JT = J.transpose();

  if (reverse)
    icov = JT * cov * J;
  else
    icov = J * cov * JT;
}

} // namespace slam
