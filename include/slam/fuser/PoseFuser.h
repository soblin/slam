#ifndef POSE_FUSER_H
#define POSE_FUSER_H

#include <Eigen/Eigen>

#include <slam/fuser/CovarianceCalculator.h>
#include <slam/icp/DataAssociator.h>

namespace slam {

class PoseFuser {
public:
  PoseFuser() {}
  ~PoseFuser() {}

  void Initialize();
  double CalcICPCovariance(const Pose2D &estMotion, const Scan2D *curScan,
                           Eigen::Matrix3d &cov);
  double FusePose(Scan2D *curScan, const Pose2D &estPose,
                  const Pose2D &odoMotion, const Pose2D &lastPose,
                  Pose2D &fusedPose, Eigen::Matrix3d &cov);
  void CalcOdometryCovariance(const Pose2D &odoMotion, const Pose2D &lastPose,
                              Eigen::Matrix3d &mcov);
  double Fuse(const Eigen::Vector3d &mu1, const Eigen::Matrix3d &cv1,
              const Eigen::Vector3d &mu2, const Eigen::Matrix3d &cv2,
              Eigen::Vector3d &mu, Eigen::Matrix3d &cv);

public:
  inline void SetDataAssociator(DataAssociator *ptr) {
    m_data_associator_ptr = ptr;
  }
  inline void SetRefBase(const Scan2D *refScan) {
    m_data_associator_ptr->SetRefBase(refScan->scaned_points());
  }

private:
  Eigen::Matrix3d m_ecov;
  Eigen::Matrix3d m_mcov;
  Eigen::Matrix3d m_totalcov;

  DataAssociator *m_data_associator_ptr = nullptr;
  CovarianceCalculator m_cov_calculator;
};

} // namespace slam
#endif /* POSE_FUSER_H */
