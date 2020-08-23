#ifndef POSE_ESTIMATOR_ICP_H
#define POSE_ESTIMATOR_ICP_H

#include <slam/geometry/Scan2D.h>
#include <slam/icp/DataAssociator.h>
#include <slam/icp/PoseOptimizer.h>

namespace slam {

class PoseEstimatorICP {
public:
  PoseEstimatorICP() {}
  ~PoseEstimatorICP() {}

  double EstimatePose(const Pose2D &initPose, Pose2D &estimatePose);
  void Initialize();

public:
  void SetPoseOptimizer(PoseOptimizer *p);
  void SetDataAssociator(DataAssociator *p);
  int GetUsedNum() const;
  double GetMatchRate() const;
  void SetScanPair(const Scan2D *cur, const Scan2D *ref);
  void SetScanPair(const Scan2D *cur, const std::vector<ScanPoint2D> &ref);

private:
  const Scan2D *m_cur_scan = nullptr;
  std::size_t m_used_points_num = 0; // the number of points used for ICP
  double m_matched_rate = 0;

  PoseOptimizer *m_optimizer_ptr = nullptr;
  DataAssociator *m_associator_ptr = nullptr;
};

inline void PoseEstimatorICP::SetPoseOptimizer(PoseOptimizer *p) {
  m_optimizer_ptr = p;
}

inline void PoseEstimatorICP::SetDataAssociator(DataAssociator *p) {
  m_associator_ptr = p;
}

inline int PoseEstimatorICP::GetUsedNum() const { return m_used_points_num; }

inline double PoseEstimatorICP::GetMatchRate() const { return m_matched_rate; }

inline void PoseEstimatorICP::SetScanPair(const Scan2D *cur,
                                          const Scan2D *ref) {
  m_cur_scan = cur;
  m_associator_ptr->SetRefBase(ref->scaned_points());
}

inline void PoseEstimatorICP::SetScanPair(const Scan2D *cur,
                                          const std::vector<ScanPoint2D> &ref) {
  m_cur_scan = cur;
  m_associator_ptr->SetRefBase(ref);
}

} /* namespace slam */
#endif /* pose_estimator_icp_h */
