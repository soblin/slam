#ifndef POSE_ESTIMATOR_ICP_H
#define POSE_ESTIMATOR_ICP_H

#include <slam/geometry/Scan2D.h>
#include <slam/icp/DataAssociator.h>
#include <slam/icp/PoseOptimizer.h>

namespace slam {

class PoseEstimatorICP {
private:
  const Scan2D *m_cur_scan;
  std::size_t m_used_points_num; // the number of points used for ICP
  double m_matched_rate;

  PoseOptimizer *m_optimizer_ptr;
  DataAssociator *m_associator_ptr;

public:
  inline void SetPoseOptimizer(PoseOptimizer *p) { m_optimizer_ptr = p; }
  inline void SetDataAssociator(DataAssociator *p) { m_associator_ptr = p; }
  inline int GetUsedNum() const { return m_used_points_num; }
  inline void SetScanPair(const Scan2D *cur, const Scan2D *ref) {
    m_cur_scan = cur;
    m_associator_ptr->SetRefBase(ref->scaned_points());
  }
  inline void SetScanPair(const Scan2D *cur,
                          const std::vector<ScanPoint2D> &ref) {
    m_cur_scan = cur;
    m_associator_ptr->SetRefBase(ref);
  }

public:
  PoseEstimatorICP() : m_used_points_num(0), m_matched_rate(0) {}
  ~PoseEstimatorICP() {}

  double EstimatePose(const Pose2D &initPose, Pose2D &estimatePose);
  void Initialize();
};

} /* namespace slam */
#endif /* pose_estimator_icp_h */
