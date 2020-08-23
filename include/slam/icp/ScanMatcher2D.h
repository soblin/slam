#ifndef SCAN_MATCHER_2D_H
#define SCAN_MATCHER_2D_H

#include <slam/fuser/PoseFuser.h>
#include <slam/geometry/PointCloudMap.h>
#include <slam/geometry/Pose2D.h>
#include <slam/geometry/Scan2D.h>
#include <slam/icp/PoseEstimatorICP.h>
#include <slam/icp/RefScanMaker.h>
#include <slam/icp/ScanPointAnalyser.h>
#include <slam/icp/ScanPointResampler.h>

namespace slam {

class ScanMatcher2D {
public:
  ScanMatcher2D() {}
  ~ScanMatcher2D() {}

  void Initialize();
  bool MatchScan(Scan2D &scan);
  void GrowMap(const Scan2D &scan, const Pose2D &pose);

public:
  void SetEstimatorICP(PoseEstimatorICP *p);
  void SetRefScanMaker(RefScanMaker *p);
  void SetInitPose(const Pose2D &p);
  void SetScanPointResampler(ScanPointResampler *p);
  void SetScanPointAnalyser(ScanPointAnalyser *p);
  void SetPoseFuser(PoseFuser *p);
  void SetDgCheck(bool t);

private:
  Pose2D m_init_pose; // the pose of the origin of the map. default(0, 0, 0)

  PoseEstimatorICP *m_estimator_ptr = nullptr;
  RefScanMaker *m_ref_scan_maker_ptr = nullptr;
  ScanPointResampler *m_scan_point_resampler_ptr = nullptr;
  ScanPointAnalyser *m_scan_point_analyser_ptr = nullptr;
  PoseFuser *m_pose_fuser_ptr = nullptr;
  bool m_dgcheck = false;
  Eigen::Matrix3d m_cov;
  Eigen::Matrix3d m_totalcov;
  double m_acc_dist = 0;
  bool m_is_first = true;

  // initialied in Initialize()
  double m_score_thresh = 0;
  double m_used_num_thresh = 0;
  PointCloudMap *m_cloud_map_ptr = nullptr;
};

inline void ScanMatcher2D::SetEstimatorICP(PoseEstimatorICP *p) {
  m_estimator_ptr = p;
}
inline void ScanMatcher2D::SetRefScanMaker(RefScanMaker *p) {
  m_ref_scan_maker_ptr = p;
}
inline void ScanMatcher2D::SetInitPose(const Pose2D &p) { m_init_pose = p; }
inline void ScanMatcher2D::SetScanPointResampler(ScanPointResampler *p) {
  m_scan_point_resampler_ptr = p;
}
inline void ScanMatcher2D::SetScanPointAnalyser(ScanPointAnalyser *p) {
  m_scan_point_analyser_ptr = p;
}
inline void ScanMatcher2D::SetPoseFuser(PoseFuser *p) { m_pose_fuser_ptr = p; }
inline void ScanMatcher2D::SetDgCheck(bool t) { m_dgcheck = t; }

} /* namespace slam */
#endif /* SCAN_MATCHER_2D_H */
