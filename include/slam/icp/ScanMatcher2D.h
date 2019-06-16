#ifndef SCAN_MATCHER_2D_H
#define SCAN_MATCHER_2D_H

#include <slam/geometry/PointCloudMap.h>
#include <slam/geometry/Pose2D.h>
#include <slam/geometry/Scan2D.h>
#include <slam/icp/PoseEstimatorICP.h>
#include <slam/icp/RefScanMaker.h>

namespace slam {

class ScanMatcher2D {
private:
  int m_cnt;
  Pose2D m_init_pose; // the pose of the origin of the map. default(0, 0, 0)

  PoseEstimatorICP *m_estimator_ptr;
  PointCloudMap *m_point_cloud_ptr;
  RefScanMaker *m_ref_scan_maker_ptr;

public:
  inline void SetEstimatorICP(PoseEstimatorICP *p) { m_estimator_ptr = p; }
  inline void SetRefScanMaker(RefScanMaker *p) {
    m_ref_scan_maker_ptr = p;
    if (m_point_cloud_ptr != nullptr) {
      m_ref_scan_maker_ptr->SetPointCloudMap(m_point_cloud_ptr);
    }
  }

  inline void SetPointCloudMap(PointCloudMap *p) {
    m_point_cloud_ptr = p;
    if (m_ref_scan_maker_ptr != nullptr) {
      m_ref_scan_maker_ptr->SetPointCloudMap(m_point_cloud_ptr);
    }
  }
  inline void Reset() { m_cnt = -1; }
  inline void SetInitPose(const Pose2D &p) { m_init_pose = p; }

public:
  ScanMatcher2D()
      : m_cnt(-1), m_estimator_ptr(nullptr), m_point_cloud_ptr(nullptr),
        m_ref_scan_maker_ptr(nullptr) {}

  ~ScanMatcher2D() {}

  bool MatchScan(Scan2D &scan);
  void GrowMap(const Scan2D &scan, const Pose2D &pose);
};

} /* namespace slam */
#endif /* scan_matcher_2d_h */
