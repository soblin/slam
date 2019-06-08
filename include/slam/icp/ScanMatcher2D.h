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
  Scan2D m_prev_scan;
  Pose2D m_init_pose; // the pose of the origin of the map. default(0, 0, 0)

  double m_score_thresh;    // the threshold for matching score
  double m_used_num_thresh; // the threshold for the number of used

  double m_acc_dist;         // accumulated distance
  double m_degenerate_check; // if check degeneration

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
  inline void SetDegenerateCheck(bool b) { m_degenerate_check = b; }
  inline void SetInitPose(const Pose2D &p) { m_init_pose = p; }

public:
  ScanMatcher2D()
      : m_cnt(-1), m_score_thresh(1.0), m_used_num_thresh(50), m_acc_dist(0),
        m_degenerate_check(false), m_estimator_ptr(nullptr),
        m_point_cloud_ptr(nullptr), m_ref_scan_maker_ptr(nullptr) {}

  ~ScanMatcher2D() {}

  bool MatchScan(Scan2D &scan);
  void GrowMap(const Scan2D &scan, const Pose2D &pose);
};

} /* namespace slam */
#endif /* scan_matcher_2d_h */
