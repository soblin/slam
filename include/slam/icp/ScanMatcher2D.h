#ifndef SCAN_MATCHER_2D_H
#define SCAN_MATCHER_2D_H

#include <slam/geometry/PointCloudMap.h>
#include <slam/geometry/Pose2D.h>
#include <slam/geometry/Scan2D.h>
#include <slam/icp/PoseEstimatorICP.h>
#include <slam/icp/RefScanMaker.h>
#include <slam/icp/ScanPointAnalyser.h>
#include <slam/icp/ScanPointResampler.h>

namespace slam {

class ScanMatcher2D {
private:
  Pose2D m_init_pose; // the pose of the origin of the map. default(0, 0, 0)

  PoseEstimatorICP *m_estimator_ptr;
  RefScanMaker *m_ref_scan_maker_ptr;
  ScanPointResampler *m_scan_point_resampler_ptr;
  ScanPointAnalyser *m_scan_point_analyser_ptr;

public:
  inline void SetEstimatorICP(PoseEstimatorICP *p) { m_estimator_ptr = p; }
  inline void SetRefScanMaker(RefScanMaker *p) { m_ref_scan_maker_ptr = p; }
  inline void SetInitPose(const Pose2D &p) { m_init_pose = p; }
  inline void SetScanPointResampler(ScanPointResampler *p) {
    m_scan_point_resampler_ptr = p;
  }
  inline void SetScanPointAnalyser(ScanPointAnalyser *p) {
    m_scan_point_analyser_ptr = p;
  }

public:
  ScanMatcher2D()
      : m_estimator_ptr(nullptr), m_ref_scan_maker_ptr(nullptr),
        m_scan_point_resampler_ptr(nullptr),
        m_scan_point_analyser_ptr(nullptr) {}

  ~ScanMatcher2D() {}

  void Initialize();
  bool MatchScan(Scan2D &scan);
  void GrowMap(const Scan2D &scan, const Pose2D &pose);
};

} /* namespace slam */
#endif /* scan_matcher_2d_h */
