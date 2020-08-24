#ifndef SLAM_FRONT_END_H
#define SLAM_FRONT_END_H

#include <slam/geometry/PointCloudMap.h>
#include <slam/icp/ScanMatcher2D.h>
#include <slam/loop_closure/LoopDetector.h>
#include <slam/loop_closure/PoseGraph.h>
#include <slam/manager/SlamBackEnd.h>

namespace slam {

class SlamFrontEnd {
public:
  SlamFrontEnd() {}
  ~SlamFrontEnd() {}

  void Initialize();
  void Process(Scan2D &scan);
  bool MakeOdometryArc(Pose2D &curPose, const Eigen::Matrix3d &cov);

public:
  void SetScanMatcher2D(ScanMatcher2D *p);
  void SetDgCheck(bool p);
  void SetLoopDetector(LoopDetector *p);

private:
  // initialized in Initialize()
  PointCloudMap *m_cloud_map_ptr = nullptr;
  ScanMatcher2D *m_scan_matcher_ptr = nullptr;
  PoseGraph *m_pose_graph_ptr = nullptr;
  LoopDetector *m_loop_detector_ptr = nullptr;
  SlamBackEnd m_slam_back_end;
  void RegisterParams();
};

inline void SlamFrontEnd::SetScanMatcher2D(ScanMatcher2D *p) {
  m_scan_matcher_ptr = p;
}
inline void SlamFrontEnd::SetDgCheck(bool p) {
  m_scan_matcher_ptr->SetDgCheck(p);
}

inline void SlamFrontEnd::SetLoopDetector(LoopDetector *p) {
  m_loop_detector_ptr = p;
  m_loop_detector_ptr->SetPoseGraph(m_pose_graph_ptr);
}

} /* namespace slam */
#endif /* slam_front_end_h */
