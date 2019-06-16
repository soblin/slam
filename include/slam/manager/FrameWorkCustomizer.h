#ifndef FRAMEWORK_CUSTOMIZER_H
#define FRAMEWORK_CUSTOMIZER_H

#include <slam/geometry/PointCloudMapBS.h>
#include <slam/icp/CostFunctionED.h>
#include <slam/icp/DataAssociatorLS.h>
#include <slam/icp/PoseEstimatorICP.h>
#include <slam/icp/PoseOptimizerSD.h>
#include <slam/icp/RefScanMakerBS.h>
#include <slam/icp/ScanMatcher2D.h>
#include <slam/manager/SlamFrontEnd.h>

namespace slam {

class FrameWorkCustomizer {
private:
  RefScanMakerBS m_ref_scan_maker;
  DataAssociatorLS m_data_associator;
  CostFunctionED m_cost_function;
  PoseOptimizerSD m_pose_optimizer;
  // This is the only one instance of PointCloudMap
  PointCloudMapBS m_point_cloud_map_bs;
  PoseEstimatorICP m_pose_estimator;
  ScanMatcher2D m_scan_matcher;

  SlamFrontEnd *m_slam_front_end_ptr; // points the the instance in SlamLauncher

public:
  FrameWorkCustomizer() : m_slam_front_end_ptr(nullptr) {}
  ~FrameWorkCustomizer() {}

  inline void SetSlamFrontEnd(SlamFrontEnd *f) { m_slam_front_end_ptr = f; }

  void CustomizeA();
};

} /* namespace slam */
#endif /* framework_customizer_h */
