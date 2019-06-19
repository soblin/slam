#ifndef FRAMEWORK_CUSTOMIZER_H
#define FRAMEWORK_CUSTOMIZER_H

#include <slam/geometry/PointCloudMapBS.h>
#include <slam/geometry/PointCloudMapGT.h>
#include <slam/icp/CostFunctionED.h>
#include <slam/icp/CostFunctionPD.h>
#include <slam/icp/DataAssociatorGT.h>
#include <slam/icp/DataAssociatorLS.h>
#include <slam/icp/PoseEstimatorICP.h>
#include <slam/icp/PoseOptimizerSD.h>
#include <slam/icp/PoseOptimizerSL.h>
#include <slam/icp/RefScanMakerBS.h>
#include <slam/icp/RefScanMakerLM.h>
#include <slam/icp/ScanMatcher2D.h>
#include <slam/icp/ScanPointAnalyser.h>
#include <slam/icp/ScanPointResampler.h>
#include <slam/manager/SlamFrontEnd.h>

namespace slam {

class FrameWorkCustomizer {
private:
  RefScanMakerBS m_ref_scan_maker_bs;
  RefScanMakerLM m_ref_scan_maker_lm;

  DataAssociatorLS m_data_associator_ls;
  DataAssociatorGT m_data_associator_gt;

  CostFunctionED m_cost_function_ed;
  CostFunctionPD m_cost_function_pd;

  PoseOptimizerSD m_pose_optimizer_sd;
  PoseOptimizerSL m_pose_optimizer_sl;

  // This is the only one instance of PointCloudMap
  PointCloudMapBS m_point_cloud_map_bs;
  PointCloudMapGT m_point_cloud_map_gt;

  ScanPointAnalyser m_scan_point_analyser;
  ScanPointResampler m_scan_point_resampler;

  PoseEstimatorICP m_pose_estimator;
  ScanMatcher2D m_scan_matcher;

  SlamFrontEnd *m_slam_front_end_ptr; // points the the instance in SlamLauncher

public:
  FrameWorkCustomizer() : m_slam_front_end_ptr(nullptr) {}
  ~FrameWorkCustomizer() {}

  inline void SetSlamFrontEnd(SlamFrontEnd *f) { m_slam_front_end_ptr = f; }

  void CustomizeA();
  void CustomizeB();
  void CustomizeC();
  void CustomizeD();
  void CustomizeE();
  void CustomizeF();
  void CustomizeG();
  void CustomizeH();
};

} /* namespace slam */
#endif /* framework_customizer_h */
