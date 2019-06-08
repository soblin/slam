#include <slam/io/FrameWorkCustomizer.h>

namespace slam {

void FrameWorkCustomizer::CustomizeA() {
  m_slam_front_end_ptr->SetPointCloudMap(&m_point_cloud_map);
  m_slam_front_end_ptr->SetScanMatcher2D(&m_scan_matcher);
  //  m_point_cloud_map_ptr = &m_point_cloud_map;
  m_scan_matcher.SetEstimatorICP(&m_pose_estimator);
  m_scan_matcher.SetPointCloudMap(&m_point_cloud_map);
  m_scan_matcher.SetRefScanMaker(&m_ref_scan_maker);

  m_ref_scan_maker.SetPointCloudMap(&m_point_cloud_map);

  m_pose_estimator.SetDataAssociator(&m_data_associator);
  m_pose_estimator.SetPoseOptimizer(&m_pose_optimizer);

  m_pose_optimizer.SetCostFunction(&m_cost_function);
}
} /* namespace slam */
