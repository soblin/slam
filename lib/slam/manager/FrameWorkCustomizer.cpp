#include <iostream>
#include <slam/geometry/PointCloudMap.h>
#include <slam/manager/FrameWorkCustomizer.h>

namespace slam {

void FrameWorkCustomizer::CustomizeA() {
  RefScanMaker *rsm = &m_ref_scan_maker_bs;
  DataAssociator *dass = &m_data_associator_ls;
  CostFunction *cfunc = &m_cost_function_ed;
  PoseOptimizer *popt = &m_pose_optimizer_sd;

  // customize
  // use PointCloudMapBS
  PointCloudMapSingleton::Create(&m_point_cloud_map_bs);

  // common
  m_slam_front_end_ptr->SetScanMatcher2D(&m_scan_matcher);
  m_scan_matcher.SetEstimatorICP(&m_pose_estimator);

  popt->SetCostFunction(cfunc);
  m_pose_estimator.SetDataAssociator(dass);
  m_pose_estimator.SetPoseOptimizer(popt);

  m_scan_matcher.SetRefScanMaker(rsm);
}

void FrameWorkCustomizer::CustomizeB() {
  RefScanMaker *rsm = &m_ref_scan_maker_lm;
  DataAssociator *dass = &m_data_associator_ls;
  CostFunction *cfunc = &m_cost_function_ed;
  PoseOptimizer *popt = &m_pose_optimizer_sd;

  // customize
  // use PointCloudMapGT
  PointCloudMapSingleton::Create(&m_point_cloud_map_gt);

  // common
  m_slam_front_end_ptr->SetScanMatcher2D(&m_scan_matcher);
  m_scan_matcher.SetEstimatorICP(&m_pose_estimator);

  popt->SetCostFunction(cfunc);
  m_pose_estimator.SetDataAssociator(dass);
  m_pose_estimator.SetPoseOptimizer(popt);

  m_scan_matcher.SetRefScanMaker(rsm);
}

void FrameWorkCustomizer::CustomizeC() {
  RefScanMaker *rsm = &m_ref_scan_maker_lm;
  DataAssociator *dass = &m_data_associator_ls;
  CostFunction *cfunc = &m_cost_function_ed;
  PoseOptimizer *popt = &m_pose_optimizer_sl;

  // customize
  // use PointCloudMapGT
  PointCloudMapSingleton::Create(&m_point_cloud_map_gt);

  // common
  m_slam_front_end_ptr->SetScanMatcher2D(&m_scan_matcher);
  m_scan_matcher.SetEstimatorICP(&m_pose_estimator);

  popt->SetCostFunction(cfunc);
  m_pose_estimator.SetDataAssociator(dass);
  m_pose_estimator.SetPoseOptimizer(popt);

  m_scan_matcher.SetRefScanMaker(rsm);
}

void FrameWorkCustomizer::CustomizeD() {
  RefScanMaker *rsm = &m_ref_scan_maker_lm;
  DataAssociator *dass = &m_data_associator_gt;
  CostFunction *cfunc = &m_cost_function_ed;
  PoseOptimizer *popt = &m_pose_optimizer_sl;

  // customize
  // use PointCloudMapGT
  PointCloudMapSingleton::Create(&m_point_cloud_map_gt);

  // common
  m_slam_front_end_ptr->SetScanMatcher2D(&m_scan_matcher);
  m_scan_matcher.SetEstimatorICP(&m_pose_estimator);

  popt->SetCostFunction(cfunc);
  m_pose_estimator.SetDataAssociator(dass);
  m_pose_estimator.SetPoseOptimizer(popt);

  m_scan_matcher.SetRefScanMaker(rsm);
}

void FrameWorkCustomizer::CustomizeE() {
  RefScanMaker *rsm = &m_ref_scan_maker_lm;
  DataAssociator *dass = &m_data_associator_gt;
  CostFunction *cfunc = &m_cost_function_ed;
  PoseOptimizer *popt = &m_pose_optimizer_sl;

  // customize
  // use PointCloudMapGT
  PointCloudMapSingleton::Create(&m_point_cloud_map_gt);

  // common
  m_slam_front_end_ptr->SetScanMatcher2D(&m_scan_matcher);
  m_scan_matcher.SetEstimatorICP(&m_pose_estimator);

  popt->SetCostFunction(cfunc);
  m_pose_estimator.SetDataAssociator(dass);
  m_pose_estimator.SetPoseOptimizer(popt);

  m_scan_matcher.SetRefScanMaker(rsm);

  m_scan_matcher.SetScanPointResampler(&m_scan_point_resampler);
}

void FrameWorkCustomizer::CustomizeF() {
  RefScanMaker *rsm = &m_ref_scan_maker_lm;
  DataAssociator *dass = &m_data_associator_gt;
  CostFunction *cfunc = &m_cost_function_pd;
  PoseOptimizer *popt = &m_pose_optimizer_sl;

  // customize
  // use PointCloudMapGT
  PointCloudMapSingleton::Create(&m_point_cloud_map_gt);

  // common
  m_slam_front_end_ptr->SetScanMatcher2D(&m_scan_matcher);
  m_scan_matcher.SetEstimatorICP(&m_pose_estimator);

  popt->SetCostFunction(cfunc);
  m_pose_estimator.SetDataAssociator(dass);
  m_pose_estimator.SetPoseOptimizer(popt);

  m_scan_matcher.SetRefScanMaker(rsm);

  m_scan_matcher.SetScanPointAnalyser(&m_scan_point_analyser);
  //  m_scan_matcher.SetScanPointResampler(&m_scan_point_resampler);
}

void FrameWorkCustomizer::CustomizeG() {
  RefScanMaker *rsm = &m_ref_scan_maker_lm;
  DataAssociator *dass = &m_data_associator_gt;
  CostFunction *cfunc = &m_cost_function_pd;
  PoseOptimizer *popt = &m_pose_optimizer_sl;

  // customize
  // use PointCloudMapGT
  PointCloudMapSingleton::Create(&m_point_cloud_map_gt);

  // common
  m_slam_front_end_ptr->SetScanMatcher2D(&m_scan_matcher);
  m_scan_matcher.SetEstimatorICP(&m_pose_estimator);

  popt->SetCostFunction(cfunc);
  m_pose_estimator.SetDataAssociator(dass);
  m_pose_estimator.SetPoseOptimizer(popt);

  m_scan_matcher.SetRefScanMaker(rsm);

  m_scan_matcher.SetScanPointAnalyser(&m_scan_point_analyser);
  m_scan_matcher.SetScanPointResampler(&m_scan_point_resampler);
}

void FrameWorkCustomizer::CustomizeH() {
  RefScanMaker *rsm = &m_ref_scan_maker_lm;
  DataAssociator *dass = &m_data_associator_gt;
  CostFunction *cfunc = &m_cost_function_pd;
  PoseOptimizer *popt = &m_pose_optimizer_sl;

  // customize
  // use PointCloudMapGT
  PointCloudMapSingleton::Create(&m_point_cloud_map_gt);

  // common
  m_slam_front_end_ptr->SetScanMatcher2D(&m_scan_matcher);
  m_scan_matcher.SetEstimatorICP(&m_pose_estimator);

  popt->SetCostFunction(cfunc);
  m_pose_estimator.SetDataAssociator(dass);
  m_pose_estimator.SetPoseOptimizer(popt);

  m_scan_matcher.SetRefScanMaker(rsm);

  m_scan_matcher.SetScanPointAnalyser(&m_scan_point_analyser);
  m_scan_matcher.SetScanPointResampler(&m_scan_point_resampler);
}

} /* namespace slam */
