#include <slam/geometry/PointCloudMap.h>
#include <slam/manager/FrameWorkCustomizer.h>

namespace slam {

void FrameWorkCustomizer::CustomizeCommon() {
  // common
  m_slam_front_end_ptr->SetScanMatcher2D(&m_scan_matcher);
  m_scan_matcher.SetEstimatorICP(&m_pose_estimator);
  m_scan_matcher.SetPoseFuser(&m_pose_fuser);
  // loop detector
  m_loop_detector_ss.SetPoseEstimator(&m_pose_estimator);
  m_loop_detector_ss.SetPoseFuser(&m_pose_fuser);
  m_loop_detector_ss.SetDataAssociator(&m_data_associator_gt);
  m_loop_detector_ss.SetCostFunction(&m_cost_function_pd);
  m_loop_detector_ss.SetPointCloudMap(&m_point_cloud_map_lp);
}

void FrameWorkCustomizer::CustomizeA() {
  RefScanMaker *rsm = &m_ref_scan_maker_bs;
  DataAssociator *dass = &m_data_associator_ls;
  CostFunction *cfunc = &m_cost_function_ed;
  PoseOptimizer *popt = &m_pose_optimizer_sd;
  LoopDetector *lpd = &m_loop_detector;

  // customize
  // use PointCloudMapBS
  PointCloudMapSingleton::Create(&m_point_cloud_map_bs);

  CustomizeCommon();

  popt->SetCostFunction(cfunc);
  m_pose_estimator.SetDataAssociator(dass);
  m_pose_estimator.SetPoseOptimizer(popt);

  m_scan_matcher.SetRefScanMaker(rsm);
  // fuser
  m_slam_front_end_ptr->SetDgCheck(false);
  m_pose_fuser.SetDataAssociator(dass);
  // loop detector
  m_slam_front_end_ptr->SetLoopDetector(lpd);
}

void FrameWorkCustomizer::CustomizeB() {
  RefScanMaker *rsm = &m_ref_scan_maker_lm;
  DataAssociator *dass = &m_data_associator_ls;
  CostFunction *cfunc = &m_cost_function_ed;
  PoseOptimizer *popt = &m_pose_optimizer_sd;
  LoopDetector *lpd = &m_loop_detector;

  // customize
  // use PointCloudMapGT
  PointCloudMapSingleton::Create(&m_point_cloud_map_gt);

  CustomizeCommon();

  popt->SetCostFunction(cfunc);
  m_pose_estimator.SetDataAssociator(dass);
  m_pose_estimator.SetPoseOptimizer(popt);

  m_scan_matcher.SetRefScanMaker(rsm);
  // fuser
  m_slam_front_end_ptr->SetDgCheck(false);
  m_pose_fuser.SetDataAssociator(dass);
  // loop detector
  m_slam_front_end_ptr->SetLoopDetector(lpd);
}

void FrameWorkCustomizer::CustomizeC() {
  RefScanMaker *rsm = &m_ref_scan_maker_lm;
  DataAssociator *dass = &m_data_associator_ls;
  CostFunction *cfunc = &m_cost_function_ed;
  PoseOptimizer *popt = &m_pose_optimizer_sl;
  LoopDetector *lpd = &m_loop_detector;

  // customize
  // use PointCloudMapGT
  PointCloudMapSingleton::Create(&m_point_cloud_map_gt);

  CustomizeCommon();

  popt->SetCostFunction(cfunc);
  m_pose_estimator.SetDataAssociator(dass);
  m_pose_estimator.SetPoseOptimizer(popt);

  m_scan_matcher.SetRefScanMaker(rsm);
  // fuser
  m_slam_front_end_ptr->SetDgCheck(false);
  m_pose_fuser.SetDataAssociator(dass);
  // loop detector
  m_slam_front_end_ptr->SetLoopDetector(lpd);
}

void FrameWorkCustomizer::CustomizeD() {
  RefScanMaker *rsm = &m_ref_scan_maker_lm;
  DataAssociator *dass = &m_data_associator_gt;
  CostFunction *cfunc = &m_cost_function_ed;
  PoseOptimizer *popt = &m_pose_optimizer_sl;
  LoopDetector *lpd = &m_loop_detector;

  // customize
  // use PointCloudMapGT
  PointCloudMapSingleton::Create(&m_point_cloud_map_gt);

  CustomizeCommon();

  popt->SetCostFunction(cfunc);
  m_pose_estimator.SetDataAssociator(dass);
  m_pose_estimator.SetPoseOptimizer(popt);

  m_scan_matcher.SetRefScanMaker(rsm);
  // fuser
  m_slam_front_end_ptr->SetDgCheck(false);
  m_pose_fuser.SetDataAssociator(dass);
  // loop detector
  m_slam_front_end_ptr->SetLoopDetector(lpd);
}

void FrameWorkCustomizer::CustomizeE() {
  RefScanMaker *rsm = &m_ref_scan_maker_lm;
  DataAssociator *dass = &m_data_associator_gt;
  CostFunction *cfunc = &m_cost_function_ed;
  PoseOptimizer *popt = &m_pose_optimizer_sl;
  LoopDetector *lpd = &m_loop_detector;

  // customize
  // use PointCloudMapGT
  PointCloudMapSingleton::Create(&m_point_cloud_map_gt);

  CustomizeCommon();

  popt->SetCostFunction(cfunc);
  m_pose_estimator.SetDataAssociator(dass);
  m_pose_estimator.SetPoseOptimizer(popt);

  m_scan_matcher.SetRefScanMaker(rsm);

  m_scan_matcher.SetScanPointResampler(&m_scan_point_resampler);
  // fuser
  m_slam_front_end_ptr->SetDgCheck(false);
  m_pose_fuser.SetDataAssociator(dass);
  // loop detector
  m_slam_front_end_ptr->SetLoopDetector(lpd);
}

void FrameWorkCustomizer::CustomizeF() {
  RefScanMaker *rsm = &m_ref_scan_maker_lm;
  DataAssociator *dass = &m_data_associator_gt;
  CostFunction *cfunc = &m_cost_function_pd;
  PoseOptimizer *popt = &m_pose_optimizer_sl;
  LoopDetector *lpd = &m_loop_detector;

  // customize
  // use PointCloudMapGT
  PointCloudMapSingleton::Create(&m_point_cloud_map_gt);

  CustomizeCommon();

  popt->SetCostFunction(cfunc);
  m_pose_estimator.SetDataAssociator(dass);
  m_pose_estimator.SetPoseOptimizer(popt);

  m_scan_matcher.SetRefScanMaker(rsm);

  m_scan_matcher.SetScanPointAnalyser(&m_scan_point_analyser);
  //  m_scan_matcher.SetScanPointResampler(&m_scan_point_resampler);
  // fuser
  m_slam_front_end_ptr->SetDgCheck(false);
  m_pose_fuser.SetDataAssociator(dass);
  // loop detector
  m_slam_front_end_ptr->SetLoopDetector(lpd);
}

void FrameWorkCustomizer::CustomizeG() {
  RefScanMaker *rsm = &m_ref_scan_maker_lm;
  DataAssociator *dass = &m_data_associator_gt;
  CostFunction *cfunc = &m_cost_function_pd;
  PoseOptimizer *popt = &m_pose_optimizer_sl;
  LoopDetector *lpd = &m_loop_detector;

  // customize
  // use PointCloudMapGT
  PointCloudMapSingleton::Create(&m_point_cloud_map_gt);

  CustomizeCommon();

  popt->SetCostFunction(cfunc);
  m_pose_estimator.SetDataAssociator(dass);
  m_pose_estimator.SetPoseOptimizer(popt);

  m_scan_matcher.SetRefScanMaker(rsm);

  m_scan_matcher.SetScanPointAnalyser(&m_scan_point_analyser);
  m_scan_matcher.SetScanPointResampler(&m_scan_point_resampler);
  // fuser
  m_slam_front_end_ptr->SetDgCheck(false);
  m_pose_fuser.SetDataAssociator(dass);
  // loop detector
  m_slam_front_end_ptr->SetLoopDetector(lpd);
}

void FrameWorkCustomizer::CustomizeH() {
  RefScanMaker *rsm = &m_ref_scan_maker_lm;
  DataAssociator *dass = &m_data_associator_gt;
  CostFunction *cfunc = &m_cost_function_pd;
  PoseOptimizer *popt = &m_pose_optimizer_sl;
  LoopDetector *lpd = &m_loop_detector;

  // customize
  // use PointCloudMapGT
  PointCloudMapSingleton::Create(&m_point_cloud_map_gt);

  CustomizeCommon();

  popt->SetCostFunction(cfunc);
  m_pose_estimator.SetDataAssociator(dass);
  m_pose_estimator.SetPoseOptimizer(popt);

  m_scan_matcher.SetRefScanMaker(rsm);

  m_scan_matcher.SetScanPointAnalyser(&m_scan_point_analyser);
  m_scan_matcher.SetScanPointResampler(&m_scan_point_resampler);
  // fuser
  m_slam_front_end_ptr->SetDgCheck(true);
  m_pose_fuser.SetDataAssociator(dass);
  // loop detector
  m_slam_front_end_ptr->SetLoopDetector(lpd);
}

void FrameWorkCustomizer::CustomizeI() {
  RefScanMaker *rsm = &m_ref_scan_maker_lm;
  DataAssociator *dass = &m_data_associator_gt;
  CostFunction *cfunc = &m_cost_function_pd;
  PoseOptimizer *popt = &m_pose_optimizer_sl;
  LoopDetector *lpd = &m_loop_detector_ss;

  // customize
  // use PointCloudMapGT
  PointCloudMapSingleton::Create(&m_point_cloud_map_lp);

  CustomizeCommon();

  popt->SetCostFunction(cfunc);
  m_pose_estimator.SetDataAssociator(dass);
  m_pose_estimator.SetPoseOptimizer(popt);

  m_scan_matcher.SetRefScanMaker(rsm);

  m_scan_matcher.SetScanPointAnalyser(&m_scan_point_analyser);
  m_scan_matcher.SetScanPointResampler(&m_scan_point_resampler);
  // fuser
  m_slam_front_end_ptr->SetDgCheck(true);
  m_pose_fuser.SetDataAssociator(dass);
  // loop detector
  m_slam_front_end_ptr->SetLoopDetector(lpd);
}

} /* namespace slam */
