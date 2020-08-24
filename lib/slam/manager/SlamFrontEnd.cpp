#include <cassert>
#include <cmath>
#include <slam/manager/CounterServer.h>
#include <slam/manager/ParamServer.h>
#include <slam/manager/SlamFrontEnd.h>
#include <slam/parameters.h>

namespace slam {

void SlamFrontEnd::Initialize() {
  CounterServer::Create();

  // Load Parameters
  ParamServer::Create();
  RegisterParams();

  m_cloud_map_ptr = PointCloudMapSingleton::GetCloudMap();
  m_cloud_map_ptr->Initialize();

  m_pose_graph_ptr = new PoseGraph();
  m_pose_graph_ptr->Initialize();

  // Initialize the chained classes and update their parameters
  m_scan_matcher_ptr->Initialize();

  m_slam_back_end.Initialize(m_pose_graph_ptr);

  m_loop_detector_ptr->Initialize();

  m_keyframe_skip = ParamServer::Get("SlamFrontEnd_KEYFRAME_SKIP");
}

// process the scan data, which was generated at SensorDataReader
void SlamFrontEnd::Process(Scan2D &scan) {
  int cnt = CounterServer::Get();

  m_scan_matcher_ptr->MatchScan(scan);

  // get the estimated current pose with ICP
  Pose2D curPose = m_cloud_map_ptr->GetLastPose();
  // and add to OdometryArc
  if (cnt == 0)
    m_pose_graph_ptr->AddNode(curPose);
  else {
    Eigen::Matrix3d &cov = m_scan_matcher_ptr->GetCovariance();
    MakeOdometryArc(curPose, cov);
  }

  // MapGen
  if (cnt % m_keyframe_skip == 0) {
    const int gt_cell_num_thresh =
        ParamServer::Get("PointCloudMapGT_CELL_POINT_NUM_COUNTER_THRESH");

    if (cnt < gt_cell_num_thresh) {
      ParamServer::Set("PointCloudMapGT_CELL_POINT_NUM_THRESH", 1.0);
    } else
      ParamServer::Set("PointCloudMapGT_CELL_POINT_NUM_THRESH", 5.0);

    m_cloud_map_ptr->MakeGlobalMap();
  }

  // loop closure
  if (cnt > m_keyframe_skip and cnt % m_keyframe_skip == 0) {
    int i = i + 1;
  }

  CounterServer::Increment();
}

bool SlamFrontEnd::MakeOdometryArc(Pose2D &curPose,
                                   const Eigen::Matrix3d &fusedCov) {
  if (m_pose_graph_ptr->nodes().size() == 0)
    return false;

  PoseNode *lastNode = m_pose_graph_ptr->nodes().back();
  PoseNode *curNode = m_pose_graph_ptr->AddNode(curPose);

  Pose2D &lastPose = lastNode->pose();
  Pose2D relPose;
  Pose2D::CalcRelativePose(curPose, lastPose, relPose);

  Eigen::Matrix3d cov;
  CovarianceCalculator::RotateCovariance(lastPose, fusedCov, cov, true);
  PoseArc *arc =
      m_pose_graph_ptr->MakeArc(lastNode->id(), curNode->id(), relPose, cov);
  m_pose_graph_ptr->AddArc(arc);

  return true;
}

void SlamFrontEnd::RegisterParams() {
  // SensorDataReader
  ParamServer::Set("Scan2D_MAX_SCAN_RANGE", param::Scan2D_MAX_SCAN_RANGE);
  ParamServer::Set("Scan2D_MIN_SCAN_RANGE", param::Scan2D_MIN_SCAN_RANGE);
  ParamServer::Set("SensorDataReader_ANGLE_OFFSET",
                   param::SensorDataReader_ANGLE_OFFSET);

  // MapDrawer
  ParamServer::Set("MapDrawer_STEP_POINT", param::MapDrawer_STEP_POINT);
  ParamServer::Set("MapDrawer_STEP_POSE", param::MapDrawer_STEP_POSE);
  ParamServer::Set("MapDrawer_DD", param::MapDrawer_DD);

  // SlamLauncher
  ParamServer::Set("SlamLauncher_SLEEP_TIME", param::SlamLauncher_SLEEP_TIME);
  ParamServer::Set("SlamLauncher_PLOT_SKIP", param::SlamLauncher_PLOT_SKIP);

  // SlamFrontEnd
  ParamServer::Set("PointCloudMapGT_CELL_POINT_NUM_COUNTER_THRESH",
                   param::PointCloudMapGT_CELL_POINT_NUM_COUNTER_THRESH);
  ParamServer::Set("SlamFrontEnd_KEYFRAME_SKIP",
                   param::SlamFrontEnd_KEYFRAME_SKIP);

  // PointCloudMap
  ParamServer::Set("PointCloudMapBS_SKIP", param::PointCloudMapBS_SKIP);
  ParamServer::Set("PointCloudMap_MAX_POINT_NUM",
                   param::PointCloudMap_MAX_POINT_NUM);
  ParamServer::Set("PointCloudMapGT_CELL_POINT_NUM_THRESH",
                   param::PointCloudMapGT_CELL_POINT_NUM_THRESH1);
  ParamServer::Set("PointCloudMapLP_ACC_DIST_THRESH",
                   param::PointCloudMapLP_ACC_DIST_THRESH);

  // NNGridTable
  ParamServer::Set("NNGridTable_MIN_DIST_THRESH",
                   param::NNGridTable_MIN_DIST_THRESH);
  ParamServer::Set("NNGridTable_CELL_SIZE", param::NNGridTable_CELL_SIZE);
  ParamServer::Set("NNGridTable_DOMAIN_SIZE", param::NNGridTable_DOMAIN_SIZE);

  // DataAssicator
  ParamServer::Set("DataAssociatorLS_DIST_THRESH",
                   param::DataAssociatorLS_DIST_THRESH);

  // CostFunction
  ParamServer::Set("CostFunction_VAL_THRESH", param::CostFunction_VAL_THRESH);

  // PoseOptimizer
  ParamServer::Set("PoseOptimizer_VAL_DIFF_THRESH",
                   param::PoseOptimizer_VAL_DIFF_THRESH);
  ParamServer::Set("PoseOptimizerSL_ITERATION",
                   param::PoseOptimizerSL_ITERATION);
  ParamServer::Set("PoseOptimizerSD_ITERATION",
                   param::PoseOptimizerSD_ITERATION);
  ParamServer::Set("PoseOptimizer_TICK_DIST", param::PoseOptimizer_TICK_DIST);
  ParamServer::Set("PoseOptimizer_TICK_THETA", param::PoseOptimizer_TICK_THETA);
  ParamServer::Set("PoseOptimizer_ERROR_THRESH",
                   param::PoseOptimizer_ERROR_THRESH);
  ParamServer::Set("PoseOptimizer_DESCENT_COEFF",
                   param::PoseOptimizer_DESCENT_COEFF);
  ParamServer::Set("PoseOptimizer_SEARCH_RANGE",
                   param::PoseOptimizer_SEARCH_RANGE);
  ParamServer::Set("PoseEstimatorICP_VAL_DIFF_THRESH",
                   param::PoseEstimatorICP_VAL_DIFF_THRESH);
  // ParamServer::Set("PoseEstimatorICP_VAL_THRESH",
  //                 param::PoseEstimatorICP_VAL_THRESH);
  ParamServer::Set("PoseEstimatorICP_ITERATION",
                   param::PoseEstimatorICP_ITERATION);

  // ScanMatcher2D
  ParamServer::Set("ScanMatcher2D_SCORE_THRESH",
                   param::ScanMatcher2D_SCORE_THRESH);
  ParamServer::Set("ScanMatcher2D_USED_NUM_THRESH",
                   param::ScanMatcher2D_USED_NUM_THRESH);

  // ScanPointResampler
  ParamServer::Set("ScanPointResampler_DIST_INTERVAL",
                   param::ScanPointResampler_DIST_INTERVAL);
  ParamServer::Set("ScanPointResampler_DIST_INTERPOLATE_THRESH",
                   param::ScanPointResampler_DIST_INTERPOLATE_THRESH);

  // ScanPointAnalyser
  ParamServer::Set("ScanPointAnalyser_FPDMIN", param::ScanPointAnalyser_FPDMIN);
  ParamServer::Set("ScanPointAnalyser_FPDMAX", param::ScanPointAnalyser_FPDMAX);
  ParamServer::Set("ScanPointAnalyser_CORNER_DEG_THRESH",
                   param::ScanPointAnalyser_CORNER_DEG_THRESH);
  ParamServer::Set("ScanPointAnalyser_INVALID_DEG",
                   param::ScanPointAnalyser_INVALID_DEG);
  ParamServer::Set("ScanPointAnalyser_COS_THRESH",
                   param::ScanPointAnalyser_COS_THRESH);

  // CovarianceCalculator
  ParamServer::Set("CovarianceCalculator_TICK_DIST",
                   param::CovarianceCalculator_TICK_DIST);
  ParamServer::Set("CovarianceCalculator_TICK_THETA",
                   param::CovarianceCalculator_TICK_THETA);
  ParamServer::Set("CovarianceCalculator_ALPHA1",
                   param::CovarianceCalculator_ALPHA1);
  ParamServer::Set("CovarianceCalculator_ALPHA2",
                   param::CovarianceCalculator_ALPHA2);
  ParamServer::Set("CovarianceCalculator_ICP_COV_SCALE1",
                   param::CovarianceCalculator_ICP_COV_SCALE1);
  ParamServer::Set("CovarianceCalculator_VEL_THRESH1",
                   param::CovarianceCalculator_VEL_THRESH1);
  ParamServer::Set("CovarianceCalculator_OMEGA_THRESH1",
                   param::CovarianceCalculator_OMEGA_THRESH1);
  ParamServer::Set("CovarianceCalculator_MCOV_COEFF_X",
                   param::CovarianceCalculator_MCOV_COEFF_X);
  ParamServer::Set("CovarianceCalculator_MCOV_COEFF_Y",
                   param::CovarianceCalculator_MCOV_COEFF_Y);
  ParamServer::Set("CovarianceCalculator_MCOV_COEFF_TH",
                   param::CovarianceCalculator_MCOV_COEFF_TH);
  ParamServer::Set("CovarianceCalculator_ICP_COV_SCALE2",
                   param::CovarianceCalculator_ICP_COV_SCALE2);
  ParamServer::Set("CovarianceCalculator_VEL_THRESH2",
                   param::CovarianceCalculator_VEL_THRESH2);
  ParamServer::Set("CovarianceCalculator_OMEGA_THRESH2",
                   param::CovarianceCalculator_OMEGA_THRESH2);

  // PoseGraph
  ParamServer::Set("PoseGraph_POOL_SIZE", param::PoseGraph_POOL_SIZE);

  // SlamBackEnd
  ParamServer::Set("SlamBackEnd_P2O_N", param::SlamBackEnd_P2O_N);

  // LoopDetectorSS
  ParamServer::Set("LoopDetectorSS_RADIUS", param::LoopDetectorSS_RADIUS);
  ParamServer::Set("LoopDetectorSS_ACC_DIST_THRESH",
                   param::LoopDetectorSS_ACC_DIST_THRESH);
  ParamServer::Set("LoopDetectorSS_SCORE_THRESH",
                   param::LoopDetectorSS_SCORE_THRESH);
  ParamServer::Set("LoopDetectorSS_USED_NUM_MIN",
                   param::LoopDetectorSS_USED_NUM_MIN);
  ParamServer::Set("LoopDetectorSS_RANGE_T", param::LoopDetectorSS_RANGE_T);
  ParamServer::Set("LoopDetectorSS_RANGE_A", param::LoopDetectorSS_RANGE_A);
  ParamServer::Set("LoopDetectorSS_DD", param::LoopDetectorSS_DD);
  ParamServer::Set("LoopDetectorSS_DA", param::LoopDetectorSS_DA);
  ParamServer::Set("LoopDetectorSS_MATCH_THRESH",
                   param::LoopDetectorSS_MATCH_THRESH);
  ParamServer::Set("LoopDetectorSS_MATCH_RATIO_THRESH",
                   param::LoopDetectorSS_MATCH_RATIO_THRESH);
}

} /* namespace slam */
