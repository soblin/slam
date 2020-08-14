#include <cassert>
#include <cmath>
#include <slam/manager/CounterServer.h>
#include <slam/manager/ParamServer.h>
#include <slam/manager/SlamFrontEnd.h>
#include <slam/parameters.h>

namespace slam {

void SlamFrontEnd::Initialize() {
  CounterServer::Create();

  ParamServer::Create();
  RegisterParams();

  m_cloud_map_ptr = PointCloudMapSingleton::GetCloudMap();
  m_cloud_map_ptr->Initialize();

  // Initialize the chained classes and update their parameters
  assert(m_scan_matcher_ptr != nullptr);
  m_scan_matcher_ptr->Initialize();
}

// process the scan data, which was generated at SensorDataReader
void SlamFrontEnd::Process(Scan2D &scan) {
  int cnt = CounterServer::Get();

  m_scan_matcher_ptr->MatchScan(scan);
  // get the estimated current pose with ICP
  //  Pose2D curPose = m_point_m_cloud_map_ptr->GetLastPose();
  if (cnt == 0)
    ParamServer::Set("PointCloudMapGT_CELL_POINT_NUM_THRESH", 1.0);
  else
    ParamServer::Set("PointCloudMapGT_CELL_POINT_NUM_THRESH", 5.0);

  m_cloud_map_ptr->MakeGlobalMap();

  CounterServer::Increment();
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

  // PointCloudMao
  ParamServer::Set("PointCloudMapBS_SKIP", param::PointCloudMapBS_SKIP);
  ParamServer::Set("PointCloudMap_MAX_POINT_NUM",
                   param::PointCloudMap_MAX_POINT_NUM);
  ParamServer::Set("PointCloudMapGT_CELL_POINT_NUM_THRESH",
                   param::PointCloudMapGT_CELL_POINT_NUM_THRESH1);

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
  ParamServer::Set("PoseOptimizer_TICK_DIST", param::PoseOptimizer_TICK_THETA);
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
}

} /* namespace slam */
