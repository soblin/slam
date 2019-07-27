#include <slam/manager/CounterServer.h>
#include <slam/manager/ParamServer.h>
#include <slam/manager/SlamFrontEnd.h>
#include <slam/parameters.h>

namespace slam {

void SlamFrontEnd::Init() {
  CounterServer::Create();

  ParamServer::Create();
  ParamServer::Set("Scan2D_MAX_SCAN_RANGE", param::Scan2D_MAX_SCAN_RANGE);
  ParamServer::Set("Scan2D_MIN_SCAN_RANGE", param::Scan2D_MIN_SCAN_RANGE);
  ParamServer::Set("PointCloudMapBS_SKIP", param::PointCloudMapBS_SKIP);
  ParamServer::Set("PointCloudMap_MAX_POINT_NUM",
                   param::PointCloudMap_MAX_POINT_NUM);
  ParamServer::Set("PointCloudMapGT_CELL_POINT_NUM_THRESH",
                   param::PointCloudMapGT_CELL_POINT_NUM_THRESH1);
  ParamServer::Set("SensorDataReader_ANGLE_OFFSET",
                   param::SensorDataReader_ANGLE_OFFSET);
  ParamServer::Set("MapDrawer_STEP_POINT", param::MapDrawer_STEP_POINT);
  ParamServer::Set("MapDrawer_STEP_POSE", param::MapDrawer_STEP_POSE);
  ParamServer::Set("MapDrawer_DD", param::MapDrawer_DD);
  ParamServer::Set("SlamLauncher_SLEEP_TIME", param::SlamLauncher_SLEEP_TIME);
  ParamServer::Set("SlamLauncher_PLOT_SKIP", param::SlamLauncher_PLOT_SKIP);
  ParamServer::Set("DataAssociatorLS_DIST_THRESH",
                   param::DataAssociatorLS_DIST_THRESH);
  ParamServer::Set("CostFunction_VAL_THRESH", param::CostFunction_VAL_THRESH);
  ParamServer::Set("PoseOptimizer_VAL_DIFF_THRESH",
                   param::PoseOptimizer_VAL_DIFF_THRESH);
  ParamServer::Set("PoseOptimizer_TickDist", param::PoseOptimizer_TickDist);
  ParamServer::Set("oseOptimizer_TickTheta", param::PoseOptimizer_TickTheta);
  ParamServer::Set("PoseOptimizer_ERROR_THRESH",
                   param::PoseOptimizer_ERROR_THRESH);
  ParamServer::Set("PoseOptimizer_DescentCoeff",
                   param::PoseOptimizer_DescentCoeff);
  ParamServer::Set("PoseEstimatorICP_VAL_DIFF_THRESH",
                   param::PoseEstimatorICP_VAL_DIFF_THRESH);
  ParamServer::Set("PoseEstimatorICP_VAL_THRESH",
                   param::PoseEstimatorICP_VAL_THRESH);
  ParamServer::Set("ScanMatcher2D_SCORE_THRESH",
                   param::ScanMatcher2D_SCORE_THRESH);
  ParamServer::Set("ScanMatcher2D_USED_NUM_THRESH",
                   param::ScanMatcher2D_USED_NUM_THRESH);
  ParamServer::Set("ScanPointResampler_DIST_INTERVAL",
                   param::ScanPointResampler_DIST_INTERVAL);
  ParamServer::Set("ScanPointResampler_DIST_INTERPOLATE_THRESH",
                   param::ScanPointResampler_DIST_INTERPOLATE_THRESH);
  ParamServer::Set("ScanPointAnalyser_FPDMIN", param::ScanPointAnalyser_FPDMIN);
  ParamServer::Set("ScanPointAnalyser_FPDMAX", param::ScanPointAnalyser_FPDMAX);
  ParamServer::Set("ScanPointAnalyser_CORNER_DEG_THRESH",
                   param::ScanPointAnalyser_CORNER_DEG_THRESH);
  ParamServer::Set("ScanPointAnalyser_INVALID_DEG",
                   param::ScanPointAnalyser_INVALID_DEG);
  ParamServer::Set("ScanPointAnalyer_CORNER_COS_THRESH",
                   param::ScanPointAnalyer_CORNER_COS_THRESH);

  ParamServer::Set("NNGridTable_MIN_DIST_THRESH",
                   param::NNGridTable_MIN_DIST_THRESH);
  ParamServer::Set("NNGridTable_CELL_SIZE", param::NNGridTable_CELL_SIZE);
  ParamServer::Set("NNGridTable_DOMAIN_SIZE", param::NNGridTable_DOMAIN_SIZE);
  ParamServer::Set("PoseOptimizer_SEARCH_RANGE",
                   param::PoseOptimizer_SEARCH_RANGE);
  ParamServer::Set("PoseOptimizerSL_ITERATION",
                   param::PoseOptimizerSL_ITERATION);
  ParamServer::Set("PoseEstimatorICP_ITERATION",
                   param::PoseEstimatorICP_ITERATION);
}

// process the scan data, which was generated at SensorDataReader
void SlamFrontEnd::Process(Scan2D &scan) {
  PointCloudMap *cloud_map_ptr = PointCloudMapSingleton::GetCloudMap();

  int cnt = CounterServer::Get();
  if (cnt == 0)
    Init();

  m_scan_matcher_ptr->MatchScan(scan);
  // get the estimated current pose with ICP
  //  Pose2D curPose = m_point_cloud_map_ptr->GetLastPose();
  if (cnt == 0)
    ParamServer::Set("PointCloudMapGT_CELL_POINT_NUM_THRESH", 1.0);
  else
    ParamServer::Set("PointCloudMapGT_CELL_POINT_NUM_THRESH", 5.0);

  cloud_map_ptr->MakeGlobalMap();

  CounterServer::Increment();
}

} /* namespace slam */
