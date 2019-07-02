#include <slam/manager/CounterServer.h>
#include <slam/manager/ParamServer.h>
#include <slam/manager/SlamFrontEnd.h>

namespace slam {

void SlamFrontEnd::Init() {
  CounterServer::Create();
  ParamServer::Create();
  ParamServer::Set("Scan2D_MAX_SCAN_RANGE", 6.0);
  ParamServer::Set("Scan2D_MIN_SCAN_RANGE", 0.1);
  ParamServer::Set("PointCloudMapBS_SKIP", 5.0);
  ParamServer::Set("NNGridTable_MIN_DIST_THRESH", 0.2);
  ParamServer::Set("PointCloudMapGT_CELL_POINT_NUM_THRESH", 1.0);
  ParamServer::Set("MapDrawer_STEP_POINT", 1.0);
  ParamServer::Set("MapDrawer_STEP_POSE", 10);
  ParamServer::Set("MapDrawer_DD", 0.4);
  ParamServer::Set("SlamLauncher_SLEEP_TIME", 100000);
  ParamServer::Set("SlamLauncher_PLOT_SKIP", 10);
}

void SlamFrontEnd::Process(Scan2D &scan) {
  PointCloudMap *cloud_map_ptr = PointCloudMapSingleton::GetCloudMap();

  int cnt = CounterServer::Get();
  if (cnt == 0)
    Init();

  m_scan_matcher_ptr->MatchScan(scan);
  // get the estimated current pose with ICP
  //  Pose2D curPose = m_point_cloud_map_ptr->GetLastPose();
  cloud_map_ptr->MakeGlobalMap();

  CounterServer::Increment();
}

} /* namespace slam */
