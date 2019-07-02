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
