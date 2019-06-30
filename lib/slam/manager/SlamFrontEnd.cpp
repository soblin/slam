#include <slam/manager/SlamFrontEnd.h>

namespace slam {

void SlamFrontEnd::Process(Scan2D &scan) {
  PointCloudMap *cloud_map_ptr = PointCloudMapSingleton::GetCloudMap();

  int cnt = 0;

  m_scan_matcher_ptr->MatchScan(scan);
  // get the estimated current pose with ICP
  //  Pose2D curPose = m_point_cloud_map_ptr->GetLastPose();
  cloud_map_ptr->MakeGlobalMap();

  cnt += 1;
}

} /* namespace slam */
