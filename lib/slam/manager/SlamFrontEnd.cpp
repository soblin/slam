#include <slam/manager/SlamFrontEnd.h>

namespace slam {

void SlamFrontEnd::Process(const Scan2D &scan) {
  m_scan_matcher_ptr->MatchScan(scan);
  // get the estimated current pose with ICP
  //  Pose2D curPose = m_point_cloud_map_ptr->GetLastPose();
}

} /* namespace slam */
