#include <slam/io/SlamFrontEnd.h>

namespace slam {

void SlamFrontEnd::Init() {
  m_scan_matcher_ptr->Reset();
  m_scan_matcher_ptr->SetPointCloudMap(m_point_cloud_map_ptr);
}

void SlamFrontEnd::Process(Scan2D &scan) {
  if (m_cnt == 0)
    Init();

  m_scan_matcher_ptr->MatchScan(scan);

  Pose2D curPose = m_point_cloud_map_ptr->GetLastPose();

  ++m_cnt;
}

} /* namespace slam */
