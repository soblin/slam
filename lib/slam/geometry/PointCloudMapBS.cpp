#include <slam/geometry/PointCloudMapBS.h>
#include <slam/manager/ParamServer.h>
#include <utility>

namespace slam {

void PointCloudMapBS::AddPose(const Pose2D &pose) { m_poses.push_back(pose); }

void PointCloudMapBS::AddPoint(const ScanPoint2D &scan) {
  m_global_map.push_back(scan);
}

void PointCloudMapBS::AddPoint(ScanPoint2D &&scan) {
  m_global_map.emplace_back(std::forward<ScanPoint2D>(scan));
}

void PointCloudMapBS::AddPoints(const std::vector<ScanPoint2D> &scans) {
  static double skip = ParamServer::Get("PointCloudMapBS_SKIP");
  for (unsigned i = 0; i < scans.size(); i += skip) {
    m_global_map.push_back(scans[i]);
  }
}

void PointCloudMapBS::MakeGlobalMap() {}
void PointCloudMapBS::MakeLocalMap() {}
void PointCloudMapBS::RemakeMaps(const std::vector<Pose2D> &newposes) {}

} /* namespace slam */
