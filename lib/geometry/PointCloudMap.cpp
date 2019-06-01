#include <slam/geometry/PointCloudMap.h>

namespace slam {

PointCloudMap::PointCloudMap() { m_global_map.reserve(MAX_POINT_NUM); }

PointCloudMap::~PointCloudMap() {
  m_poses.reserve(0);
  m_global_map.reserve(0);
}

void PointCloudMap::AddPose(const Pose2D &pose) { m_poses.push_back(pose); }

void PointCloudMap::AddPoint(const ScanPoint2D &scan) {
  m_global_map.push_back(scan);
}

void PointCloudMap::AddPoints(const std::vector<ScanPoint2D> &scans) {
  int skip = 5;
  for (unsigned i = 0; i < scans.size(); i += skip) {
    m_global_map.push_back(scans[i]);
  }
}

} /* namespace slam */
