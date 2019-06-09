#include <slam/geometry/PointCloudMapBS.h>
#include <slam/parameters.h>

namespace slam {

PointCloudMapBS::PointCloudMapBS() : PointCloudMap() {}

PointCloudMapBS::~PointCloudMapBS() {
  m_poses.reserve(0);
  m_global_map.reserve(0);
}

void PointCloudMapBS::AddPose(const Pose2D &pose) { m_poses.push_back(pose); }

void PointCloudMapBS::AddPoint(const ScanPoint2D &scan) {
  m_global_map.push_back(scan);
}

void PointCloudMapBS::AddPoints(const std::vector<ScanPoint2D> &scans) {
  for (unsigned i = 0; i < scans.size(); i += param::PointCloudMapBS_SKIP) {
    m_global_map.push_back(scans[i]);
  }
}

void PointCloudMapBS::MakeGlobalMap() {}
void PointCloudMapBS::MakeLocalMap() {}
void PointCloudMapBS::RemakeMaps(const std::vector<Pose2D> &newposes) {}

} /* namespace slam */
