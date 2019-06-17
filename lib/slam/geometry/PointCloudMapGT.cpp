#include <slam/geometry/PointCloudMapGT.h>

namespace slam {

void PointCloudMapGT::AddPose(const Pose2D &p) { m_poses.emplace_back(p); }

void PointCloudMapGT::SubsamplePoints(std::vector<ScanPoint2D> &subs) {
  m_grid_table.Clear();
  for (unsigned i = 0; i < m_all_points.size(); ++i)
    m_grid_table.AddPoint(&m_all_points[i]);

  m_grid_table.MakeCellPoints(subs);
}

void PointCloudMapGT::AddPoints(const std::vector<ScanPoint2D> &points) {
  for (const auto point : points)
    m_all_points.emplace_back(point);
}

void PointCloudMapGT::MakeGlobalMap() {
  m_global_map.clear();
  SubsamplePoints(m_global_map);
}

void PointCloudMapGT::MakeLocalMap() { m_local_map = m_global_map; }

void PointCloudMapGT::RemakeMaps(const std::vector<Pose2D> &newPoses) {
  return;
}

} // namespace slam
