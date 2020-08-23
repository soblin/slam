#include <slam/geometry/PointCloudMapGT.h>
#include <utility>

namespace slam {

void PointCloudMapGT::Initialize() {
  int reserve_size = ParamServer::Get("PointCloudMap_MAX_POINT_NUM");
  m_global_map.reserve(reserve_size);

  m_grid_table.Initialize();
}

void PointCloudMapGT::AddPose(const Pose2D &p) { m_poses.emplace_back(p); }

void PointCloudMapGT::AddPoint(const ScanPoint2D &p) {
  m_all_points.emplace_back(p);
}

void PointCloudMapGT::AddPoint(ScanPoint2D &&p) {
  m_all_points.emplace_back(std::forward<ScanPoint2D>(p));
}

void PointCloudMapGT::SubsamplePoints(std::vector<ScanPoint2D> &subs) {
  m_grid_table.Clear();
  for (size_t i = 0; i < m_all_points.size(); ++i)
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
  // m_global_map = m_all_points;
}

void PointCloudMapGT::MakeLocalMap() { m_local_map = m_global_map; }

void PointCloudMapGT::RemakeMaps(const std::vector<Pose2D> &newPoses) {
  return;
}
} // namespace slam
