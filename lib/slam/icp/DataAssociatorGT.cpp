#include <slam/icp/DataAssociatorGT.h>

namespace slam {

void DataAssociatorGT::Initialize() {}

void DataAssociatorGT::SetRefBase(const std::vector<ScanPoint2D> &refPoints) {
  m_grid_table.Clear();
  for (const auto &refPoint : refPoints) {
    m_grid_table.AddPoint(&refPoint);
  }
}

// remap curScan to global-frame based on predictedPose and do scan matching
// and store the result to m_cur_points and m_ref_points respectively
double DataAssociatorGT::FindCorrespondence(const Scan2D *curScan,
                                            const Pose2D &predictedPose) {
  m_cur_points.clear();
  m_ref_points.clear();

  auto length = curScan->scaned_points().size();

  for (unsigned i = 0; i < length; ++i) {
    // query point
    const ScanPoint2D *cur_point_ptr = &(curScan->scaned_points()[i]);

    const auto nearest_point_iter =
        m_grid_table.FindClosestPoint(cur_point_ptr, predictedPose);

    if (nearest_point_iter != nullptr) {
      m_cur_points.emplace_back(cur_point_ptr);
      m_ref_points.emplace_back(nearest_point_iter);
    }
  }

  double ratio = (1.0 * m_cur_points.size()) / length;

  return ratio;
}

} // namespace slam
