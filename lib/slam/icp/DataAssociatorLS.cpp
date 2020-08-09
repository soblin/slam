#include <cmath>
#include <slam/geometry/ScanPoint2D.h>
#include <slam/icp/DataAssociatorLS.h>
#include <slam/manager/ParamServer.h>

namespace slam {

void DataAssociatorLS::Initialize() {}

void DataAssociatorLS::SetRefBase(const std::vector<ScanPoint2D> &points) {
  m_base_points.clear();
  for (unsigned i = 0; i < points.size(); ++i) {
    m_base_points.emplace_back(&points[i]);
  }
}

double DataAssociatorLS::FindCorrespondence(const slam::Scan2D *curScanPtr,
                                            const slam::Pose2D &predictedPose) {
  // for each scaned point in curScanPtr, find its nearest neighbour from
  // m_base_points
  // note: scaned point in curScanPtr is in local frame (of predictedPose)
  // while each point in m_base_points is in global frame
  static const double dist_thresh =
      ParamServer::Get("DataAssociatorLS_DIST_THRESH");
  return FindCorrespondence(curScanPtr, predictedPose, dist_thresh);
}

double DataAssociatorLS::FindCorrespondence(const Scan2D *curScanPtr,
                                            const Pose2D &predictedPose,
                                            double thresh) {

  // for each scaned point in curScanPtr, find its nearest neighbour from
  // m_base_points
  // note: scaned point in curScanPtr is in local frame (of predictedPose)
  // while each point in m_base_points is in global frame

  m_cur_points.clear();
  m_ref_points.clear();

  for (unsigned i = 0; i < curScanPtr->scaned_points().size(); ++i) {
    // the scan point of interest
    const ScanPoint2D *point_of_interest_local =
        &(curScanPtr->scaned_points()[i]);
    ScanPoint2D point_of_interest;
    // convert to global frame
    predictedPose.ToGlobalPoint(*point_of_interest_local, point_of_interest);

    double min_dist = HUGE_VAL;
    const ScanPoint2D *nearest_point_ptr = nullptr;
    // find the point that is nearest to point_of_interest within m_base_points
    for (const auto ref_point_iter : m_base_points) {
      double dx = (ref_point_iter->x() - point_of_interest.x());
      double dy = (ref_point_iter->y() - point_of_interest.y());
      double dist = std::hypot(dx, dy);
      if (dist < thresh * thresh && dist < min_dist) {
        min_dist = dist;
        nearest_point_ptr = ref_point_iter;
      }
    }
    if (nearest_point_ptr != nullptr) {
      m_cur_points.emplace_back(point_of_interest_local);
      m_ref_points.emplace_back(nearest_point_ptr);
    }
  }

  double ratio =
      (1.0 * m_cur_points.size()) / (curScanPtr->scaned_points().size());

  return ratio;
}

} /* namespace slam */
