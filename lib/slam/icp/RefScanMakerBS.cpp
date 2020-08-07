#include <slam/geometry/PointCloudMap.h>
#include <slam/icp/RefScanMakerBS.h>

namespace slam {

const Scan2D *RefScanMakerBS::MakeRefScan() {
  std::vector<ScanPoint2D> newScan;

  Pose2D lastPose;
  m_cloud_map_ptr->GetLastPose(lastPose);
  double tx = lastPose.tx();
  double ty = lastPose.ty();

  const std::vector<ScanPoint2D> &scaned_points =
      m_cloud_map_ptr->GetLastScan().scaned_points();
  for (unsigned i = 0; i < scaned_points.size(); ++i) {
    const ScanPoint2D &scan_point = scaned_points[i];

    double x =
        lastPose.R00() * scan_point.x() + lastPose.R01() * scan_point.y() + tx;
    double y =
        lastPose.R10() * scan_point.x() + lastPose.R11() * scan_point.y() + ty;
    double nx =
        lastPose.R00() * scan_point.nx() + lastPose.R01() * scan_point.ny();
    double ny =
        lastPose.R10() * scan_point.nx() + lastPose.R11() * scan_point.ny();

    newScan.emplace_back(x, y, nx, ny, ScanPoint2D::PointType::UNKNOWN);
  }

  m_ref_scan.SetScanedPoints(newScan);

  return &m_ref_scan;
}

void RefScanMakerBS::Initialize() {
  m_cloud_map_ptr = PointCloudMapSingleton::GetCloudMap();
}

} /* namespace slam */
