#include <slam/icp/RefScanMakerBS.h>

namespace slam {

const Scan2D *RefScanMakerBS::MakeRefScan() {
  std::vector<ScanPoint2D> newScan;

  Pose2D lastPose;
  m_point_cloud_map->GetLastPose(lastPose);
  double tx = lastPose.tx();
  double ty = lastPose.ty();

  const std::vector<ScanPoint2D> &scaned_points =
      m_point_cloud_map->GetLastScan().scaned_points();
  for (unsigned i = 0; i < scaned_points.size(); ++i) {
    const ScanPoint2D &scan_point = scaned_points[i];

    ScanPoint2D point;
    point.x() =
        lastPose.R00() * scan_point.x() + lastPose.R01() * scan_point.y() + tx;
    point.y() =
        lastPose.R10() * scan_point.x() + lastPose.R11() * scan_point.y() + ty;
    point.nx() =
        lastPose.R00() * scan_point.nx() + lastPose.R01() * scan_point.ny();
    point.ny() =
        lastPose.R10() * scan_point.nx() + lastPose.R11() * scan_point.ny();
    // point = lastPose.ToGlobalPoint(scan_point);
    newScan.emplace_back(point);
  }

  m_ref_scan.SetScanedPoints(newScan);

  return &m_ref_scan;
}

} /* namespace slam */
