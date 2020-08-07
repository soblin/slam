#include <slam/geometry/PointCloudMap.h>
#include <slam/icp/RefScanMakerLM.h>

namespace slam {

const Scan2D *RefScanMakerLM::MakeRefScan() {
  std::vector<ScanPoint2D> refPoints;

  const auto &localMap = m_cloud_map_ptr->local_map();

  for (unsigned i = 0; i < localMap.size(); ++i) {
    const ScanPoint2D &point = localMap[i];
    refPoints.emplace_back(point);
  }

  m_ref_scan.SetScanedPoints(refPoints);
  return &m_ref_scan;
}

void RefScanMakerLM::Initialize() {
  m_cloud_map_ptr = PointCloudMapSingleton::GetCloudMap();
}

} // namespace slam
