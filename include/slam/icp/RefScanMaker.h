#ifndef REF_SCAN_MAKER_H
#define REF_SCAN_MAKER_H

#include <slam/geometry/PointCloudMap.h>
#include <slam/geometry/Scan2D.h>

namespace slam {

class RefScanMaker {
protected:
  const PointCloudMap *m_point_cloud_map; // pointer to the global map
  Scan2D m_ref_scan; // provide this ref scan data to clients

public:
  inline void SetPointCloudMap(const PointCloudMap *p) {
    m_point_cloud_map = p;
  }

public:
  RefScanMaker() : m_point_cloud_map(nullptr) {}
  ~RefScanMaker() {}

  virtual const Scan2D *MakeRefScan() = 0;
};

} /* namespace slam */
#endif /* ref_scan_maker_h */
