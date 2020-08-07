#ifndef REF_SCAN_MAKER_H
#define REF_SCAN_MAKER_H

#include <slam/geometry/PointCloudMap.h>
#include <slam/geometry/Scan2D.h>

namespace slam {

class RefScanMaker {
public:
  RefScanMaker() {}
  virtual ~RefScanMaker() {}

  virtual void Initialize() = 0;
  virtual const Scan2D *MakeRefScan() = 0;

protected:
  Scan2D m_ref_scan; // provide this ref scan data to clients
  PointCloudMap *m_cloud_map_ptr = nullptr;
};

} /* namespace slam */
#endif /* REF_SCAN_MAKER_H */
