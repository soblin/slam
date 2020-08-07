#ifndef REF_SCAN_MAKER_H
#define REF_SCAN_MAKER_H

#include <slam/geometry/PointCloudMap.h>
#include <slam/geometry/Scan2D.h>

namespace slam {

class RefScanMaker {
protected:
  Scan2D m_ref_scan; // provide this ref scan data to clients
  PointCloudMap *m_cloud_map_ptr = nullptr;

public:
  RefScanMaker() {}
  virtual ~RefScanMaker() {}

  virtual const Scan2D *MakeRefScan() = 0;
  virtual void Initialize() = 0;
};

} /* namespace slam */
#endif /* ref_scan_maker_h */
