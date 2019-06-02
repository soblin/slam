#ifndef REF_SCAN_MAKER_BS_H
#define REF_SCAN_MAKER_BS_H

#include <slam/icp/RefScanMaker.h>
namespace slam {

class RefScanMakerBS : public RefScanMaker {
public:
  RefScanMakerBS() : RefScanMaker() {}
  virtual ~RefScanMakerBS() {}

  virtual const Scan2D *MakeRefScan() override;
};

} /* namespace slam */
#endif /* ref_scan_maker_bs_h */
