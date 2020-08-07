#ifndef REF_SCAN_MAKER_LM_H
#define REF_SCAN_MAKER_LM_H

#include <slam/icp/RefScanMaker.h>

namespace slam {

class RefScanMakerLM : public RefScanMaker {
public:
  RefScanMakerLM() : RefScanMaker() {}
  virtual ~RefScanMakerLM(){};

  virtual void Initialize() override;
  virtual const Scan2D *MakeRefScan() override;
};

} // namespace slam
#endif /* REF_SCAN_MAKER_LM_H */
