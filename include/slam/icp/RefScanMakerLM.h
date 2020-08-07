#ifndef REF_SCAN_MAKER_LM_H
#define REF_SCAN_MAKER_LM_H

#include <slam/icp/RefScanMaker.h>

namespace slam {

class RefScanMakerLM : public RefScanMaker {
public:
  RefScanMakerLM() : RefScanMaker() {}
  virtual ~RefScanMakerLM(){};

  virtual const Scan2D *MakeRefScan() override;
  virtual void Initialize() override;
};

} // namespace slam
#endif /* REF_SCAN_MAKER_LM_H */
