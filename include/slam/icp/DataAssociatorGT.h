#ifndef DATA_ASSOCIATOR_GT_H
#define DATA_ASSOCIATOR_GT_H

#include <slam/geometry/NNGridTable.h>
#include <slam/icp/DataAssociator.h>

namespace slam {

class DataAssociatorGT : public DataAssociator {
private:
  NNGridTable m_grid_table;

public:
  DataAssociatorGT() {}
  virtual ~DataAssociatorGT() { m_grid_table.Clear(); }

  // store the points of RefScanMaker::m_cur_scan to GridTable.
  virtual void SetRefBase(const std::vector<ScanPoint2D> &refPoints) override;

  virtual double FindCorrespondence(const Scan2D *curScan,
                                    const Pose2D &predictedPose) override;
};

} // namespace slam
#endif /* DATA_ASSOCIATOR_GT_H */
