#ifndef DATA_ASSOCIATOR_LS_H
#define DATA_ASSOCIATOR_LS_H

#include <slam/icp/DataAssociator.h>

namespace slam {

// do matching between current scan `m_cur_points` and reference scan
// m_ref_points
class DataAssociatorLS : public DataAssociator {
private:
  std::vector<const ScanPoint2D *> m_base_points; // use as temporary storage
  // Impl of FindCorrespondence
  double FindCorrespondence(const Scan2D *curScanPtr,
                            const Pose2D &predictedPose, double thresh);

public:
  DataAssociatorLS() : DataAssociator() {}
  virtual ~DataAssociatorLS() {}

  virtual void SetRefBase(const std::vector<ScanPoint2D> &points) override;
  virtual double FindCorrespondence(const Scan2D *curScanPtr,
                                    const Pose2D &predictedPose) override;
  virtual void Initialize() override;

  friend class DataAssociatorLSTestFriend;
};

} /* namespace slam */
#endif /* data_associator_ls_h */
