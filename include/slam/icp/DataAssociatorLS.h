#ifndef DATA_ASSOCIATOR_LS_H
#define DATA_ASSOCIATOR_LS_H

#include <slam/icp/DataAssociator.h>

namespace slam {

// do matching between current scan `m_cur_points`(local frame) and reference
// scan `m_ref_points`(global frame)
class DataAssociatorLS : public DataAssociator {
public:
  DataAssociatorLS() : DataAssociator() {}
  virtual ~DataAssociatorLS() {}

  virtual void Initialize() override;
  virtual void SetRefBase(const std::vector<ScanPoint2D> &points) override;
  virtual double FindCorrespondence(const Scan2D *curScanPtr,
                                    const Pose2D &predictedPose) override;

  double FindCorrespondence(const Scan2D *curScanPtr,
                            const Pose2D &predictedPose, double thresh);

private:
  std::vector<const ScanPoint2D *> m_base_points; // use as temporary storage

public:
  friend class DataAssociatorLSTestFriend;
};

} /* namespace slam */
#endif /* DATA_ASSOCIATOR_LS_H */
