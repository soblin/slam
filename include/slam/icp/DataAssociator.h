#ifndef DATA_ASSOCIATOR_H
#define DATA_ASSOCIATOR_H

#include <slam/geometry/Pose2D.h>
#include <slam/geometry/Scan2D.h>
#include <slam/geometry/ScanPoint2D.h>
#include <vector>

namespace slam {

class DataAssociator {
protected:
  std::vector<const ScanPoint2D *> m_cur_points;
  std::vector<const ScanPoint2D *> m_ref_points;

public:
  inline std::vector<const ScanPoint2D *> &cur_points() { return m_cur_points; }
  inline std::vector<const ScanPoint2D *> &ref_points() { return m_ref_points; }

public:
  DataAssociator() {}
  ~DataAssociator() {}

  virtual void SetRefBase(const std::vector<ScanPoint2D> &points) = 0;
  virtual double FindCorrespondence(const Scan2D *curScanPtr,
                                    const Pose2D &predictedPose) = 0;
};

} /* namespace slam */
#endif /* data_associator_h */
