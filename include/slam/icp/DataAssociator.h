#ifndef DATA_ASSOCIATOR_H
#define DATA_ASSOCIATOR_H

#include <slam/geometry/Pose2D.h>
#include <slam/geometry/Scan2D.h>
#include <slam/geometry/ScanPoint2D.h>
#include <vector>

namespace slam {

class DataAssociator {
public:
  DataAssociator() {}
  ~DataAssociator() {}

  virtual void Initialize() = 0;
  virtual void SetRefBase(const std::vector<ScanPoint2D> &points) = 0;
  virtual double FindCorrespondence(const Scan2D *curScanPtr,
                                    const Pose2D &predictedPose) = 0;

public:
  inline std::vector<const ScanPoint2D *> cur_points() { return m_cur_points; }
  inline std::vector<const ScanPoint2D *> ref_points() { return m_ref_points; }
  inline const std::vector<const ScanPoint2D *> &cur_points_ref() {
    return m_cur_points;
  }
  inline const std::vector<const ScanPoint2D *> &ref_points_ref() {
    return m_ref_points;
  }

protected:
  std::vector<const ScanPoint2D *> m_cur_points; // current scan, local-frame
  std::vector<const ScanPoint2D *> m_ref_points; // global-frame
};

} /* namespace slam */
#endif /* DATA_ASSOCIATOR_H */
