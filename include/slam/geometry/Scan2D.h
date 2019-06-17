#ifndef SCAN2D_H
#define SCAN2D_H

#include <slam/geometry/Pose2D.h>
#include <vector>

namespace slam {

class Scan2D {
private:
  Pose2D m_pose;
  std::vector<ScanPoint2D> m_scaned_points;

public:
  inline const std::vector<ScanPoint2D> &scaned_points() const {
    return m_scaned_points;
  }
  inline std::vector<ScanPoint2D> &scaned_points_ref() {
    return m_scaned_points;
  }
  inline double &tx() { return m_pose.tx(); }
  inline double &ty() { return m_pose.ty(); }
  inline const Pose2D &pose() const { return m_pose; }

public:
  Scan2D();
  ~Scan2D();
  void SetScanedPoints(const std::vector<ScanPoint2D> &scaned_points);
  void SetPose(const Pose2D &pose);
  void SetAngle(double rad);
};

} // namespace slam

#endif
