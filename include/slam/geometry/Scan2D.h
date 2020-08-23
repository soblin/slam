#ifndef SCAN2D_H
#define SCAN2D_H

#include <slam/geometry/Pose2D.h>
#include <vector>

namespace slam {

class Scan2D {
public:
  Scan2D() {}
  ~Scan2D() { m_scaned_points.reserve(0); }

  void SetScanedPoints(const std::vector<ScanPoint2D> &scaned_points);
  void SetPose(const Pose2D &pose);
  void SetAngle(double rad);
  void SetId(int id);

public:
  const std::vector<ScanPoint2D> &scaned_points() const;
  std::vector<ScanPoint2D> &scaned_points_ref();
  double &tx();
  double &ty();
  const Pose2D &pose() const;
  int id() const;

private:
  Pose2D m_pose;
  std::vector<ScanPoint2D> m_scaned_points;
  int m_id;
};

inline const std::vector<ScanPoint2D> &Scan2D::scaned_points() const {
  return m_scaned_points;
}
inline std::vector<ScanPoint2D> &Scan2D::scaned_points_ref() {
  return m_scaned_points;
}
inline double &Scan2D::tx() { return m_pose.tx(); }
inline double &Scan2D::ty() { return m_pose.ty(); }
inline const Pose2D &Scan2D::pose() const { return m_pose; }
inline int Scan2D::id() const { return m_id; }

} // namespace slam

#endif
