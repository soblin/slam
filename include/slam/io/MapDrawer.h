#ifndef MAP_DRAWER_H
#define MAP_DRAWER_H

#include <stdio.h>
#include <vector>

#include <slam/geometry/PointCloudMap.h>
#include <slam/geometry/Pose2D.h>
#include <slam/geometry/Scan2D.h>

namespace slam {

class MapDrawer {
public:
  MapDrawer() {}
  ~MapDrawer() { FinishGnuplot(); }

  void Initialize();
  void FinishGnuplot();
  void SetAspectRatio(double a);
  void SetRange(double R);
  void SetRange(double xR, double yR);
  void SetRange(double xm, double xM, double ym, double yM);
  void DrawScanGp(const Scan2D &scan);
  void DrawTrajectoryGp(const std::vector<Pose2D> &poses);
  void DrawGp(const std::vector<ScanPoint2D> &scaned_points,
              const std::vector<Pose2D> &poses, bool flush = true);
  void DrawGp(const PointCloudMap *pcmap);

private:
  FILE *m_gp = nullptr;
  double m_xmin = -10, m_xmax = 10;
  double m_ymin = -10, m_ymax = 10;
  double m_ratio = -1.0;
  double m_step_point = 0;
  double m_step_pose = 0;
  double m_dd = 0;
};

} // namespace slam

#endif
