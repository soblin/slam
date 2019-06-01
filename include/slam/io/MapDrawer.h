#ifndef MAP_DRAWER_H
#define MAP_DRAWER_H

#include <stdio.h>
#include <vector>

#include <slam/geometry/PointCloudMap.h>
#include <slam/geometry/Pose2D.h>
#include <slam/geometry/Scan2D.h>

namespace slam {

class MapDrawer {
private:
  FILE *m_gp;
  double m_xmin, m_xmax;
  double m_ymin, m_ymax;
  double m_ratio;

public:
  MapDrawer();
  ~MapDrawer();

  void InitGnuplot();
  void FinishGnuplot();
  void SetAspectRatio(double a);
  void SetRange(double R);
  void SetRange(double xR, double yR);
  void SetRange(double xm, double xM, double ym, double yM);
  void DrawScanGp(const Scan2D &scan);
  void DrawTrajectoryGp(const std::vector<Pose2D> &poses);
  void DrawGp(const std::vector<ScanPoint2D> &scaned_points,
              const std::vector<Pose2D> &poses, bool flush = true);
  void DrawGp(const PointCloudMap &pcmap);
};

} // namespace slam

#endif
