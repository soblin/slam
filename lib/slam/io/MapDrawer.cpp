#include <iostream>
#include <slam/geometry/PointCloudMap.h>
#include <slam/io/MapDrawer.h>
#include <slam/manager/ParamServer.h>

#ifdef DEBUG
static constexpr bool logger = true;
#else
static constexpr bool logger = false;
#endif

namespace slam {

void MapDrawer::Initialize() {
#ifdef __linux__
  m_gp = popen("gnuplot", "w");
#elif _WIN32
  m_gp = _popen("gnuplot", "w");
#endif
  m_step_point = static_cast<int>(ParamServer::Get("MapDrawer_STEP_POINT"));
  m_step_pose = static_cast<int>(ParamServer::Get("MapDrawer_STEP_POSE"));
  m_dd = ParamServer::Get("MapDrawer_DD");
}

void MapDrawer::FinishGnuplot() {
  if (m_gp != nullptr) {
#ifdef __linux__
    pclose(m_gp);
#elif _WIN32
    _pclose(m_gp);
#endif
  }
}

void MapDrawer::SetAspectRatio(double a) {
  m_ratio = a;
  fprintf(m_gp, "set size ratio %lf\n", m_ratio);
}

void MapDrawer::SetRange(double R) {
  m_xmin = m_ymin = -R;
  m_xmax = m_ymax = R;
  fprintf(m_gp, "set xrange [%lf:%lf]\n", m_xmin, m_xmax);
}

void MapDrawer::SetRange(double xR, double yR) {
  m_xmin = -xR;
  m_xmax = xR;
  m_ymin = -yR;
  m_ymax = yR;
  fprintf(m_gp, "set xrange [%lf:%lf]\n", m_xmin, m_xmax);
  fprintf(m_gp, "set yrange [%lf:%lf]\n", m_ymin, m_ymax);
}

void MapDrawer::SetRange(double x_min, double x_max, double y_min,
                         double y_max) {
  m_xmin = x_min;
  m_xmax = x_max;
  m_ymin = y_min;
  m_ymax = y_max;
  fprintf(m_gp, "set xrange [%lf:%lf]\n", m_xmin, m_xmax);
  fprintf(m_gp, "set yrange [%lf:%lf]\n", m_ymin, m_ymax);
}

void MapDrawer::DrawScanGp(const Scan2D &scan) {
  std::vector<Pose2D> poses;
  Pose2D dummy;
  poses.emplace_back(dummy);
  DrawGp(scan.scaned_points(), poses);
}

void MapDrawer::DrawTrajectoryGp(const std::vector<Pose2D> &poses) {
  std::vector<ScanPoint2D> dummy;
  DrawGp(dummy, poses);
}

void MapDrawer::DrawGp(const std::vector<ScanPoint2D> &scaned_points,
                       const std::vector<Pose2D> &poses, bool flush) {
  std::cout << "DrawGp: scan points are " << scaned_points.size() << std::endl;

  fprintf(m_gp, "set multiplot\n");
  fprintf(m_gp, "plot '-' w p pt 7 ps 0.1 lc rgb 0x0, '-' with vector\n");

  // plot the point cloud
  for (size_t i = 0, size = scaned_points.size(); i < size; i += m_step_point) {
    double tx = scaned_points[i].x();
    double ty = scaned_points[i].y();
    fprintf(m_gp, "%lf %lf\n", tx, ty);
  }

  fprintf(m_gp, "e\n");

  // plot the robot pose
  double d = m_dd;

  for (size_t i = 0, size = poses.size(); i < size; i += m_step_pose) {
    double cx = poses[i].tx();
    double cy = poses[i].ty();
    double Cos = poses[i].R00();
    double Sin = poses[i].R10();

    double x1 = Cos * d;
    double y1 = Sin * d;
    double x2 = -Sin * d;
    double y2 = Cos * d;

    fprintf(m_gp, "%lf %lf %lf %lf\n", cx, cy, x1, y1);
    fprintf(m_gp, "%lf %lf %lf %lf\n", cx, cy, x2, y2);
  }

  fprintf(m_gp, "e\n");

  if (flush)
    fflush(m_gp);
}

void MapDrawer::DrawGp(const PointCloudMap *map) {
  DrawGp(map->global_map(), map->poses());
}
} // namespace slam
