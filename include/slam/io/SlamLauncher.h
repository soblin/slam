#ifndef SLAM_LAUNCHER_H
#define SLAM_LAUNCHER_H

#include <slam/geometry/PointCloudMap.h>
#include <slam/io/MapDrawer.h>
#include <slam/io/SensorDataReader.h>

namespace slam {

class SlamLauncher {
private:
  int m_draw_skip;
  bool m_odometry_only;
  Pose2D m_initial_pose;
  SensorDataReader m_sensor_reader;
  MapDrawer m_map_drawer;
  PointCloudMap m_map;

public:
  SlamLauncher();
  ~SlamLauncher();
  void SetOdometryOnly(bool only);
  void ShowScans();
  void MapByOdometry(const Scan2D &scan);
  bool SetFilename(const std::string filename);
  void Run();
};

} // namespace slam

#endif
