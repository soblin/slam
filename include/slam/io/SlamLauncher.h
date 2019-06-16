#ifndef SLAM_LAUNCHER_H
#define SLAM_LAUNCHER_H

#include <slam/geometry/PointCloudMapBS.h>
#include <slam/io/FrameWorkCustomizer.h>
#include <slam/io/MapDrawer.h>
#include <slam/io/SensorDataReader.h>
#include <slam/io/SlamFrontEnd.h>

namespace slam {

class SlamLauncher {
private:
  bool m_odometry_only;
  Pose2D m_initial_pose;
  SensorDataReader m_sensor_reader;
  MapDrawer m_map_drawer;
  SlamFrontEnd m_slam_frontend;
  FrameWorkCustomizer m_customizer;

public:
  SlamLauncher();
  ~SlamLauncher();
  void SetOdometryOnly(bool only);
  void ShowScans();
  void MapByOdometry(const Scan2D &scan);
  bool SetFilename(const std::string filename);
  void Run();
  void CustomizeFrameWork();
};

} // namespace slam

#endif
