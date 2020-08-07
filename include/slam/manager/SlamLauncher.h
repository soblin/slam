#ifndef SLAM_LAUNCHER_H
#define SLAM_LAUNCHER_H

#include <slam/geometry/PointCloudMapBS.h>
#include <slam/io/MapDrawer.h>
#include <slam/io/SensorDataReader.h>
#include <slam/manager/FrameWorkCustomizer.h>
#include <slam/manager/SlamFrontEnd.h>

namespace slam {

class SlamLauncher {
public:
  SlamLauncher() {}
  ~SlamLauncher() {}

  void Initialize();
  void SetOdometryOnly(bool only);
  void ShowScans();
  void MapByOdometry(const Scan2D &scan);
  bool SetFilename(const std::string filename);
  void Run();
  void CustomizeFrameWork(const std::string &type);

private:
  Pose2D m_initial_pose;
  SensorDataReader m_sensor_reader;
  MapDrawer m_map_drawer;
  SlamFrontEnd m_slam_frontend;
  FrameWorkCustomizer m_customizer;

  // function paramters
  // all of these parameters are iniatialized in Initialize()n
private:
  PointCloudMap *m_cloud_map_ptr = nullptr;
  bool m_odometry_only = false;
  int m_draw_skip = 0;
  int m_usleep_time = 0;
};

} // namespace slam

#endif
