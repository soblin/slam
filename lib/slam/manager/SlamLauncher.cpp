#include <iostream>
#include <unistd.h>

#include <slam/geometry/PointCloudMap.h>
#include <slam/manager/CounterServer.h>
#include <slam/manager/ParamServer.h>
#include <slam/manager/SlamLauncher.h>

#ifdef DEBUG
static constexpr bool logger = true;
#else
static constexpr bool logger = false;
#endif

namespace slam {

void SlamLauncher::Initialize() {
  // Initialize member instance
  // Initialization of frontend, the parameters are registered
  m_slam_frontend.Initialize();
  m_map_drawer.Initialize();
  m_map_drawer.SetAspectRatio(-0.9);
  // the parameter values have been registered
  m_sensor_reader.Initialize();
  m_odometry_only = false;
  m_draw_skip = static_cast<int>(ParamServer::Get("SlamLauncher_PLOT_SKIP"));
  m_usleep_time = static_cast<int>(ParamServer::Get("SlamLauncher_SLEEP_TIME"));
}

void SlamLauncher::SetOdometryOnly(bool only) { m_odometry_only = only; }

void SlamLauncher::MapByOdometry(const Scan2D &scan) {
  Pose2D pose;
  Pose2D::CalcRelativePose(scan.pose(), m_initial_pose, pose);
  const auto scaned_points = scan.scaned_points();
  std::vector<ScanPoint2D> global_points;
  ScanPoint2D global_point;
  // convert scaned_points(in robot frame) to global frame
  for (size_t j = 0, size = scaned_points.size(); j < size; ++j) {
    auto scan_point = scaned_points[j];
    pose.ToGlobalPoint(scan_point, global_point);
    global_points.emplace_back(global_point);
  }

  m_cloud_map_ptr->AddPose(pose);
  m_cloud_map_ptr->AddPoints(global_points);
  m_cloud_map_ptr->MakeGlobalMap();
}

bool SlamLauncher::SetFilename(const std::string filename) {
  return m_sensor_reader.OpenScanFile(filename);
}

void SlamLauncher::Run() {
  Scan2D scan_buf;
  bool eof = m_sensor_reader.LoadScan(scan_buf);

  while (!eof) {
    int cnt = CounterServer::Get();

    if (!m_odometry_only) {
      // use ICP and add to PointCloudMap
      m_slam_frontend.Process(scan_buf);
    } else {
      if (cnt == 0) {
        m_initial_pose = scan_buf.pose();
        m_initial_pose.CalcRmat();
      }
      // use raw scan and add to PointCloudMap
      MapByOdometry(scan_buf);
    }

    if (cnt % m_draw_skip == 0)
      m_map_drawer.DrawGp(m_cloud_map_ptr);

    eof = m_sensor_reader.LoadScan(scan_buf);
    usleep(m_usleep_time);
  }
}

void SlamLauncher::CustomizeFrameWork(const std::string &type) {
  m_customizer.SetSlamFrontEnd(&m_slam_frontend);

  if (type == "customA")
    m_customizer.CustomizeA();
  else if (type == "customB")
    m_customizer.CustomizeB();
  else if (type == "customC")
    m_customizer.CustomizeC();
  else if (type == "customD")
    m_customizer.CustomizeD();
  else if (type == "customE")
    m_customizer.CustomizeE();
  else if (type == "customF")
    m_customizer.CustomizeF();
  else if (type == "customG")
    m_customizer.CustomizeG();
  else if (type == "customH")
    m_customizer.CustomizeH();
  else if (type == "customI")
    m_customizer.CustomizeI();
  else
    m_customizer.CustomizeI();

  m_cloud_map_ptr = PointCloudMapSingleton::GetCloudMap();
}

} // namespace slam
