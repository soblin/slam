#include <iostream>
#include <unistd.h>

#include <slam/geometry/PointCloudMap.h>
#include <slam/manager/SlamLauncher.h>
#include <slam/parameters.h>

#ifdef DEBUG
static constexpr bool logger = true;
#else
static constexpr bool logger = false;
#endif

namespace slam {

void SlamLauncher::SetOdometryOnly(bool only) { m_odometry_only = only; }

void SlamLauncher::ShowScans() {
  m_map_drawer.InitGnuplot();
  m_map_drawer.SetRange(-6, 6, -6, 6);
  m_map_drawer.SetAspectRatio(-0.9);

  size_t cnt = 0;

  // store the read data to scan
  // but this does not accumulate the scans
  Scan2D scan_buf;
  bool eof = m_sensor_reader.LoadScan(cnt, scan_buf);
  while (!eof) {
    usleep(param::SlamLauncher_SLEEP_TIME);
    m_map_drawer.DrawScanGp(scan_buf);

    if (logger)
      std::cout << "--- scan num=" << cnt << " ---" << '\n';

    eof = m_sensor_reader.LoadScan(cnt, scan_buf);
    ++cnt;
  }
  this->m_sensor_reader.CloseScanFile();
  if (logger)
    std::cout << "SlamLauncher finished" << std::endl;
}

void SlamLauncher::MapByOdometry(const Scan2D &scan) {
  PointCloudMap *cloud_map_ptr = PointCloudMapSingleton::GetCloudMap();

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

  cloud_map_ptr->AddPose(pose);
  cloud_map_ptr->AddPoints(global_points);
  cloud_map_ptr->MakeGlobalMap();
}

bool SlamLauncher::SetFilename(const std::string filename) {
  return m_sensor_reader.OpenScanFile(filename);
}

void SlamLauncher::Run() {
  PointCloudMap *cloud_map_ptr = PointCloudMapSingleton::GetCloudMap();

  m_map_drawer.InitGnuplot();
  m_map_drawer.SetAspectRatio(-0.9);

  size_t cnt = 0;
  Scan2D scan_buf;
  bool eof = m_sensor_reader.LoadScan(cnt, scan_buf);
  while (!eof) {
    if (m_odometry_only) {
      if (cnt == 0) {
        m_initial_pose = scan_buf.pose();
        m_initial_pose.CalcRmat();
      }
      // use raw scan and add to PointCloudMap
      MapByOdometry(scan_buf);
    } else {
      // use ICP and add to PointCloudMap
      m_slam_frontend.Process(scan_buf);
    }
    if (cnt % param::SlamLauncher_PLOT_SKIP == 0) {
      m_map_drawer.DrawGp(cloud_map_ptr);
    }
    ++cnt;
    eof = m_sensor_reader.LoadScan(cnt, scan_buf);
    usleep(param::SlamLauncher_SLEEP_TIME);
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
  else
    m_customizer.CustomizeH();
}

} // namespace slam
