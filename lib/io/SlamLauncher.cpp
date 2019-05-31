#include <iostream>
#include <unistd.h>

#include <slam/io/SlamLauncher.h>

#ifdef DEBUG
static constexpr bool logger = true;
#else
static constexpr bool logger = false;
#endif

namespace slam {

SlamLauncher::SlamLauncher() : m_draw_skip(10), m_odometry_only(false) {
  /*
    m_point_cloud = std::make_unique<PointCloudMapBS>();
   */
}

SlamLauncher::~SlamLauncher() {
  /*
    m_point_cloud.reset();
   */
}

void SlamLauncher::SetOdometryOnly(bool only) { m_odometry_only = only; }

void SlamLauncher::ShowScans() {
  m_map_drawer.InitGnuplot();
  m_map_drawer.SetRange(-6, 6, -6, 6);
  m_map_drawer.SetAspectRatio(-0.9);

  size_t cnt = 0;
  Scan2D scan;
  bool eof = m_sensor_reader.LoadScan(cnt, scan);
  while (!eof) {
    usleep(100000);
    m_map_drawer.DrawScanGp(scan);

    if (logger)
      std::cout << "--- scan num=" << cnt << " ---" << '\n';

    eof = m_sensor_reader.LoadScan(cnt, scan);
    ++cnt;
  }
  this->m_sensor_reader.CloseScanFile();
  if (logger)
    std::cout << "SlamLauncher finished" << std::endl;
}

void SlamLauncher::MapByOdometry(const Scan2D &scan) {
  Pose2D pose;
  Pose2D::CalcRelativePose(scan.pose(), m_initial_pose, pose);
  auto scaned_points = scan.scaned_points();
  std::vector<ScanPoint2D> global_points;
  ScanPoint2D global_point;
  for (size_t j = 0, size = scaned_points.size(); j < size; ++j) {
    auto scan_point = scaned_points[j];
    pose.ToGlobalPoint(scan_point, global_point);
    global_points.emplace_back(global_point);
  }
}

bool SlamLauncher::SetFilename(const std::string filename) {
  return m_sensor_reader.OpenScanFile(filename);
}

void SlamLauncher::Run() {
  m_map_drawer.InitGnuplot();
  m_map_drawer.SetAspectRatio(-0.9);

  size_t cnt = 0;
  Scan2D scan;
  bool eof = m_sensor_reader.LoadScan(cnt, scan);
  while (!eof) {
    if (m_odometry_only) {
      if (cnt == 0) {
        m_initial_pose = scan.pose();
        m_initial_pose.CalcRmat();
      }
      MapByOdometry(scan);
    } else {
      std::cout << "Pleases set odometryOnly = true" << std::endl;
      exit(1);
    }
    if (cnt % m_draw_skip == 0) {
      //
    }
    ++cnt;
    eof = m_sensor_reader.LoadScan(cnt, scan);
    usleep(100000);
  }
}
} // namespace slam
