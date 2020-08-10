#include <iostream>
#include <math.h>
#include <slam/io/SensorDataReader.h>
#include <slam/manager/ParamServer.h>

namespace slam {

void SensorDataReader::Initialize() {
  m_max_scan_range = ParamServer::Get("Scan2D_MAX_SCAN_RANGE");
  m_min_scan_range = ParamServer::Get("Scan2D_MIN_SCAN_RANGE");
  m_angle_offset = ParamServer::Get("SensorDataReader_ANGLE_OFFSET");
}

bool SensorDataReader::OpenScanFile(const std::string &filepath) {
  m_in_file.open(filepath.c_str());
  if (!m_in_file.is_open()) {
    std::cerr << "Error : cannot open " << filepath << std::endl;
    return false;
  }
  return true;
}

void SensorDataReader::CloseScanFile() { m_in_file.close(); }

void SensorDataReader::SetAngleOffset(int offset) { m_angle_offset = offset; }

bool SensorDataReader::LoadScan(Scan2D &output) {
  bool is_scan = false;
  while (!m_in_file.eof() and !is_scan) {
    is_scan = LoadScanImpl(output);
  }
  if (is_scan)
    return false; // the file continues
  else
    return true; // EOF of scan file
}

bool SensorDataReader::LoadScanImpl(Scan2D &output) {
  std::string type;
  // data format
  // Attention! the angle is [deg]
  // LASERSCAN id t1 t2 n theta_1[deg] d_1 ... theta_n[deg] d_n odom_x odom_y
  // odom_th[rad]
  m_in_file >> type;
  if (type == "LASERSCAN") {
    // misc
    int id, t1, t2;
    m_in_file >> id >> t1 >> t2;

    // Scan2D
    std::vector<ScanPoint2D> scan_points;
    int scan_num;
    m_in_file >> scan_num;
    scan_points.reserve(scan_num);
    double deg, dist;
    ScanPoint2D scan_point;
    for (int i = 0; i < scan_num; ++i) {
      m_in_file >> deg >> dist;
      deg += m_angle_offset;
      if (dist >= m_max_scan_range || dist <= m_min_scan_range)
        continue;
      scan_point.CalcXY(dist, M_PI * deg / 180);
      scan_points.emplace_back(scan_point);
    }
    output.SetScanedPoints(scan_points);

    // Pose2D
    double tx, ty;
    m_in_file >> tx >> ty;
    output.tx() = tx;
    output.ty() = ty;
    double th; // [rad]
    m_in_file >> th;
    output.SetAngle(th);

    return true;
  } else {
    std::string line;
    getline(m_in_file, line);
    return false;
  }
}

} // namespace slam
