#ifndef SENSOR_DATA_READER_H
#define SENSOR_DATA_READER_H

#include <fstream>
#include <slam/geometry/Scan2D.h>

namespace slam {

class SensorDataReader {
public:
  SensorDataReader() {}

  ~SensorDataReader(){};

  bool OpenScanFile(const std::string &filepath);
  void CloseScanFile();
  void SetAngleOffset(int offset);
  void Initialize();
  bool LoadScan(Scan2D &output);

private:
  bool LoadScanImpl(Scan2D &output);

private:
  int m_angle_offset = 0;
  std::ifstream m_in_file;
  double m_max_scan_range = 0;
  double m_min_scan_range = 0;
};
} // namespace slam

#endif
