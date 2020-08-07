#ifndef SENSOR_DATA_READER_H
#define SENSOR_DATA_READER_H

#include <fstream>
#include <slam/geometry/Scan2D.h>

namespace slam {

class SensorDataReader {
private:
  int m_angle_offset;
  std::ifstream m_in_file;

public:
  SensorDataReader() {}

  ~SensorDataReader(){};

  bool OpenScanFile(const std::string &filepath);
  void CloseScanFile();
  void SetAngleOffset(int offset);

  bool LoadScan(Scan2D &output);

private:
  bool LoadScanImpl(Scan2D &output);
};
} // namespace slam

#endif
