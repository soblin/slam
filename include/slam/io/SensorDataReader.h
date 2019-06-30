#ifndef SENSOR_DATA_READER_H
#define SENSOR_DATA_READER_H

#include <fstream>
#include <slam/geometry/Scan2D.h>
#include <slam/parameters.h>

namespace slam {

class SensorDataReader {
private:
  int m_angle_offset;
  std::ifstream m_in_file;

public:
  SensorDataReader() : m_angle_offset(param::SensorDataReader_ANGLE_OFFSET) {}

  ~SensorDataReader(){};

  bool OpenScanFile(const std::string &filepath);
  void CloseScanFile();
  void SetAngleOffset(int offset);

  bool LoadScan(size_t cnt_id, Scan2D &output);

private:
  bool LoadScanImpl(size_t cnt_id, Scan2D &output);
};
} // namespace

#endif
