#include <fstream>
#include <cmath>
#include <gtest/gtest.h>

#include "slam/io/SensorDataReader.h"

TEST(SensorDataReader, read_test) {
  std::ofstream ofs("test.lsc");
  // (x, y, theta) = (10, 10, π/3)
  /*
    double Scan2D::MAX_SCAN_RANGE = 6;
    double Scan2D::MIN_SCAN_RANGE = 0.1;
   */
  ofs << "LASERSCAN 12345 12345 12345 7 -180 1 -150 2 -120 3 -90 4 -60 5 -30 6 "
         "0 7 10 10 0.52359877"
      << '\n';
  // only (-180 1) (-150 2) (-120 3) (-90 4) (-60 5) is used

  // (x, y, theta) = (12, 12, π/2)
  ofs << "LASERSCAN 12345 12345 12345 5 -165 1 -150 0.1 -120 3 -90 6.5 -60 5 "
         "12 12 0.785398"
      << EOF;
  // only (-165 1) (-120 3) (-60 5) is used
  ofs.close();

  slam::SensorDataReader reader;
  reader.OpenScanFile("test.lsc");
  slam::Scan2D scan1;
  reader.LoadScan(0, scan1);
  slam::Scan2D scan2;
  reader.LoadScan(1, scan2);
  scan1.scaned_points()[0];
  ASSERT_EQ(5, scan1.scaned_points().size());
  ASSERT_EQ(3, scan2.scaned_points().size());
}

TEST(SensorDataReader, scaned_local_position_test){
  slam::SensorDataReader reader;
  reader.OpenScanFile("test.lsc");
  slam::Scan2D scan;
  reader.LoadScan(0, scan);
  // attention!! angle_offset = 180[deg]
  ASSERT_FLOAT_EQ(1.0, scan.scaned_points()[0].x());
  ASSERT_FLOAT_EQ(0, scan.scaned_points()[0].y());
  ASSERT_FLOAT_EQ(std::sqrt(3.0), scan.scaned_points()[1].x());
  ASSERT_FLOAT_EQ(1.0, scan.scaned_points()[1].y());

  decltype(scan) scan2;
  reader.LoadScan(1, scan2);
  ASSERT_FLOAT_EQ(1.5, scan2.scaned_points()[1].x());
  ASSERT_FLOAT_EQ(std::sqrt(3.) * 1.5, scan2.scaned_points()[1].y());
}

int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
