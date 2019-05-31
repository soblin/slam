#include <fstream>
#include <gtest/gtest.h>

#include "slam/io/SensorDataReader.h"

TEST(SensorDataReader, read_test) {
  std::ofstream ofs("test.lsc");
  // (x, y, theta) = (10, 10, π/3)
  ofs << "LASERSCAN 12345 12345 12345 7 -180 1 -150 2 -120 3 -90 4 -60 5 -30 6 "
         "0 7 10 10 0.52359877"
      << '\n';
  ofs << "LASERSCAN 12345 12345 12345 5 -165 1 -150 0.1 -120 3 -90 6.5 -60 5 "
         "12 12 0.785398"
      << EOF;
  ofs.close();

  slam::SensorDataReader reader;
  reader.OpenScanFile("test.lsc");
  slam::Scan2D scan1;
  reader.LoadScan(0, scan1);
  slam::Scan2D scan2;
  reader.LoadScan(1, scan2);
  scan1.scaned_points()[0];
  ASSERT_EQ(6, scan1.scaned_points().size());
  ASSERT_EQ(5, scan2.scaned_points().size());
}

int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
