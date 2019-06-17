#include <gtest/gtest.h>
#include <slam/geometry/Scan2D.h>
#include <slam/icp/ScanPointResampler.h>

namespace slam {

class ScanPointResamplerTestFriend : public ::testing::Test {
public:
  void ResamplePoints(ScanPointResampler &resampler, Scan2D *scan) {
    resampler.ResamplePoints(scan);
  }
};

using namespace slam;

TEST_F(ScanPointResamplerTestFriend, testResamplePoints) {
  Scan2D scan;

  double y = 2.0;
  std::vector<ScanPoint2D> biased_points = {
      {0.0, y},  {0.09, y}, {0.14, y}, {0.18, y}, {0.225, y},
      {0.25, y}, {0.26, y}, {0.29, y}, {0.32, y}, {0.38, y},
      {0.43, y}, {0.65, y}, {0.72, y}};

  ASSERT_EQ(biased_points.size(), 13);

  scan.SetScanedPoints(biased_points);
  ScanPointResampler resampler;
  ResamplePoints(resampler, &scan);

  auto resampled_points = scan.scaned_points();
  ASSERT_EQ(resampled_points.size(), 11);

  auto point = resampled_points[0];
  ASSERT_FLOAT_EQ(point.x(), 0.0);
  point = resampled_points[1];
  ASSERT_FLOAT_EQ(point.x(), 0.05);

  point = resampled_points[2];
  ASSERT_FLOAT_EQ(point.x(), 0.10);

  point = resampled_points[3];
  ASSERT_FLOAT_EQ(point.x(), 0.15);

  point = resampled_points[4];
  ASSERT_FLOAT_EQ(point.x(), 0.20);

  point = resampled_points[5];
  ASSERT_FLOAT_EQ(point.x(), 0.25);

  point = resampled_points[6];
  ASSERT_FLOAT_EQ(point.x(), 0.30);

  point = resampled_points[7];
  ASSERT_FLOAT_EQ(point.x(), 0.35);

  point = resampled_points[8];
  ASSERT_FLOAT_EQ(point.x(), 0.40);

  point = resampled_points[9];
  ASSERT_FLOAT_EQ(point.x(), 0.65);

  point = resampled_points[10];
  ASSERT_FLOAT_EQ(point.x(), 0.70);
}
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
