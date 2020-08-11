#include <gtest/gtest.h>
#include <slam/geometry/NNGridTable.h>
#include <slam/manager/ParamServer.h>

namespace slam {

class NNGridTableTestFriend : public ::testing::Test {
public:
  int GetTableSize(NNGridTable &table) { return table.m_table.size(); }
};
} // namespace slam

using namespace slam;

TEST_F(NNGridTableTestFriend, testTableSize) {
  ParamServer::Create();
  // [-10, 10] x [-10, 10]
  ParamServer::Set("NNGridTable_DOMAIN_SIZE", 10.0);
  // ParamServer::Set("PointCloudMapGT_CELL_POINT_NUM_THRESH", 5.0);
  ParamServer::Set("NNGridTable_CELL_SIZE", 2.0);
  ParamServer::Set("NNGridTable_MIN_DIST_THRESH", 5.0);
  NNGridTable grid_table;
  grid_table.Initialize();

  std::vector<ScanPoint2D> points;
  for (int i = -6; i <= 6; i++) {
    for (int j = -6; j <= 6; j++) {
      points.emplace_back(2 * i, 2 * j, 0, 0, ScanPoint2D::PointType::LINE);
    }
  }
  for (int i = 0; i < points.size(); ++i) {
    grid_table.AddPoint(&points[i]);
  }

  ASSERT_EQ(GetTableSize(grid_table), 121);

  Pose2D basePose(0, 0, 0);
  ScanPoint2D query1(-8.1, -7.1);

  const ScanPoint2D *nearest = grid_table.FindClosestPoint(&query1, basePose);

  ASSERT_EQ(nearest->x(), -8.0);
  ASSERT_EQ(nearest->y(), -8.0);

  ScanPoint2D query2(-6.2, -2.2);

  nearest = grid_table.FindClosestPoint(&query2, basePose);

  ASSERT_EQ(nearest->x(), -6.0);
  ASSERT_EQ(nearest->y(), -2.0);

  ScanPoint2D temp(-6.1, -2.1, 0, 0, ScanPoint2D::PointType::LINE);
  grid_table.AddPoint(&temp);
  nearest = grid_table.FindClosestPoint(&query2, basePose);

  ASSERT_EQ(nearest->x(), -6.1);
  ASSERT_EQ(nearest->y(), -2.1);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
