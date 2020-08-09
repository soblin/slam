#include <gtest/gtest.h>
#include <slam/icp/DataAssociatorGT.h>
#include <slam/manager/SlamFrontEnd.h>

using namespace slam;

TEST(DataAssociatorGT, testFindCorrespondence) {
  SlamFrontEnd frontend;
  frontend.Init();

  std::vector<ScanPoint2D> ref_points;
  std::vector<ScanPoint2D> cur_points;
  Pose2D predPose(0, 0, 0);

  // generate data
  std::vector<double> Xs = {2, 3, 4, 5, 6, 7, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9};
  std::vector<double> Ys = {10, 10, 10, 10, 10, 10, 10, 10,
                            9,  8,  7,  6,  5,  4,  3,  2};
  for (unsigned i = 0; i < Xs.size(); ++i) {
    ref_points.emplace_back(Xs[i], Ys[i]);
  }
  for (unsigned i = 0; i < Xs.size(); ++i) {
    cur_points.emplace_back(Xs[i] - 0.8, Ys[i] - 1.0);
  }
  Scan2D curScan;
  curScan.SetPose(predPose);
  curScan.SetScanedPoints(cur_points);

  DataAssociatorGT dass;

  dass.SetRefBase(ref_points);
  double correspondence = FindCorrespondence(dass, &curScan, predPose, 2.0);

  std::vector<const ScanPoint2D *> curPointResult;
  std::vector<const ScanPoint2D *> refPointResult;
  curPointResult = dass.cur_points();
  refPointResult = dass.ref_points();

  // 0
  ASSERT_NEAR(curPointResult[0]->x(), Xs[0] - 0.8, 0.1);
  ASSERT_NEAR(curPointResult[0]->y(), Ys[0] - 1.0, 0.1);
  ASSERT_NEAR(refPointResult[0]->x(), Xs[0], 0.1);
  ASSERT_NEAR(refPointResult[0]->y(), Ys[0], 0.1);

  // 1
  ASSERT_NEAR(curPointResult[1]->x(), Xs[1] - 0.8, 0.1);
  ASSERT_NEAR(curPointResult[1]->y(), Ys[1] - 1.0, 0.1);
  ASSERT_NEAR(refPointResult[1]->x(), Xs[0], 0.1);
  ASSERT_NEAR(refPointResult[1]->y(), Ys[0], 0.1);

  // 2
  ASSERT_NEAR(curPointResult[2]->x(), Xs[2] - 0.8, 0.1);
  ASSERT_NEAR(curPointResult[2]->y(), Ys[2] - 1.0, 0.1);
  ASSERT_NEAR(refPointResult[2]->x(), Xs[1], 0.1);
  ASSERT_NEAR(refPointResult[2]->y(), Ys[1], 0.1);

  // 3
  ASSERT_NEAR(curPointResult[3]->x(), Xs[3] - 0.8, 0.1);
  ASSERT_NEAR(curPointResult[3]->y(), Ys[3] - 1.0, 0.1);
  ASSERT_NEAR(refPointResult[3]->x(), Xs[2], 0.1);
  ASSERT_NEAR(refPointResult[3]->y(), Ys[2], 0.1);

  // 4
  ASSERT_NEAR(curPointResult[4]->x(), Xs[4] - 0.8, 0.1);
  ASSERT_NEAR(curPointResult[4]->y(), Ys[4] - 1.0, 0.1);
  ASSERT_NEAR(refPointResult[4]->x(), Xs[3], 0.1);
  ASSERT_NEAR(refPointResult[4]->y(), Ys[3], 0.1);

  // 5
  ASSERT_NEAR(curPointResult[5]->x(), Xs[5] - 0.8, 0.1);
  ASSERT_NEAR(curPointResult[5]->y(), Ys[5] - 1.0, 0.1);
  ASSERT_NEAR(refPointResult[5]->x(), Xs[4], 0.1);
  ASSERT_NEAR(refPointResult[5]->y(), Ys[4], 0.1);

  // 6
  ASSERT_NEAR(curPointResult[6]->x(), Xs[6] - 0.8, 0.1);
  ASSERT_NEAR(curPointResult[6]->y(), Ys[6] - 1.0, 0.1);
  ASSERT_NEAR(refPointResult[6]->x(), Xs[5], 0.1);
  ASSERT_NEAR(refPointResult[6]->y(), Ys[5], 0.1);

  ASSERT_NEAR(curPointResult[7]->x(), Xs[8] - 0.8, 0.1);
  ASSERT_NEAR(curPointResult[7]->y(), Ys[8], 0.1);
  ASSERT_NEAR(refPointResult[7]->x(), Xs[8], 0.1);
  ASSERT_NEAR(refPointResult[7]->y(), Ys[8], 0.1);

  ASSERT_NEAR(curPointResult[8]->x(), Xs[9] - 0.8, 0.1);
  ASSERT_NEAR(curPointResult[8]->y(), Ys[9], 0.1);
  ASSERT_NEAR(refPointResult[8]->x(), Xs[9], 0.1);
  ASSERT_NEAR(refPointResult[8]->y(), Ys[9], 0.1);

  ASSERT_NEAR(curPointResult[9]->x(), Xs[10] - 0.8, 0.1);
  ASSERT_NEAR(curPointResult[9]->y(), Ys[10], 0.1);
  ASSERT_NEAR(refPointResult[9]->x(), Xs[10], 0.1);
  ASSERT_NEAR(refPointResult[9]->y(), Ys[10], 0.1);

  ASSERT_NEAR(curPointResult[10]->x(), Xs[11] - 0.8, 0.1);
  ASSERT_NEAR(curPointResult[10]->y(), Ys[11], 0.1);
  ASSERT_NEAR(refPointResult[10]->x(), Xs[11], 0.1);
  ASSERT_NEAR(refPointResult[10]->y(), Ys[11], 0.1);

  ASSERT_NEAR(curPointResult[11]->x(), Xs[12] - 0.8, 0.1);
  ASSERT_NEAR(curPointResult[11]->y(), Ys[12], 0.1);
  ASSERT_NEAR(refPointResult[11]->x(), Xs[12], 0.1);
  ASSERT_NEAR(refPointResult[11]->y(), Ys[12], 0.1);

  ASSERT_NEAR(curPointResult[12]->x(), Xs[13] - 0.8, 0.1);
  ASSERT_NEAR(curPointResult[12]->y(), Ys[13], 0.1);
  ASSERT_NEAR(refPointResult[12]->x(), Xs[13], 0.1);
  ASSERT_NEAR(refPointResult[12]->y(), Ys[13], 0.1);

  ASSERT_NEAR(curPointResult[13]->x(), Xs[14] - 0.8, 0.1);
  ASSERT_NEAR(curPointResult[13]->y(), Ys[14], 0.1);
  ASSERT_NEAR(refPointResult[13]->x(), Xs[14], 0.1);
  ASSERT_NEAR(refPointResult[13]->y(), Ys[14], 0.1);

  ASSERT_NEAR(curPointResult[14]->x(), Xs[15] - 0.8, 0.1);
  ASSERT_NEAR(curPointResult[14]->y(), Ys[15], 0.1);
  ASSERT_NEAR(refPointResult[14]->x(), Xs[15], 0.1);
  ASSERT_NEAR(refPointResult[14]->y(), Ys[15], 0.1);

  ASSERT_NEAR(curPointResult[15]->x(), Xs[15] - 0.8, 0.1);
  ASSERT_NEAR(curPointResult[15]->y(), Ys[15] - 1.0, 0.1);
  ASSERT_NEAR(refPointResult[15]->x(), Xs[15], 0.1);
  ASSERT_NEAR(refPointResult[15]->y(), Ys[15], 0.1);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
