#include <gtest/gtest.h>

#include <slam/icp/CostFunctionED.h>
#include <slam/icp/DataAssociatorLS.h>

using namespace slam;

TEST(CostFunctionED, testCalcValue) {
  CostFunctionED cfunc;
  DataAssociatorLS dass;
  Scan2D curScan;

  std::vector<ScanPoint2D> ref_points;
  std::vector<ScanPoint2D> cur_points;
  Pose2D predPose(0, 0, 0);
  std::vector<double> Xs = {2, 3, 4, 5, 6, 7, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9};
  std::vector<double> Ys = {10, 10, 10, 10, 10, 10, 10, 10,
                            9,  8,  7,  6,  5,  4,  3,  2};
  for (unsigned i = 0; i < Xs.size(); ++i) {
    ref_points.emplace_back(Xs[i], Ys[i]);
  }
  for (unsigned i = 0; i < Xs.size(); ++i) {
    cur_points.emplace_back(Xs[i] - 0.8, Ys[i] - 1.0);
  }
  curScan.SetPose(predPose);
  curScan.SetScanedPoints(cur_points);

  dass.SetRefBase(ref_points);
  double correspondence = dass.FindCorrespondence(&curScan, predPose, 2.0);

  auto ref_points_matched = dass.ref_points();
  auto cur_points_matched = dass.cur_points();

  cfunc.SetPoints(cur_points_matched, ref_points_matched);
  // use val_thresh = 2.0, expecting matching rate of 1.0
  double total_error = cfunc.CalcValue(0, 0, 0, 2.0);
  double match_rate = cfunc.GetMatchRate();
  ASSERT_FLOAT_EQ(match_rate, 1.0);

  double error = 0;
  for (unsigned i = 0; i < ref_points_matched.size(); ++i) {
    double dx = ref_points_matched[i]->x() - cur_points_matched[i]->x();
    double dy = ref_points_matched[i]->y() - cur_points_matched[i]->y();

    error += dx * dx + dy * dy;
  }

  ASSERT_FLOAT_EQ(error / 16.0 * 100, total_error);
}

TEST(CostFunctionED, testCalcValueConcreteValue) {
  CostFunctionED cfunc;
  DataAssociatorLS dass;
  Scan2D curScan;

  std::vector<ScanPoint2D> ref_points;
  std::vector<ScanPoint2D> cur_points;
  Pose2D predPose(0, 0, 0);
  std::vector<double> Xs = {2, 3, 4, 5, 6, 7, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9};
  std::vector<double> Ys = {10, 10, 10, 10, 10, 10, 10, 10,
                            9,  8,  7,  6,  5,  4,  3,  2};
  for (unsigned i = 0; i < Xs.size(); ++i) {
    ref_points.emplace_back(Xs[i], Ys[i]);
  }
  for (unsigned i = 0; i < Xs.size(); ++i) {
    cur_points.emplace_back(Xs[i] - 0.8, Ys[i] - 1.0);
  }
  curScan.SetPose(predPose);
  curScan.SetScanedPoints(cur_points);

  dass.SetRefBase(ref_points);
  double correspondence = dass.FindCorrespondence(&curScan, predPose, 2.0);

  auto ref_points_matched = dass.ref_points();
  auto cur_points_matched = dass.cur_points();

  cfunc.SetPoints(cur_points_matched, ref_points_matched);
  // use val_thresh = 2.0, expecting matching rate of 1.0
  double total_error = cfunc.CalcValue(0, 0, 0, 2.0);
  double match_rate = cfunc.GetMatchRate();
  ASSERT_FLOAT_EQ(match_rate, 1.0);

  double error = 0;
  error += (0.8 * 0.8) + 1.0;
  error += ((0.2 * 0.2) + 1.0) * 6;
  error += (0.8 * 0.8) * 8;
  error += (0.8 * 0.8) + 1.0;
  ASSERT_FLOAT_EQ(error / 16.0 * 100, total_error);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
