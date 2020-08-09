#include <slam/icp/CostFunctionED.h>
#include <slam/icp/DataAssociatorLS.h>
#include <slam/icp/PoseOptimizerSD.h>
#include <slam/manager/ParamServer.h>

#include <gtest/gtest.h>

using namespace slam;

TEST(PoseOptimizerSDTest, testOptimizePost) {
  ParamServer::Create();
  ParamServer::Set("PoseOptimizer_VAL_DIFF_THRESH", 0.1);
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

  DataAssociatorLS dass;
  dass.Initialize();
  dass.SetRefBase(ref_points);
  dass.FindCorrespondence(&curScan, predPose, 2.0);

  std::vector<const ScanPoint2D *> curPointResult;
  std::vector<const ScanPoint2D *> refPointResult;
  curPointResult = dass.cur_points();
  refPointResult = dass.ref_points();

  CostFunctionED cfunc;
  cfunc.Initialize();
  PoseOptimizerSD optimizer;
  optimizer.SetCostFunction(&cfunc);
  optimizer.Initialize();
  optimizer.SetPoints(curPointResult, refPointResult);
  // optimizer.SetValThresh(2.0);
  // optimizer.SetValDiffThresh(0.1);
  Pose2D initPose(0, 0, 0);
  Pose2D estimatePose(0, 0, 0);

  double result = optimizer.OptimizePose(initPose, estimatePose);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
