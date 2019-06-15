#include <gtest/gtest.h>

#include <slam/icp/CostFunctionED.h>
#include <slam/icp/DataAssociatorLS.h>

namespace slam {

class DataAssociatorLSTestFriend : public ::testing::Test {
public:
  void GetCurPoints(DataAssociatorLS &dass,
                    std::vector<const ScanPoint2D *> &result) {
    result = dass.m_cur_points;
  }
  void GetRefPoints(DataAssociatorLS &dass,
                    std::vector<const ScanPoint2D *> &result) {
    result = dass.m_ref_points;
  }
  double FindCorrespondenceImpl(DataAssociatorLS &dass, const Scan2D *curScan,
                                const Pose2D &predictedPose, double thresh) {
    return dass.FindCorrespondence(curScan, predictedPose, thresh);
  }
};

class CostFunctionEDTestFriend : public ::testing::Test {
public:
  double CalcValue(CostFunctionED &func, double tx, double ty, double th,
                   double val_thresh) {
    return func.CalcValueImpl(tx, ty, th, val_thresh);
  }
  const std::vector<const ScanPoint2D *> &GetRefPoints(CostFunctionED &func) {
    return func.m_ref_points;
  }
  const std::vector<const ScanPoint2D *> &GetCurPoints(CostFunctionED &func) {
    return func.m_cur_points;
  }
};
}

using namespace slam;

static CostFunctionED cfunc;
static Scan2D curScan;

TEST_F(DataAssociatorLSTestFriend, testFindCorrespondence) {
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

  curScan.SetPose(predPose);
  curScan.SetScanedPoints(cur_points);

  DataAssociatorLS dass;

  dass.SetRefBase(ref_points);
  FindCorrespondenceImpl(dass, &curScan, predPose, 2.0);

  std::vector<const ScanPoint2D *> curPointResult;
  std::vector<const ScanPoint2D *> refPointResult;
  GetCurPoints(dass, curPointResult);
  GetRefPoints(dass, refPointResult);

  cfunc.SetPoints(curPointResult, refPointResult);
}

TEST_F(CostFunctionEDTestFriend, testCalcValue) {
  auto ref_points = GetRefPoints(cfunc);
  ASSERT_EQ(ref_points.size(), 16);
  auto cur_points = GetCurPoints(cfunc);
  ASSERT_EQ(cur_points.size(), 16);
  // use val_thresh = 2.0, expecting matching rate of 1.0
  double total_error = CalcValue(cfunc, 0, 0, 0, 2.0);
  double match_rate = cfunc.GetMatchRate();
  ASSERT_FLOAT_EQ(match_rate, 1.0);

  double error = 0;
  for (unsigned i = 0; i < ref_points.size(); ++i) {
    double dx = ref_points[i]->x() - cur_points[i]->x();
    double dy = ref_points[i]->y() - cur_points[i]->y();

    error += dx * dx + dy * dy;
  }

  ASSERT_FLOAT_EQ(error / 16.0 * 100, total_error);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
