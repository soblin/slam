#include <slam/icp/CostFunctionED.h>
#include <slam/icp/DataAssociatorLS.h>

#include <gtest/gtest.h>

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
}

using namespace slam;

TEST_F(DataAssociatorLSTestFriend, testFindCorrespondence) {
  std::vector<ScanPoint2D> ref_points;
  std::vector<ScanPoint2D> cur_points;
  Pose2D predPose(0, 0, 0);

  // generate data
  std::vector<double> Xs = {2, 3, 4, 5, 6, 7, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9};
  std::vector<double> Ys = {10, 10, 10, 10, 10, 10, 10, 10,
                            9,  8,  7,  6,  5,  4,  3,  2};
  for (unsigned i = 0; i < Xs.size(); ++i) {
    ref_points.emplace_back(i, Xs[i], Ys[i]);
  }
  for (unsigned i = 0; i < Xs.size(); ++i) {
    cur_points.emplace_back(i, Xs[i] - 0.8, Ys[i] - 1.0);
  }
  Scan2D curScan;
  curScan.SetPose(predPose);
  curScan.SetScanedPoints(cur_points);

  DataAssociatorLS dass;

  dass.SetRefBase(ref_points);
  FindCorrespondenceImpl(dass, &curScan, predPose, 2.0);

  std::vector<const ScanPoint2D *> curPointResult;
  std::vector<const ScanPoint2D *> refPointResult;
  GetCurPoints(dass, curPointResult);
  GetRefPoints(dass, refPointResult);

  CostFunctionED cfunc;
  cfunc.SetPoints(curPointResult, refPointResult);
  double ratio = cfunc.CalcValue(0, 0, 0);

  ASSERT_EQ(ratio >= 0, true);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
