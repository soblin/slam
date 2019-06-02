#include <gtest/gtest.h>
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

  ASSERT_EQ(curPointResult[0]->id(), 0);
  ASSERT_EQ(refPointResult[0]->id(), 0);

  ASSERT_EQ(curPointResult[1]->id(), 1);
  ASSERT_EQ(refPointResult[1]->id(), 0);

  ASSERT_EQ(curPointResult[2]->id(), 2);
  ASSERT_EQ(refPointResult[2]->id(), 1);

  ASSERT_EQ(curPointResult[3]->id(), 3);
  ASSERT_EQ(refPointResult[3]->id(), 2);

  ASSERT_EQ(curPointResult[4]->id(), 4);
  ASSERT_EQ(refPointResult[4]->id(), 3);

  ASSERT_EQ(curPointResult[5]->id(), 5);
  ASSERT_EQ(refPointResult[5]->id(), 4);

  ASSERT_EQ(curPointResult[6]->id(), 6);
  ASSERT_EQ(refPointResult[6]->id(), 5);

  ASSERT_EQ(curPointResult[7]->id(), 7);
  ASSERT_EQ(refPointResult[7]->id(), 8);

  ASSERT_EQ(curPointResult[8]->id(), 8);
  ASSERT_EQ(refPointResult[8]->id(), 9);

  ASSERT_EQ(curPointResult[9]->id(), 9);
  ASSERT_EQ(refPointResult[9]->id(), 10);

  ASSERT_EQ(curPointResult[10]->id(), 10);
  ASSERT_EQ(refPointResult[10]->id(), 11);

  ASSERT_EQ(curPointResult[11]->id(), 11);
  ASSERT_EQ(refPointResult[11]->id(), 12);

  ASSERT_EQ(curPointResult[12]->id(), 12);
  ASSERT_EQ(refPointResult[12]->id(), 13);

  ASSERT_EQ(curPointResult[13]->id(), 13);
  ASSERT_EQ(refPointResult[13]->id(), 14);

  ASSERT_EQ(curPointResult[14]->id(), 14);
  ASSERT_EQ(refPointResult[14]->id(), 15);

  ASSERT_EQ(curPointResult[15]->id(), 15);
  ASSERT_EQ(refPointResult[15]->id(), 15);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
