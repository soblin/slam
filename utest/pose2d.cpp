#include <cmath>
#include <fstream>

#include <gtest/gtest.h>

#include "slam/geometry/Pose2D.h"

// for graphical understanding of this test, see docs/images/Pose2DTestCase.png

TEST(Pose2D, Initialize) {
  slam::Pose2D origin(0, 0, 0);
  slam::Pose2D pose0(3, 3, 60);
  slam::Pose2D pose1(4, 4, 120);

  ASSERT_FLOAT_EQ(0.0, origin.tx());
  ASSERT_FLOAT_EQ(0.0, origin.ty());
  ASSERT_FLOAT_EQ(0.0, origin.th());

  ASSERT_FLOAT_EQ(3.0, pose0.tx());
  ASSERT_FLOAT_EQ(3.0, pose0.ty());
  ASSERT_FLOAT_EQ(M_PI / 3, pose0.th());

  ASSERT_FLOAT_EQ(4.0, pose1.tx());
  ASSERT_FLOAT_EQ(4.0, pose1.ty());
  ASSERT_FLOAT_EQ(2 * M_PI / 3, pose1.th());
}

TEST(Pose2D, Reset) {
  slam::Pose2D pose0(3, 3, 60);

  pose0.Reset();
  ASSERT_FLOAT_EQ(0., pose0.tx());
  ASSERT_FLOAT_EQ(0., pose0.ty());
  ASSERT_FLOAT_EQ(0., pose0.th());
}

TEST(Pose2D, SetVal) {
  slam::Pose2D point(0, 0, 0);
  point.SetVal(3, 3, 7 * M_PI / 3 /* == M_PI/3*/);

  ASSERT_FLOAT_EQ(3.0, point.tx());
  ASSERT_FLOAT_EQ(3.0, point.ty());
  ASSERT_FLOAT_EQ(M_PI / 3, point.th());

  ASSERT_FLOAT_EQ(0.5, point.R00());
  ASSERT_FLOAT_EQ(-std::sqrt(3.0) / 2, point.R01());
  ASSERT_FLOAT_EQ(std::sqrt(3.0) / 2, point.R10());
  ASSERT_FLOAT_EQ(0.5, point.R11());
}

TEST(Pose2D, SetTranslationAndSetAngle) {
  slam::Pose2D point(0, 0, 0);

  point.SetTranslation(4, 4);
  point.SetAngle(8 * M_PI / 3);

  ASSERT_FLOAT_EQ(4.0, point.tx());
  ASSERT_FLOAT_EQ(4.0, point.ty());
  ASSERT_FLOAT_EQ(2 * M_PI / 3, point.th());

  ASSERT_FLOAT_EQ(-0.5, point.R00());
  ASSERT_FLOAT_EQ(-std::sqrt(3) / 2, point.R01());
  ASSERT_FLOAT_EQ(std::sqrt(3) / 2, point.R10());
  ASSERT_FLOAT_EQ(-0.5, point.R11());
}

TEST(Pose2D, CalcGloalAndLocalPose) {
  const slam::Pose2D origin(0, 0, 0);
  const slam::Pose2D pose0(3, 3, 60);
  const slam::Pose2D pose1(4, 4, 120);

  slam::Pose2D rel;
  slam::Pose2D abso;

  // relative pose of pose0 with respect to origin
  slam::Pose2D::CalcRelativePose(pose0 /*argument of interest*/,
                                 origin /*base pose*/,
                                 rel /*result to be stored*/);
  ASSERT_DOUBLE_EQ(rel.tx(), 3.0);
  ASSERT_DOUBLE_EQ(rel.ty(), 3.0);
  ASSERT_DOUBLE_EQ(rel.th(), M_PI / 3);
  // check if return to the original with inverse manipulation
  slam::Pose2D::CalcGlobalPose(rel /*argument of interest*/,
                               origin /*base pose*/, abso /*result*/);
  ASSERT_DOUBLE_EQ(abso.tx(), pose0.tx());
  ASSERT_DOUBLE_EQ(abso.ty(), pose0.ty());
  ASSERT_DOUBLE_EQ(abso.th(), pose0.th());

  // relative pose of pose1 with respect to origin
  slam::Pose2D::CalcRelativePose(pose1, origin, rel);
  ASSERT_DOUBLE_EQ(rel.tx(), 4.0);
  ASSERT_DOUBLE_EQ(rel.ty(), 4.0);
  ASSERT_DOUBLE_EQ(rel.th(), 2 * M_PI / 3);
  // check if return to the original with inverse manupulation
  slam::Pose2D::CalcGlobalPose(rel, origin, abso);
  ASSERT_DOUBLE_EQ(abso.tx(), pose1.tx());
  ASSERT_DOUBLE_EQ(abso.ty(), pose1.ty());
  ASSERT_DOUBLE_EQ(abso.th(), pose1.th());

  // relative pose of pose1 with respect to pose0
  slam::Pose2D::CalcRelativePose(pose1, pose0, rel);
  ASSERT_DOUBLE_EQ(rel.tx(), std::sqrt(2) * std::cos(15.0 / 180 * M_PI));
  ASSERT_DOUBLE_EQ(rel.ty(), -std::sqrt(2) * std::sin(15.0 / 180 * M_PI));
  ASSERT_DOUBLE_EQ(rel.th(), M_PI / 3);
  slam::Pose2D::CalcGlobalPose(rel, pose0, abso);
  ASSERT_DOUBLE_EQ(abso.tx(), pose1.tx());
  ASSERT_DOUBLE_EQ(abso.ty(), pose1.ty());
  ASSERT_DOUBLE_EQ(abso.th(), pose1.th());

  // relative pose of pose0 with respect to pose1
  slam::Pose2D::CalcRelativePose(pose0, pose1, rel);
  ASSERT_DOUBLE_EQ(rel.tx(), std::sqrt(2) * std::cos(105.0 / 180 * M_PI));
  ASSERT_DOUBLE_EQ(rel.ty(), std::sqrt(2) * std::sin(105.0 / 180 * M_PI));
  ASSERT_DOUBLE_EQ(rel.th(), -M_PI / 3);
  slam::Pose2D::CalcGlobalPose(rel, pose1, abso);
  ASSERT_DOUBLE_EQ(abso.tx(), pose0.tx());
  ASSERT_DOUBLE_EQ(abso.ty(), pose0.ty());
  ASSERT_DOUBLE_EQ(abso.th(), pose0.th());
}

TEST(Pose2D, ToRelativeAndGlobalPoint) {
  const slam::Pose2D origin(0, 0, 0);
  const decltype(origin) pose0(3, 3, 60);
  const decltype(origin) pose1(4, 4, 120);
  const slam::ScanPoint2D origin_pos(0, 0); // id, x, y
  const decltype(origin_pos) pos0(3, 3);
  const decltype(origin_pos) pos1(4, 4);

  slam::ScanPoint2D rel, abso, abso1;

  rel = origin.ToRelativePoint(pos0);
  ASSERT_DOUBLE_EQ(rel.x(), 3.0);
  ASSERT_DOUBLE_EQ(rel.y(), 3.0);
  abso = origin.ToGlobalPoint(rel);
  ASSERT_NEAR(abso.x(), 3.0, 1.0e-7);
  ASSERT_NEAR(abso.y(), 3.0, 1.0e-7);
  origin.ToGlobalPoint(rel, abso1);
  ASSERT_NEAR(abso.x(), abso1.x(), 1.0e-7);
  ASSERT_NEAR(abso.y(), abso1.y(), 1.0e-7);

  rel = pose0.ToRelativePoint(origin_pos);
  ASSERT_NEAR(rel.x(), 3 * std::sqrt(2) * std::cos(165.0 / 180 * M_PI), 1.0e-7);
  ASSERT_NEAR(rel.y(), 3 * std::sqrt(2) * std::sin(165.0 / 180 * M_PI), 1.0e-7);
  abso = pose0.ToGlobalPoint(rel);
  ASSERT_NEAR(abso.x(), 0.0, 1.0e-7);
  ASSERT_NEAR(abso.y(), 0.0, 1.0e-7);
  pose0.ToGlobalPoint(rel, abso1);
  ASSERT_NEAR(abso.x(), abso1.x(), 1.0e-7);
  ASSERT_NEAR(abso.y(), abso1.y(), 1.0e-7);

  rel = pose1.ToRelativePoint(pos0);
  ASSERT_NEAR(rel.x(), std::sqrt(2) * std::cos(105.0 / 180 * M_PI), 1.0e-7);
  ASSERT_NEAR(rel.y(), std::sqrt(2) * std::sin(105.0 / 180 * M_PI), 1.0e-7);
  abso = pose1.ToGlobalPoint(rel);
  ASSERT_NEAR(abso.x(), 3.0, 1.0e-7);
  ASSERT_NEAR(abso.y(), 3.0, 1.0e-7);
  pose1.ToGlobalPoint(rel, abso1);
  ASSERT_NEAR(abso.x(), abso1.x(), 1.0e-7);
  ASSERT_NEAR(abso.y(), abso1.y(), 1.0e-7);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
