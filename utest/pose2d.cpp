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

TEST(Pose2D, CalcGloalAndLocalPose){
  const slam::Pose2D origin(0, 0, 0);
  const slam::Pose2D pose0(3, 3, 60);
  const slam::Pose2D pose1(4, 4, 120);

  slam::Pose2D rel;
  slam::Pose2D abso;

  
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
