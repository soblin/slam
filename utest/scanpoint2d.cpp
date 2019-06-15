#include "slam/geometry/ScanPoint2D.h"
#include <cmath>
#include <memory>

#include "gtest/gtest.h"

TEST(ScanPoint2D, methods) {
  auto ptr = std::make_shared<slam::ScanPoint2D>(3, 4);

  ASSERT_FLOAT_EQ(ptr->x(), 3.0);
  ASSERT_FLOAT_EQ(ptr->y(), 4.0);
  //  ASSERT_FLOAT_EQ(ptr->id(), 11);

  ptr->SetData(30, 40);
  ASSERT_FLOAT_EQ(ptr->x(), 30);
  ASSERT_FLOAT_EQ(ptr->y(), 40);
  //  ASSERT_FLOAT_EQ(ptr->id(), 111);

  double r = 13;
  double rad = 2.0 / 3 * M_PI;
  ptr->CalcXY(r, rad);
  ASSERT_NEAR(ptr->x(), 13 * std::cos(2.0 / 3 * M_PI), 1.0e-7);
  ASSERT_NEAR(ptr->y(), 13 * std::sin(2.0 / 3 * M_PI), 1.0e-7);

  // ptr->SetId(1111);
  // ASSERT_EQ(ptr->id(), 1111);

  // ptr->SetNormal(10, 100);
  // ASSERT_EQ(ptr->nx(), 10);
  // ASSERT_EQ(ptr->ny(), 100);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
