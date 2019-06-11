// Copyright 2019 UBC Sailbot

#include "CoordinateTranslatorTest.h"
#include <math.h>
#include <util/CoordinateTranslator.h>

CoordinateTranslatorTest::CoordinateTranslatorTest() {}

TEST_F(CoordinateTranslatorTest, TestTranslationBase) {
  const Coordinate
      &coordinate = CoordinateTranslator::translateToReferenceFrame(kTestLat, kTestLon, kTestLat, kTestLon);
  EXPECT_EQ(0, coordinate.getX());
  EXPECT_EQ(0, coordinate.getY());
}

TEST_F(CoordinateTranslatorTest, TestTranslationOffset) {
  const Coordinate
      &coordinate = CoordinateTranslator::translateToReferenceFrame(kTestLat, kTestLon, 51.142047, kTestLon);

  ASSERT_NEAR(coordinate.getY(), 30, 1.2);
  ASSERT_NEAR(coordinate.getX(), 0, 1.2);
}

TEST_F(CoordinateTranslatorTest, TestTranslationMapping) {
  const Coordinate
      &coordinate = CoordinateTranslator::translateToReferenceFrame(kTestLat, kTestLon, kTestLat, kTestLon);
  const std::pair<double, double>
      &output = CoordinateTranslator::translateFromReferenceFrame(coordinate, kTestLat, kTestLon);

  ASSERT_NEAR(output.first, kTestLat, 0.0001);
  ASSERT_NEAR(output.second, kTestLon, 0.0001);
}

