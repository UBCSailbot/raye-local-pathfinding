// Copyright 2019 UBC Sailbot

#ifndef UTIL_COORDINATETRANSLATORTEST_H_
#define UTIL_COORDINATETRANSLATORTEST_H_

#include <gtest/gtest.h>

static const double kTestLat = 51.141777;
static const double kTestLon = -114.286236;

class CoordinateTranslatorTest : public ::testing::Test {
 protected:
  CoordinateTranslatorTest();
};

#endif  // UTIL_COORDINATETRANSLATORTEST_H_
