// Copyright 2019 UBC Sailbot

#include "CoordinateTranslator.h"
#include "UTM.h"

namespace CoordinateTranslator {

Coordinate translateToReferenceFrame(double baseLatitude,
                                     double baseLongitude,
                                     double offsetLatitude,
                                     double offsetLongitude) {
  double base_x;
  double base_y;
  UTM::LatLonToUTMXY(baseLatitude, baseLongitude, 0, base_x, base_y);

  double offset_x;
  double offset_y;
  UTM::LatLonToUTMXY(offsetLatitude, offsetLongitude, 0, offset_x, offset_y);
  return {offset_x - base_x, offset_y - base_y};
}

std::pair<double, double> translateFromReferenceFrame(const Coordinate &e, double baseLatitude, double baseLongitude) {
  double base_x;
  double base_y;
  int zone = UTM::LatLonToUTMXY(baseLatitude, baseLongitude, 0, base_x, base_y);

  double out_lat;
  double out_lon;
  UTM::UTMXYToLatLon(base_x + e.getX(), base_y + e.getY(), zone, baseLatitude < 0, out_lat, out_lon);

  return {out_lat, out_lon};
}
}  // namespace CoordinateTranslator
