// Copyright 2019 UBC Sailbot

#ifndef UTIL_COORDINATETRANSLATOR_H_
#define UTIL_COORDINATETRANSLATOR_H_

#include <datatypes/Coordinate.h>
#include <utility>
namespace CoordinateTranslator {

/**
 *
 * @param baseLatitude
 * @param baseLongitude
 * @param offsetLatitude
 * @param offsetLongitude
 * @return
 */
Coordinate translateToReferenceFrame(double baseLatitude,
                                            double baseLongitude,
                                            double offsetLatitude,
                                            double offsetLongitude);
/**
 *
 * @param e
 * @return Pair representing Latitude and Longitude
 */
std::pair<double, double> translateFromReferenceFrame(const Coordinate &e, double baseLatitude, double baseLongitude);


}  // namespace CoordinateTranslator

#endif  // UTIL_COORDINATETRANSLATOR_H_
