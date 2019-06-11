// Copyright 2019 Chuck Taylor
// Port to C++ by Alex Hajnal
//
// This is a simple port of the code on the Geographic/UTM Coordinate Converter (1) page from Javascript to C++.
// Using this you can easily convert between UTM and WGS84 (latitude and longitude).
// Accuracy seems to be around 50cm (I suspect rounding errors are limiting precision).
// This code is provided as-is and has been minimally tested; enjoy but use at your own risk!
// The license for UTM.cpp and UTM.h is the same as the original Javascript:
// "The C++ source code in UTM.cpp and UTM.h may be copied and reused without restriction."
//
// 1) http://home.hiwaay.net/~taylorc/toolbox/geography/geoutm.html

// QGC Note: This file has been slightly modified to prevent possible conflicts with other parts of the system

#ifndef UTIL_UTM_H_
#define UTIL_UTM_H_
namespace UTM {

/**
 * Converts degrees to radians.
 * @param deg 
 * @return
 */
double DegToRad(double deg);

/**
 * Converts radians to degrees
 * @param rad
 * @return
 */
double RadToDeg(double rad);

/**
 *  ArcLengthOfMeridian
Computes the ellipsoidal distance from the equator to a point at a
given latitude.

Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.

 * @param phi Latitude of the point, in radians.
 * @return The ellipsoidal distance of the point from the equator, in meters.
 */
double ArcLengthOfMeridian(double phi);

/**
 * UTMCentralMeridian
 * Determines the central meridian for the given UTM zone.

 * @param zone An integer value designating the UTM zone, range [1,60].
 * @return  The central meridian for the given UTM zone, in radians
   Range of the central meridian is the radian equivalent of [-177,+177].
 */
double UTMCentralMeridian(int zone);

/**
 * Computes the footpoint latitude for use in converting transverse
 * Mercator coordinates to ellipsoidal coordinates.
 *
 * Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
 *  GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
 * @param y y - The UTM northing coordinate, in meters.
 * @return The footpoint latitude, in radians.
 */
double FootpointLatitude(double y);

/**
 * MapLatLonToXY
 * Converts a latitude/longitude pair to x and y coordinates in the
 * Transverse Mercator projection.  Note that Transverse Mercator is not
 * the same as UTM; a scale factor is required to convert between them.

 * Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
 * GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
 *
 * @param phi Latitude of the point, in radians.
 * @param lambda Longitude of the point, in radians.
 * @param lambda0 Longitude of the central meridian to be used, in radians.
 * @param x  The x coordinate of the computed point.
 * @param y The y coordinate of the computed poin
 */
void MapLatLonToXY(double phi, double lambda, double lambda0, double *x, double *y);

/**
 *  MapXYToLatLon
 * Converts x and y coordinates in the Transverse Mercator projection to
 * a latitude/longitude pair.  Note that Transverse Mercator is not
 * the same as UTM; a scale factor is required to convert between them.
 *
 * Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
 *   GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
 *
 *
 *   The local variables Nf, nuf2, tf, and tf2 serve the same purpose as
 *   N, nu2, t, and t2 in MapLatLonToXY, but they are computed with respect
 *   to the footpoint latitude phif.
 *
 *   x1frac, x2frac, x2poly, x3poly, etc. are to enhance readability and
 *   to optimize computations.
 *
 * @param x The easting of the point, in meters.
 * @param y  The northing of the point, in meters.
 * @param lambda0 Longitude of the central meridian to be used, in radians.
 * @param phi Latitude in radians.
 * @param lambda Longitude in radians.
 */
void MapXYToLatLon(double x, double y, double lambda0, double *phi, double *lambda);

/**
 *  LatLonToUTMXY
 * Converts a latitude/longitude pair to x and y coordinates in the
 * Universal Transverse Mercator projection.
 * @param lat Latitude of the point, in degrees.
 * @param lon  Longitude of the point, in degrees.
 * @param zone UTM zone to be used for calculating values for x and y. If zone is less than 1 or greater than 60,
 * the routine will determine the appropriate zone from the value of lon.
 * @param x The x coordinate (easting) of the computed point. (in meters)
 * @param y The y coordinate (northing) of the computed point. (in meters)
 * @return The UTM zone used for calculating the values of x and y.
 */
int LatLonToUTMXY(double lat, double lon, int zone, double *x, double *y);

/***
 * UTMXYToLatLon
 * Converts x and y coordinates in the Universal Transverse Mercator//   The UTM zone parameter should be in the range [1,60].
 * projection to a latitude/longitude pair.
 * @param x The easting of the point, in meters.
 * @param y The northing of the point, in meters.
 * @param zone The UTM zone in which the point lies.
 * @param southhemi True if the point is in the southern hemisphere; false otherwise.
 * @param lat The latitude of the point, in degrees.
 * @param lon The longitude of the point, in degrees.
 */
void UTMXYToLatLon(double x, double y, int zone, bool southhemi, double *lat, double *lon);
}  // namespace UTM

#endif  // UTIL_UTM_H_

