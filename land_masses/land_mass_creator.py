#!/usr/bin/env python

import argparse
from datetime import datetime
from netCDF4 import Dataset
import os
import timeit

CURR_DIR = os.path.dirname(__file__)
NETCDF_FILE = os.path.join(CURR_DIR, 'SalishSea.nc')

if __name__ == '__main__':
    start = timeit.default_timer()

    parser = argparse.ArgumentParser(description='Returns locations of shallow water or land, given coordinates')
    parser.add_argument('south', type=float, help='bottom latitude bound')
    parser.add_argument('north', type=float, help='top latitude bound')
    parser.add_argument('west', type=float, help='left longitude bound')
    parser.add_argument('east', type=float, help='right longitude bound')
    parser.add_argument('-d', '--threshold_depth', type=float, default=5,
                        help='maximum depth below sea level of the returned location (in metres, exclusive, default 5)')
    args = parser.parse_args()

    # Bounds of our netCDF file
    nc = Dataset(NETCDF_FILE, 'r')
    lat_min = nc.variables['lat'][0]
    lat_max = nc.variables['lat'][nc.variables['lat'].shape[0]-1]
    lon_min = nc.variables['lon'][0]
    lon_max = nc.variables['lon'][nc.variables['lon'].shape[0]-1]
    print 'netCDF dimensions:', lat_min, lat_max, lon_min, lon_max

    # Calculating indices of coordinate arguments
    lat_start = int((args.south - lat_min) / 0.004166666667)
    lat_end = int((args.north - lat_min) / 0.004166666667)
    lon_start = int((args.west - lon_min) / 0.004166666667)
    lon_end = int((args.east - lon_min) / 0.004166666667)
    print('nc index bounds: lat=[{},{}) lon=[{},{})'.format(lat_start, lat_end, lon_start, lon_end))

    land_mass_file_list = os.path.join(CURR_DIR, 'land_depth_{}m_{}'.format(args.threshold_depth,
                                                                       datetime.now().strftime('%m_%d_%Y-%H_%M_%S')))
    kml = open(land_mass_file_list + '.kml', 'w')
    kml.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
              "<kml xmlns=\"http://earth.google.com/kml/2.0\"><Document><Placemark><LineString><coordinates>\n")

    csv = open(land_mass_file_list + '.csv', 'w')

    # All GPS locations of land above threshold depth, with a resolution of 0.00416667 degrees
    endPoints = []
    for i in range(lat_start, lat_end):
        isLeftmostCoord = True  # order of latitudes is from left to right, so first land coordinate
        landPresentInLat = False  # true when there is at least one land coordinate for a latitude

        for j in range(lon_start, lon_end):
            count = 0
            elevation = nc.variables['elevation'][i][j]
            if (elevation > -1 * args.threshold_depth):
                landPresentInLat = True

                lat = nc.variables['lat'][i]
                lon = nc.variables['lon'][j]

                # Write the leftmost coordinate for each latitude
                if (isLeftmostCoord):
                    csv.write(str(lat) + ',' + str(lon) + '\n')
                    kml.write(str(lon) + ',' + str(lat) + '\n')
                    isLeftmostCoord = False

                # Write the current rightmost coordinate for each latitude
                currRightmostLon = lon
                currRightmostLat = lat

        # Write the rightmost coordinate for each latitude if it exists
        if (landPresentInLat):
            endPoints.append([currRightmostLat, currRightmostLon])

    # write rightmost coordinates in reverse order
    endPoints.reverse()
    for k in endPoints:
        csv.write(str(k[0]) + ',' + str(k[1]) + '\n')
        kml.write(str(k[1]) + ',' + str(k[0]) + '\n')

    csv.close()
    kml.write("</coordinates></LineString></Placemark></Document></kml>\n")
    kml.close()
    print 'Saved to', land_mass_file_list + '.(csv|kml)'

    stop = timeit.default_timer()
    print 'Took', stop - start, 'seconds'
