import numpy as np
import argparse
import timeit
import os
import sys
from netCDF4 import Dataset

THRESHOLD = -5  #We don't want to go anywhere less than 5 meters deep (This is a placeholder)

start = timeit.default_timer()

#Locating netCDF file
dirname = os.path.dirname(__file__)
filename = os.path.join(dirname, 'SalishSea.nc')

nc = Dataset(filename,'r')

parser = argparse.ArgumentParser(description='Returns locations of shallow water or land, given coordinates',
usage = 'netCDF.py [south] [north] [west] [east]')
parser.add_argument("--test", nargs='+', type=float)
args = parser.parse_args()
south = args.test[0]
north = args.test[1]
west = args.test[2]
east = args.test[3]

#Tracks if there are "no-go zones" in current search
count = 0

#Bounds of our netCDF file
lat_min = nc.variables['lat'][0]
lat_max = nc.variables['lat'][nc.variables['lat'].shape[0]-1]
lon_min = nc.variables['lon'][0]
lon_max = nc.variables['lon'][nc.variables['lon'].shape[0]-1]

print 'netCDF dimensions: ', lat_min, lat_max, lon_min, lon_max

#Calculating indices of coordinate arguments
lat_start = int((south-lat_min)/0.004166666667)
lat_end = int((north-lat_min)/0.004166666667)
lon_start = int((west-lon_min)/0.004166666667)
lon_end = int((east-lon_min)/0.004166666667)

kml = open('Land.kml', 'w')
kml.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
      "<kml xmlns=\"http://earth.google.com/kml/2.0\"><Document><Placemark><LineString><coordinates>\n")

endPoints = [[]]

txt = open('Land.txt', 'w')

#All GPS locations of land above threshold depth, with a resolution of 0.00416667 degrees
print 'Locations of land or shallow water:'
for i in range(lat_start, lat_end):
    lonCount = 0
    if (count > 0):
         endPoints.append([lastLat,lastLon])
    for j in range(lon_start, lon_end):
        count = 0
        elevation = nc.variables['elevation'][i][j]
        if (elevation > THRESHOLD):
            lat = nc.variables['lat'][i]
            lon = nc.variables['lon'][j]
            if (lonCount == 0):
                txt.write(str(lat) + ',' + str(lon) + '\n')
                kml.write(str(lon) + ',' + str(lat) + '\n')
                lonCount = 1
            lastLon = lon
            lastLat = lat
            count = 1;

endPoints.reverse()
endPoints.pop()

for k in endPoints:
    txt.write(str(k[0]) + ',' + str(k[1]) + '\n')
    kml.write(str(k[1]) + ',' + str(k[0]) + '\n')

stop = timeit.default_timer()

kml.write("</coordinates></LineString></Placemark></Document></kml>\n")

print 'Took', stop - start,'seconds'
