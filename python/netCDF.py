import numpy as np
import argparse
import timeit
import os
from netCDF4 import Dataset

THRESHOLD = -5  #We don't want to go anywhere less than 5 meters deep (This is a placeholder)

start = timeit.default_timer()

#Locating netCDF file
dirname = os.path.dirname(__file__)
filename = os.path.join(dirname, 'gebco_2019_n48.554153_s20.875637_w-156.680558_e-124.420409.nc')

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

#All GPS locations of land above threshold depth, with a resolution of 0.00416667 degrees
print 'Locations of land or shallow water:'
for i in range(lat_start, lat_end):
    for j in range(lon_start, lon_end):
        elevation = nc.variables['elevation'][i][j]
        if (elevation > THRESHOLD):
            print nc.variables['lat'][i],  (nc.variables['lon'][j])
            count = 1;

stop = timeit.default_timer()

if (count == 0):
    print 'None!'

print 'Took', stop - start,'seconds'
