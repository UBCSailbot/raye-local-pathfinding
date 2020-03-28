import math
import matplotlib.pyplot as plt
from utilities import * 
from updated_geometric_planner import Obstacle
import numpy

#obstacle = Obstacle(0, 0, 5, 2, 45)
#plt.xlim(-10, 10)
#plt.ylim(-10, 10)
#ax = plt.gca()
#for i in range(0, 360):
#    point = ellipseFormula(obstacle, i)
#    point = list(point)
#    plt.plot([point[0]], [point[1]], marker = 'o', markersize= 3, color="red")
#ax.add_patch(patches.Ellipse((obstacle.x, obstacle.y), obstacle.width, obstacle.height, obstacle.angle))
#plt.show()
#
#print(isValid([10, 10], [obstacle]))
#print(isValid([1, 1], [obstacle]))
#print(isValid([1.579, 1.579], [obstacle]))
#print(isValid([1.6, 1.6], [obstacle]))
#print(isValid([1.8, 1.8], [obstacle]))
#print()
#
#print(isValid([-1.6, -1.6], [obstacle]))
#print(isValid([-1.8, -1.8], [obstacle]))
#

currentLatlon = latlon(0, 0)
referenceLatlon = currentLatlon
shipLatlon = XYToLatlon(xy=(1, 0), referenceLatlon=referenceLatlon)
ships = [AISShip(ID=1000, lat=shipLatlon.lat, lon=shipLatlon.lon, headingDegrees=HEADING_WEST, speedKmph=1)]
obstacles = extendObstaclesArray(aisArray=ships, sailbotPosition=currentLatlon, sailbotSpeedKmph=1, referenceLatlon=referenceLatlon)

ax = plt.gca()
for obstacle in obstacles:
 ax.add_patch(patches.Ellipse((obstacle.x, obstacle.y), obstacle.width, obstacle.height, obstacle.angle))
plt.show()

print isValid([0, 0], obstacles)
print isValid([1, 0], obstacles)
print isValid([2, 0], obstacles)
