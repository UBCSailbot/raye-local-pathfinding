import math
import matplotlib.pyplot as plt
from local_pathfinding.msg import latlon
from utilities import getDesiredHeading
import numpy as np

position = latlon()
position.lat = 0.5
position.lon = 0.5
localPath = [latlon(0, 0), latlon(1, 1)]

def localWaypointReached(position, localPath, localPathIndex):
    previousWaypoint = localPath[localPathIndex - 1]
    localWaypoint = localPath[localPathIndex]
    isSlopePos = localWaypoint.lat < previousWaypoint.lat
    isStartNorth = isSlopePos
    isStartEast = localWaypoint.lon > previousWaypoint.lon

    angle = getDesiredHeading(previousWaypoint, localWaypoint)
    tangentVector = [math.cos(angle), math.sin(angle)]
    tangentSlope = tangentVector[1] / tangentVector[0]
    normal = [tangentVector[1], tangentVector[0]]
    normalSlope = math.fabs(normal[1] / normal[0])
#    normal = math.fabs(math.tan(math.radians(getDesiredHeading(previousWaypoint, localWaypoint) - math.pi / 2)))

    #if init slope is pos then normal is neg
    if isSlopePos:
        normalSlope = -normalSlope

#this is wrong this gives the previous waypoint location in the new coord system but not boatx and boaty
    startX = localWaypoint.lon - previousWaypoint.lon if isStartEast else previousWaypoint.lon - localWaypoint.lon
    startY = previousWaypoint.lat - localWaypoint.lat if isStartNorth else localWaypoint.lat - previousWaypoint.lat

    isBoatEast = localWaypoint.lon > position.lon
    isBoatNorth = position.lat > localWaypoint.lat
    boatX = localWaypoint.lon - position.lon if isBoatEast else position.lon - localWaypoint.lon
    boatY = position.lat- localWaypoint.lat if isBoatNorth else localWaypoint.lat - position.lat


    plt.plot([previousWaypoint.lon], [previousWaypoint.lat], marker = "o", markersize="3", color="green")
    plt.plot([0], [0], marker = 'o', markersize=3, color="red")
    plt.plot([startX], [startY], marker="o", markersize=3, color="green")
    plt.plot([boatX], [boatY], marker = "o", markersize=5, color = "black")
    xvalues = [0, previousWaypoint.lon]
    yvalues = [0, previousWaypoint.lat]
    plt.plot(xvalues, yvalues)
    x = np.linspace(0, 1, 100)
    y = normalSlope * x
    plt.plot(x, y, '-y')
    y = tangentSlope * x
    plt.plot(x, y, '-r')
    plt.show()
    
    y = lambda x: normal * x
    x = lambda y: y / float(normal)

    if isStartNorth and boatY < y(boatX):
        return True
    elif boatY > y(boatX):
        return True

    if isStartEast and boatX < x(boatY):
        return True
    elif boatX > x(boatY):
        return True

    return False

print(localWaypointReached(position, localPath, 0))


