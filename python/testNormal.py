import math
import matplotlib.pyplot as plt
from local_pathfinding.msg import latlon
from utilities import getDesiredHeading
import numpy as np

position = latlon()
position.lat = -1 
position.lon = -1

start = latlon()
start.lat = 8.46696
start.lon = -17.03663
end = latlon()
end.lat = 65.35996
end.lon = -17.03663

localPath = [start, end]

def localWaypointReached(position, localPath, localPathIndex):
    previousWaypoint = localPath[localPathIndex - 1]
    localWaypoint = localPath[localPathIndex]
#    isSlopePos = localWaypoint.lat < previousWaypoint.lat
    isStartNorth = localWaypoint.lat < previousWaypoint.lat 
    isStartEast = localWaypoint.lon > previousWaypoint.lon

    print "start: " , previousWaypoint 
    print ""
    print "end: " ,  localWaypoint
    angle = getDesiredHeading(previousWaypoint, localWaypoint)
    print angle
    tangentVector = [math.cos(math.radians(angle)), math.sin(math.radians(angle))]
    tangentSlope = tangentVector[1] / tangentVector[0]
    normal = [-tangentVector[1], tangentVector[0]]
    normalSlope = (normal[1] / normal[0])
#    normal = math.fabs(math.tan(math.radians(getDesiredHeading(previousWaypoint, localWaypoint) - math.pi / 2)))

    #if init slope is pos then normal is neg
#    if isSlopePos:
#        normalSlope = -normalSlope

#this is wrong this gives the previous waypoint location in the new coord system but not boatx and boaty
    startX = localWaypoint.lon - previousWaypoint.lon if isStartEast else previousWaypoint.lon - localWaypoint.lon
    startY = localWaypoint.lat - previousWaypoint.lat if isStartNorth else previousWaypoint.lat - localWaypoint.lat

    isBoatEast = localWaypoint.lon > position.lon
    isBoatNorth = position.lat > localWaypoint.lat
    boatX = localWaypoint.lon - position.lon if isBoatEast else position.lon - localWaypoint.lon
    boatY = localWaypoint.lat - position.lat if isBoatNorth else position.lat - localWaypoint.lat

    print "tangent slope: {}".format(tangentSlope)
    print "normal slope: {}".format(normalSlope)
    print "Previous Waypoint Adjusted:   x: {}, y: y {}".format(startX, startY)
    print "Boat:    x: {}, y: {}".format(boatX, boatY)

#    plt.plot([previousWaypoint.lon], [previousWaypoint.lat], marker = "o", markersize="3", color="green")
    plt.xlim(-200, 200)
    plt.ylim(-200, 200)
    plt.plot([0], [0], marker = 'o', markersize=10, color="red")
    plt.plot([startX], [startY], marker="o", markersize=10, color="green")
    plt.plot([boatX], [boatY], marker = "o", markersize=10, color = "black")
    xvalues = [0, startX] 
    yvalues = [0, startY]
    plt.plot(xvalues, yvalues, "-g")
    x = np.linspace(-200, 200, 100)
    y = normalSlope * x
    plt.plot(x, y, '-r')
    y = tangentSlope * x
    plt.plot(x, y, '-b')
    plt.show()
    
    y = lambda x: normalSlope * x
    x = lambda y: y / float(normalSlope)

    print "isStartNorth", isStartNorth
    print "y(x) = ", y(boatX)
    print "boatY = ", boatY

    if isStartNorth and boatY < y(boatX):
        return True
    elif boatY > y(boatX):
        return True

    if isStartEast and boatX < x(boatY):
        return True
    elif boatX > x(boatY):
        return True

    return False

print(localWaypointReached(position, localPath, 1))

print getDesiredHeading(start, end)
