import math
import matplotlib.pyplot as plt
from local_pathfinding.msg import latlon
from utilities import getDesiredHeading
import numpy as np

class State:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def getX(self):
        return self.x
    def getY(self):
        return self.y

'''
position = latlon()
position.lat = -100 
position.lon = 140 

end = latlon()
end.lat = 10.3599
end.lon = -17.03663
start = latlon()
start.lat = 8.46696
start.lon = -47.03663
'''

start = State(140, -100)
end = State(-17.2, 10.0)
position = State(100.0, -140.0)

localPath = [start, end]
def localWaypointReached(position, localPath, localPathIndex):
    position = latlon(float(position.getY()), float(position.getX()))
    previousWaypoint = latlon(float(localPath[localPathIndex - 1].getY()), float(localPath[localPathIndex - 1].getX()))
    localWaypoint = latlon(float(localPath[localPathIndex].getY()), float(localPath[localPathIndex].getX()))
    isStartNorth = localWaypoint.lat < previousWaypoint.lat 
    isStartEast = localWaypoint.lon < previousWaypoint.lon

    tangentSlope = (localWaypoint.lat - previousWaypoint.lat) / (localWaypoint.lon - previousWaypoint.lon)
    normalSlope = -1/tangentSlope

    startX = previousWaypoint.lon - localWaypoint.lon
    startY = previousWaypoint.lat - localWaypoint.lat 
    boatX = position.lon - localWaypoint.lon 
    boatY = position.lat - localWaypoint.lat

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

    print "isStartEast", isStartEast
    print "x(y) = ", x(boatY)
    print "boatX = ", boatX
    
    if isStartNorth: 
        if boatY < y(boatX):
            return True
    elif boatY > y(boatX):
        return True

    if isStartEast: 
        if boatX < x(boatY):
            return True
    elif boatX > x(boatY):
        return True

    return False

print(localWaypointReached(position, localPath, 1))
