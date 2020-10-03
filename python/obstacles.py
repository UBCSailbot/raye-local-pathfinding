import math
import numpy as np
import matplotlib.pyplot as plt
import utilities as utils
from matplotlib import patches
from local_pathfinding.msg import latlon
from geopy.distance import distance

# Constants
OBSTACLE_MAX_TIME_TO_LOC_HOURS = 3  # Do not extend objects more than X hours distance
WEDGE_EXPAND_ANGLE_DEGREES = 10.0
AIS_BOAT_RADIUS_KM = 0.2
AIS_BOAT_LENGTH_KM = 1
AIS_BOAT_CIRCLE_SPACING_KM = AIS_BOAT_RADIUS_KM * 1.5  # Distance between circles that make up an AIS boat


def getProjectedPosition(aisShip, sailbotPosition, sailbotSpeedKmph, referenceLatlon):
    """ Calculate position where we project the boat to be if the sailbot moves directly towards it """
    # Get positions of ais boat and sailbot
    aisX, aisY = utils.latlonToXY(latlon(aisShip.lat, aisShip.lon), referenceLatlon)
    sailbotPositionX, sailbotPositionY = utils.latlonToXY(latlon(sailbotPosition.lat, sailbotPosition.lon),
                                                          referenceLatlon)

    # Project ais boat's velocity vector onto the line between the boats.
    # Will be +ve if moving towards sailbot, -ve if moving away form sailbot
    angleBoatToSailbotDegrees = math.degrees(math.atan2(sailbotPositionY - aisY, sailbotPositionX - aisX))
    boatSpeedInDirToSailbotKmph = (aisShip.speedKmph *
                                   math.cos(math.radians(aisShip.headingDegrees - angleBoatToSailbotDegrees)))

    # Calculate MINIMUM time it would take for the boats to collide, but have an upper bound
    MAX_TIME_TO_LOC_HOURS = 3
    distanceBetweenBoatsKm = distance((aisShip.lat, aisShip.lon), (sailbotPosition.lat, sailbotPosition.lon)).kilometers
    if sailbotSpeedKmph + boatSpeedInDirToSailbotKmph == 0:
        smallestTimeToLocHours = MAX_TIME_TO_LOC_HOURS
    else:
        smallestTimeToLocHours = distanceBetweenBoatsKm / (sailbotSpeedKmph + boatSpeedInDirToSailbotKmph)
        smallestTimeToLocHours = smallestTimeToLocHours if smallestTimeToLocHours < MAX_TIME_TO_LOC_HOURS
                                                        else MAX_TIME_TO_LOC_HOURS
    minimumBoatDistanceTravelledKm = smallestTimeToLocHours * aisShip.speedKmph

    # Calculate projected position
    projectedX = aisX + minimumBoatDistanceTravelledKm * math.cos(math.radians(aisShip.headingDegrees))
    projectedY = aisY + minimumBoatDistanceTravelledKm * math.sin(math.radians(aisShip.headingDegrees))
    projectedPosition = [projectedX, projectedY]
    return projectedPosition


def getObstacles(state, referenceLatlon):
    '''Creates a list of obstacles in xy coordinates. Uses the obstacle type from the rosparam "obstacle_type"

    Args:
       state (BoatState): State of the sailbot
       referenceLatlon (local_pathfinding.msg._latlon.latlon): Position of the reference point that will be at (0,0)

    Returns:
       list of obstacles that implement the obstacle interface
    '''
    ships, position, speedKmph = state.aisShip.ships, state.position, state.speedKmph
    obstacle_type = rospy.get_param('obstacle_type', 'hybrid_circle')
    obstacles = []
    if obstacle_type == "ellipse":
        for ship in ships:
            obstacles.append(EllipseObstacle(ship, position, speedKmph, referenceLatlon))
    elif obstacle_type == "wedge":
        for ship in ships:
            obstacles.append(Wedge(ship, position, speedKmph, referenceLatlon))
    elif obstacle_type == "circles":
        for ship in ships:
            obstacles.append(Circles(ship, position, speedKmph, referenceLatlon))
    elif obstacle_type == "hybrid_ellipse":
        for ship in ships:
            obstacles.append(HybridEllipse(ship, position, speedKmph, referenceLatlon))
    elif obstacle_type == "hybrid_circle":
        for ship in ships:
            obstacles.append(HybridCircle(ship, position, speedKmph, referenceLatlon))
    return obstacles


class ObstacleInterface:
    def __init__(self, aisShip, sailbotPosition, sailbotSpeedKmph, referenceLatlon):

    def __str__(self):
        pass

    def _extendObstacle(self, aisShip, sailbotPosition, sailbotSpeedKmph, referenceLatlon):
        """ Extends obstacle based on speed and heading """
        pass

    def getPatch(self, color):
        """ Add matplotlib.patches to axes """
        pass

    def isValid(self, xy):
        """ Checks validity of xy"""
        pass

    def clearance(self, xy):
        """ Return distance from obstacle to xy"""
        pass


class Ellipse():
    def __init__(self, x, y, height, width, angle):
        self.x = x
        self.y = y
        self.height = height
        self.width = width
        self.angle = angle

    def __str__(self):
        return str((self.x, self.y, self.height, self.width, self.angle))

    def isValid(self, xy):
        x = xy[0] - self.x
        y = xy[1] - self.y
        x_ = math.cos(math.radians(self.angle)) * x + math.sin(math.radians(self.angle)) * y
        y_ = -math.sin(math.radians(self.angle)) * x + math.cos(math.radians(self.angle)) * y
        distance_center_to_boat = math.sqrt(x_ ** 2 + y_ ** 2)
        angle_center_to_boat = math.degrees(math.atan2(y_, x_))
        angle_center_to_boat = (angle_center_to_boat + 360) % 360

        a = self.width * 0.5
        b = self.height * 0.5

        t_param = math.atan2(a * y_, b * x_)
        edge_pt = self._ellipseFormula(t_param)
        distance_to_edge = math.sqrt((edge_pt[0] - self.x) ** 2 + (edge_pt[1] - self.y) ** 2)

        delta = 0.001
        if distance_center_to_boat < distance_to_edge or math.fabs(distance_to_edge - distance_center_to_boat) <= delta:
            return False
        return True

    def _ellipseFormula(self, t):
        init_pt = np.array([self.x, self.y])
        a = 0.5 * self.width
        b = 0.5 * self.height
        rotation_col1 = np.array([math.cos(math.radians(self.angle)), math.sin(math.radians(self.angle))])
        rotation_col2 = np.array([-math.sin(math.radians(self.angle)), math.cos(math.radians(self.angle))])
        edge_pt = init_pt + a * math.cos(t) * rotation_col1 + b * math.sin(t) * rotation_col2
        return edge_pt

    def addPatch(self, axes):
        axes.add_patch(patches.Ellipse((self.x, self.y), self.width, self.height, self.angle))


class EllipseObstacle(ObstacleInterface, Ellipse):
    def __init__(self, aisShip, sailbotPosition, sailbotSpeedKmph, referenceLatlon):
        ObstacleInterface.__init__(self, aisShip, sailbotPosition, sailbotSpeedKmph, referenceLatlon)
        self._extendObstacle(self.aisShip, self.sailbotPosition, self.sailbotSpeedKmph, self.referenceLatlon)

    def __str__(self):
        return Ellipse.__str__(self)

    def _extendObstacle(self, aisShip, sailbotPosition, sailbotSpeedKmph, referenceLatlon):
        # Calculate width and height of ellipse
        timeExtendLengthHours = 0.4
        extendBoatLengthKm = aisShip.speedKmph * timeExtendLengthHours

        if extendBoatLengthKm == 0:
            width = AIS_BOAT_RADIUS_KM
        else:
            width = extendBoatLengthKm
        height = AIS_BOAT_RADIUS_KM
        angle = aisShip.headingDegrees

        # Calculate xy center of ellipse, which is has its tip at the projected position
        xy = [self.projectedX + extendBoatLengthKm * math.cos(math.radians(angle)) * 0.5,
              self.projectedY + extendBoatLengthKm * math.sin(math.radians(angle)) * 0.5]
        self.x, self.y = xy[0], xy[1]
        self.width, self.height = width, height
        self.angle = angle

    def isValid(self, xy):
        return Ellipse.isValid(self, xy)

    def _ellipseFormula(self, t):
        return Ellipse._ellipseFormula(self, t)

    def addPatch(self, axes):
        Ellipse.addPatch(self, axes)
        # TODO: Add patch for ellipse at current position

    def clearance(self, xy):
        # TODO: Make this clearance better
        return (self.x - xy[0])**2 + (self.y - xy[1])**2


class Wedge(ObstacleInterface):
    def __init__(self, aisShip, sailbotPosition, sailbotSpeedKmph, referenceLatlon):
        ObstacleInterface.__init__(self, aisShip, sailbotPosition, sailbotSpeedKmph, referenceLatlon)
        self._extendObstacle(self.aisShip, self.sailbotPosition, self.sailbotSpeedKmph, self.referenceLatlon)

    def __str__(self):
        return str((self.x, self.y, self.radius, self.theta1, self.theta2))

    def _extendObstacle(self, aisShip, sailbotPosition, sailbotSpeedKmph, referenceLatlon):
        theta1 = aisShip.headingDegrees - WEDGE_EXPAND_ANGLE_DEGREES / 2.0
        theta2 = aisShip.headingDegrees + WEDGE_EXPAND_ANGLE_DEGREES / 2.0

        # Ensure theta1 >= 0 and theta2 >= theta1
        if theta1 < 0:
            theta1 += 360
        while theta2 < theta1:
            theta2 += 360

        # Defines how long the wedge should be
        timeExtendLengthHours = 1
        radius = aisShip.speedKmph * timeExtendLengthHours
        self.x, self.y = self.projectedX, self.projectedY
        self.radius = radius
        self.theta1, self.theta2 = theta1, theta2

    def addPatch(self, axes):
        projectedPositionWedge = patches.Wedge((self.x, self.y), self.radius, self.theta1, self.theta2)
        projectedPositionWedge.set_color('blue')
        currentPositionWedge = patches.Wedge((self.currentX, self.currentY), self.radius, self.theta1, self.theta2)
        currentPositionWedge.set_color('red')
        axes.add_patch(projectedPositionWedge)
        axes.add_patch(currentPositionWedge)

    def isValid(self, xy):
        angle = math.degrees(math.atan2(xy[1] - self.y, xy[0] - self.x))

        # Ensure that angle >= theta1
        while angle < self.theta1:
            angle += 360

        distance = math.sqrt((xy[1] - self.y) ** 2 + (xy[0] - self.x) ** 2)
        if (angle > self.theta1 and angle < self.theta2 and distance <= self.radius):
            return False
        return True

    def clearance(self, xy):
        # TODO: Make this clearance better
        return (self.x - xy[0])**2 + (self.y - xy[1])**2


class Circles(ObstacleInterface):
    def __init__(self, aisShip, sailbotPosition, sailbotSpeedKmph, referenceLatlon):
        ObstacleInterface.__init__(self, aisShip, sailbotPosition, sailbotSpeedKmph, referenceLatlon)
        self._extendObstacle(self.aisShip, self.sailbotPosition, self.sailbotSpeedKmph, self.referenceLatlon)

    def isValid(self, xy):
        for obstacle in self.obstacles:
            if not obstacle.isValid(xy):
                return False
        return True

    def _extendObstacle(self, aisShip, sailbotPosition, sailbotSpeedKmph, referenceLatlon):
        self.obstacles = []
        aisX, aisY = utils.latlonToXY(latlon(aisShip.lat, aisShip.lon), referenceLatlon)
        # Calculate length to extend boat
        distanceToBoatKm = distance((aisShip.lat, aisShip.lon), (sailbotPosition.lat, sailbotPosition.lon)).kilometers
        if sailbotSpeedKmph == 0 or distanceToBoatKm / sailbotSpeedKmph > OBSTACLE_MAX_TIME_TO_LOC_HOURS:
            timeToLocHours = OBSTACLE_MAX_TIME_TO_LOC_HOURS
        else:
            timeToLocHours = distanceToBoatKm / sailbotSpeedKmph
        extendBoatLengthKm = aisShip.speedKmph * timeToLocHours

        if extendBoatLengthKm == 0:
            self.obstacles.append(Circle(aisX, aisY, AIS_BOAT_RADIUS_KM))

        if aisShip.headingDegrees == 90 or aisShip.headingDegrees == 270:
            if aisShip.headingDegrees == 90:
                endY = aisY + extendBoatLengthKm
                yRange = np.arange(aisY, endY, AIS_BOAT_CIRCLE_SPACING_KM)
            if aisShip.headingDegrees == 270:
                endY = aisY - extendBoatLengthKm
                yRange = np.arange(endY, aisY, AIS_BOAT_CIRCLE_SPACING_KM)
            for y in yRange:
                # Multiplier to increase size of circles showing where the boat will be in the future in range [1, 2]
                multiplier = 1 + abs(float(y - aisY) / (endY - aisY))
                self.obstacles.append(Circle(aisX, y, AIS_BOAT_RADIUS_KM * multiplier))
        else:
            isHeadingWest = aisShip.headingDegrees < 270 and aisShip.headingDegrees > 90
            slope = math.tan(math.radians(aisShip.headingDegrees))
            dx = AIS_BOAT_CIRCLE_SPACING_KM / math.sqrt(1 + slope**2)

            if aisX > 0:
                b = aisY + slope * -math.fabs(aisX)
            else:
                b = aisY + slope * math.fabs(aisX)
            xDistTravelled = math.fabs(extendBoatLengthKm * math.cos(math.radians(aisShip.headingDegrees)))
            def y(x): return slope * x + b
            if isHeadingWest:
                endX = aisX - xDistTravelled
                xRange = np.arange(endX, aisX, dx)
            else:
                endX = aisX + xDistTravelled
                xRange = np.arange(aisX, endX, dx)
            for x in xRange:
                # Multiplier to increase size of circles showing where the boat will be in the future in range [1, 2]
                multiplier = 1 + abs(float(x - aisX) / (endX - aisX))
                self.obstacles.append(Circle(x, y(x), AIS_BOAT_RADIUS_KM * multiplier))

    def addPatch(self, axes):
        for obstacle in self.obstacles:
            obstacle.addPatch(axes)

    def clearance(self, xy):
        # TODO: Make this clearance better
        return (self.obstacles[0].x - xy[0])**2 + (self.obstacles[0].y - xy[1])**2


class Circle():
    """ Helper class for Circles representation"""

    def __init__(self, x, y, radius, color="blue"):
        self.x = x
        self.y = y
        self.radius = radius
        self.color = color

    def isValid(self, xy):
        x, y = xy
        if math.sqrt(pow(x - self.x, 2) + pow(y - self.y, 2)) - self.radius <= 0:
            return False
        return True

    def addPatch(self, axes):
        circlePatch = plt.Circle((self.x, self.y), radius=self.radius)
        circlePatch.set_color(self.color)
        axes.add_patch(circlePatch)

class HybridEllipse(ObstacleInterface):
    def __init__(self, aisShip, sailbotPosition, sailbotSpeedKmph, referenceLatlon):
        ObstacleInterface.__init__(self, aisShip, sailbotPosition, sailbotSpeedKmph, referenceLatlon)
        self._extendObstacle(self.aisShip, self.sailbotPosition, self.sailbotSpeedKmph, self.referenceLatlon)
        self.xy = utils.latlonToXY(latlon(aisShip.lat, aisShip.lon), referenceLatlon)
        self.ellipse = Ellipse(self.xy[0], self.xy[1], AIS_BOAT_RADIUS_KM, AIS_BOAT_LENGTH_KM,
                               aisShip.headingDegrees)

    def __str__(self):
        return str(self.ellipse) + str(self.wedge)

    def _extendObstacle(self, aisShip, sailbotPosition, sailbotSpeedKmph, referenceLatlon):
        self.wedge = Wedge(aisShip, sailbotPosition, sailbotSpeedKmph, referenceLatlon)

    def addPatch(self, axes):
        self.wedge.addPatch(axes)
        self.ellipse.addPatch(axes)

    def isValid(self, xy):
        return (self.wedge.isValid(xy) and self.ellipse.isValid(xy))

    def clearance(self, xy):
        return (self.xy[0] - xy[0])**2 + (self.xy[1] - xy[1])**2


class HybridCircle(ObstacleInterface):
    def __init__(self, aisShip, sailbotPosition, sailbotSpeedKmph, referenceLatlon):
        # Store member variables
        self.aisShip = aisShip
        self.sailbotPosition = sailbotPosition
        self.sailbotSpeedKmph = sailbotSpeedKmph
        self.referenceLatlon = referenceLatlon
        self.currentX, self.currentY = utils.latlonToXY(latlon(aisShip.lat, aisShip.lon), referenceLatlon)
        self.projectedX, self.projectedY = getProjectedPosition(aisShip, sailbotPosition,
                                                                sailbotSpeedKmph, referenceLatlon)
        self._extendObstacle(self.aisShip, self.sailbotPosition, self.sailbotSpeedKmph, self.referenceLatlon)

    def __str__(self):
        return str(self.circle) + str(self.wedge)

    def _extendObstacle(self, aisShip, sailbotPosition, sailbotSpeedKmph, referenceLatlon):
        self.origWedge = Wedge(aisShip, sailbotPosition, sailbotSpeedKmph, referenceLatlon)
        self.wedge = Wedge(aisShip, sailbotPosition, sailbotSpeedKmph, referenceLatlon)
        self.origCircle = Circle(self.currentX, self.currentY, AIS_BOAT_RADIUS_KM, "red")
        self.circle = Circle(self.projectedX, self.projectedY, AIS_BOAT_RADIUS_KM, "blue")

    def addPatch(self, axes):
        self.wedge.addPatch(axes)
        self.circle.addPatch(axes)
        self.origCircle.addPatch(axes)

    def isValid(self, xy):
        return (self.wedge.isValid(xy) and self.circle.isValid(xy))

    def clearance(self, xy):
        return (self.projectedX - xy[0])**2 + (self.projectedY - xy[1])**2
