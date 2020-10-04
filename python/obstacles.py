import math
import numpy as np
import matplotlib.pyplot as plt
import utilities as utils
from matplotlib import patches
from local_pathfinding.msg import latlon
from geopy.distance import distance

# Constants
MAX_PROJECT_OBSTACLE_TIME_HOURS = 3  # Maximum obstacle can be projected (dist btwn current and projected positions)
OBSTACLE_EXTEND_TIME_HOURS = 1       # Amount obstacles are extended forward (how long the shape is) TODO: max/min
WEDGE_EXPAND_ANGLE_DEGREES = 10.0
AIS_BOAT_RADIUS_KM = 0.2
AIS_BOAT_LENGTH_KM = 1
AIS_BOAT_CIRCLE_SPACING_KM = AIS_BOAT_RADIUS_KM * 1.5  # Distance between circles that make up an AIS boat
CURRENT_BOAT_COLOR = "red"
PROJECTED_BOAT_COLOR = "blue"


def getProjectedPosition(aisShip, sailbotPosition, sailbotSpeedKmph, referenceLatlon):
    """ Calculate position where we project the boat to be if the sailbot moves directly towards it

    Args:
       aisShip (AISShip): State of the ais ship
       sailbotPosition (local_pathfinding.msg._latlon.latlon): Position of sailbot
       sailbotSpeedKmph (float): Speed of sailbot in kmph
       referenceLatlon (local_pathfinding.msg._latlon.latlon): Position of the reference point that will be at (0,0)

    Returns:
       [x, y] projected position of the ais ship
    """
    # Get positions of ais boat and sailbot
    aisX, aisY = utils.latlonToXY(latlon(aisShip.lat, aisShip.lon), referenceLatlon)
    sailbotX, sailbotY = utils.latlonToXY(latlon(sailbotPosition.lat, sailbotPosition.lon), referenceLatlon)

    # Project ais boat's velocity vector onto the line between the boats.
    # Will be +ve if moving towards sailbot, -ve if moving away form sailbot
    angleBoatToSailbotDegrees = math.degrees(math.atan2(sailbotY - aisY, sailbotX - aisX))
    boatSpeedInDirToSailbotKmph = (aisShip.speedKmph *
                                   math.cos(math.radians(aisShip.headingDegrees - angleBoatToSailbotDegrees)))

    # Calculate MINIMUM time it would take for the boats to collide, but have an upper bound
    distanceBetweenBoatsKm = distance((aisShip.lat, aisShip.lon), (sailbotPosition.lat, sailbotPosition.lon)).kilometers
    if sailbotSpeedKmph + boatSpeedInDirToSailbotKmph == 0:
        smallestTimeToLocHours = MAX_PROJECT_OBSTACLE_TIME_HOURS
    else:
        smallestTimeToLocHours = distanceBetweenBoatsKm / (sailbotSpeedKmph + boatSpeedInDirToSailbotKmph)
        smallestTimeToLocHours = smallestTimeToLocHours if smallestTimeToLocHours < MAX_PROJECT_OBSTACLE_TIME_HOURS
                                                        else MAX_PROJECT_OBSTACLE_TIME_HOURS
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


class ObstacleInterface():
    def __init__(self, aisShip, sailbotPosition, sailbotSpeedKmph, referenceLatlon):
        """ Initialize obstacle """
        pass

    def __str__(self):
        """ String representation of obstacle """
        pass

    def _extendObstacle(self, aisShip, sailbotPosition, sailbotSpeedKmph, referenceLatlon):
        """ Extends obstacle based on speed and heading """
        pass

    def addPatch(self, axes, color):
        """ Add matplotlib.patches to axes with the given color """
        pass

    def isValid(self, xy):
        """ Checks validity of xy"""
        pass

    def clearance(self, xy):
        """ Return distance from obstacle to xy"""
        pass



class EllipseObstacle(ObstacleInterface):
    def __init__(self, aisShip, sailbotPosition, sailbotSpeedKmph, referenceLatlon):
        # Store member variables
        self.aisShip = aisShip
        self.sailbotPosition = sailbotPosition
        self.sailbotSpeedKmph = sailbotSpeedKmph
        self.referenceLatlon = referenceLatlon

        # Calculate current and projected position
        self.currentX, self.currentY = utils.latlonToXY(latlon(aisShip.lat, aisShip.lon), referenceLatlon)
        self.projectedX, self.projectedY = getProjectedPosition(aisShip, sailbotPosition,

        # Create ellipses
        self.currentEllipse = self.createEllipse(self.currentX, self.currentY)
        self.projectedEllipse = self.createEllipse(self.projectedX, self.projectedY)

    def __str__(self):
        return "Current ellipse: {}. Projecte ellipse: {}".format(self.currentEllipse, self.projectedEllipse)

    def createEllipse(self, aisX, aisY):
        # Calculate width and height of ellipse
        extendBoatLengthKm = self.aisShip.speedKmph * OBSTACLE_EXTEND_TIME_HOURS
        width = extendBoatLengthKm if extendBoatLengthKm > AIS_BOAT_RADIUS_KM else AIS_BOAT_RADIUS_KM
        height = AIS_BOAT_RADIUS_KM
        angle = self.aisShip.headingDegrees

        # Calculate xy center of ellipse, which is has its tip at the actual (aisX, aisY)
        xy = [self.projectedX + extendBoatLengthKm * math.cos(math.radians(angle)) * 0.5,
              self.projectedY + extendBoatLengthKm * math.sin(math.radians(angle)) * 0.5]
        x, y = xy
        return Ellipse(x, y, height, width, angle)

    def isValid(self, xy):
        return self.projectedEllipse.isValid(xy)

    def addPatch(self, axes):
        self.projectedEllipse.addPatch(axes, color=PROJECTED_BOAT_COLOR)
        self.currentEllipse.addPatch(axes, color=CURRENT_BOAT_COLOR)

    def clearance(self, xy):
        return self.projectedEllipse.clearance(xy)



class WedgeObstacle(ObstacleInterface):
    def __init__(self, aisShip, sailbotPosition, sailbotSpeedKmph, referenceLatlon):
        # Store member variables
        self.aisShip = aisShip
        self.sailbotPosition = sailbotPosition
        self.sailbotSpeedKmph = sailbotSpeedKmph
        self.referenceLatlon = referenceLatlon

        # Calculate current and projected position
        self.currentX, self.currentY = utils.latlonToXY(latlon(aisShip.lat, aisShip.lon), referenceLatlon)
        self.projectedX, self.projectedY = getProjectedPosition(aisShip, sailbotPosition,

        # Create ellipses
        self.currentWedge = self.createWedge(self.currentX, self.currentY)
        self.projectedWedge = self.createWedge(self.projectedX, self.projectedY)

    def __str__(self):
        return str((self.x, self.y, self.radius, self.theta1, self.theta2))

    def createWedge(self, aisX, aisY):
        theta1 = self.aisShip.headingDegrees - WEDGE_EXPAND_ANGLE_DEGREES / 2.0
        theta2 = self.aisShip.headingDegrees + WEDGE_EXPAND_ANGLE_DEGREES / 2.0

        # Ensure theta1 >= 0 and theta2 >= theta1
        if theta1 < 0:
            theta1 += 360
        while theta2 < theta1:
            theta2 += 360

        # Defines how long the wedge should be
        extendBoatLengthKm = aisShip.speedKmph * OBSTACLE_EXTEND_TIME_HOURS
        radius = extendBoatLengthKm if extendBoatLengthKm > AIS_BOAT_RADIUS_KM else AIS_BOAT_RADIUS_KM

        return Wedge(aisX, aisY, radius, theta1, theta2)

    def addPatch(self, axes):
        self.projectedWedge.addPatch(axes, color=PROJECTED_BOAT_COLOR)
        self.currentWedge.addPatch(axes, color=CURRENT_BOAT_COLOR)

    def isValid(self, xy):
        self.projectedWedge.isValid(xy)

    def clearance(self, xy):
        return self.projectedWedge.clearance(xy)


class CirclesObstacle(ObstacleInterface):
    def __init__(self, aisShip, sailbotPosition, sailbotSpeedKmph, referenceLatlon):
        # Store member variables
        self.aisShip = aisShip
        self.sailbotPosition = sailbotPosition
        self.sailbotSpeedKmph = sailbotSpeedKmph
        self.referenceLatlon = referenceLatlon

        # Calculate current and projected position
        self.currentX, self.currentY = utils.latlonToXY(latlon(aisShip.lat, aisShip.lon), referenceLatlon)
        self.projectedX, self.projectedY = getProjectedPosition(aisShip, sailbotPosition,

        # Create ellipses
        self.currentCircles = self.createCircles(self.currentX, self.currentY)
        self.projectedCircles = self.createCircles(self.projectedX, self.projectedY)

    def isValid(self, xy):
        for circle in self.projectedCircles:
            if not circle.isValid(xy):
                return False
        return True

    def createCircles(self, aisX, aisY):
        circles = []

        # Calculate length to extend boat
        extendBoatLengthKm = self.aisShip.speedKmph * OBSTACLE_EXTEND_TIME_HOURS

        # Boat not moving
        if extendBoatLengthKm == 0:
            circles.append(Circle(aisX, aisY, AIS_BOAT_RADIUS_KM))

        # Boat moving vertically
        if self.aisShip.headingDegrees == 90 or self.aisShip.headingDegrees == 270:
            if self.aisShip.headingDegrees == 90:
                endY = aisY + extendBoatLengthKm
                yRange = np.arange(aisY, endY, AIS_BOAT_CIRCLE_SPACING_KM)
            if self.aisShip.headingDegrees == 270:
                endY = aisY - extendBoatLengthKm
                yRange = np.arange(endY, aisY, AIS_BOAT_CIRCLE_SPACING_KM)

            for y in yRange:
                # Multiplier to increase size of circles showing where the boat will be in the future in range [1, 2]
                multiplier = 1 + abs(float(y - aisY) / (endY - aisY))
                circles.append(Circle(aisX, y, AIS_BOAT_RADIUS_KM * multiplier))

        # All other cases
        else:
            isHeadingWest = self.aisShip.headingDegrees < 270 and self.aisShip.headingDegrees > 90
            slope = math.tan(math.radians(self.aisShip.headingDegrees))
            dx = AIS_BOAT_CIRCLE_SPACING_KM / math.sqrt(1 + slope**2)

            if aisX > 0:
                b = aisY + slope * -math.fabs(aisX)
            else:
                b = aisY + slope * math.fabs(aisX)
            xDistTravelled = math.fabs(extendBoatLengthKm * math.cos(math.radians(self.aisShip.headingDegrees)))
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
                circles.append(Circle(x, y(x), AIS_BOAT_RADIUS_KM * multiplier))

    def addPatch(self, axes):
        for circle in self.projectedCircles:
            circle.addPatch(axes, color=PROJECTED_BOAT_COLOR)
        for circle in self.currentCircles:
            circle.addPatch(axes, color=CURRENT_BOAT_COLOR)

    def clearance(self, xy):
        # TODO: Make this clearance better
        return (self.projectedCircles[0].x - xy[0])**2 + (self.projectedCircles[0].y - xy[1])**2



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


class ShapeInterface():
    def __str__(self):
        pass

    def isValid(self, xy):
        pass

    def addPatch(self, axes, color):
        pass

    def clearance(self, xy):
        pass

class Circle(ShapeInterface):
    """ Helper class for Circles representation"""
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

    def __str__(self):
        return str((self.x, self.y, self.radius))

    def isValid(self, xy):
        x, y = xy
        distance = math.sqrt(pow(x - self.x, 2) + pow(y - self.y, 2)
        return distance > self.radius

    def addPatch(self, axes, color):
        circlePatch = plt.Circle((self.x, self.y), radius=self.radius)
        circlePatch.set_color(color)
        axes.add_patch(circlePatch)

    def clearance(self, xy):
        x, y = xy
        distance = math.sqrt(pow(x - self.x, 2) + pow(y - self.y, 2)
        return distance if distance > 0 else 0

class Ellipse(ShapeInterface):
    def __init__(self, x, y, height, width, angleDegrees):
        self.x = x
        self.y = y
        self.height = height
        self.width = width
        self.angleDegrees = angleDegrees

    def __str__(self):
        return str((self.x, self.y, self.height, self.width, self.angleDegrees))

    def isValid(self, xy):
        x = xy[0] - self.x
        y = xy[1] - self.y
        x_ = math.cos(math.radians(self.angleDegrees)) * x + math.sin(math.radians(self.angleDegrees)) * y
        y_ = -math.sin(math.radians(self.angleDegrees)) * x + math.cos(math.radians(self.angleDegrees)) * y
        distance_center_to_boat = math.sqrt(x_ ** 2 + y_ ** 2)
        angleCenterToBoatDegrees = math.degrees(math.atan2(y_, x_))
        angleCenterToBoatDegrees = (angleCenterToBoatDegrees + 360) % 360

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
        rotation_col1 = np.array([math.cos(math.radians(self.angleDegrees)), math.sin(math.radians(self.angleDegrees))])
        rotation_col2 = np.array([-math.sin(math.radians(self.angleDegrees)), math.cos(math.radians(self.angleDegrees))])
        edge_pt = init_pt + a * math.cos(t) * rotation_col1 + b * math.sin(t) * rotation_col2
        return edge_pt

    def addPatch(self, axes, color):
        patch = patches.Ellipse((self.x, self.y), self.width, self.height, self.angleDegrees)
        patch.set_color(color)
        axes.add_patch(patch)

    def clearance(self, xy):
        # TODO: Make this clearance distance from ellipse edge, not center
        return (self.x - xy[0])**2 + (self.y - xy[1])**2

class Wedge(ShapeInterface):
    def __init__(self, x, y, radius, theta1, theta2):
        self.x = x
        self.y = y
        self.radius = radius
        self.theta1 = theta1
        self.theta2 = theta2

    def __str__(self):
        return str((self.x, self.y, self.radius, self.theta1, self.theta2))

    def isValid(self, xy):
        angle = math.degrees(math.atan2(xy[1] - self.y, xy[0] - self.x))

        # Ensure that angle >= theta1
        while angle < self.theta1:
            angle += 360

        distance = math.sqrt((xy[1] - self.y) ** 2 + (xy[0] - self.x) ** 2)
        return not (angle > self.theta1 and angle < self.theta2 and distance <= self.radius)

    def addPatch(self, axes, color):
        patch = patches.Wedge((self.x, self.y), self.radius, self.theta1, self.theta2)
        patch.set_color(color)
        axes.add_patch(patch)

    def clearance(self, xy):
        # TODO: Make this clearance distance from wedge edge, not tip
        return (self.x - xy[0])**2 + (self.y - xy[1])**2
