import math
import numpy as np
import matplotlib.pyplot as plt
import utilities as utils
from matplotlib import patches
from sailbot_msg.msg import latlon
from geopy.distance import distance
from shapely.geometry import Point as ShapelyPoint
from shapely.geometry.polygon import Polygon as ShapelyPolygon
import os
import rospy

# Obstacle dimension constants
MAX_PROJECT_OBSTACLE_TIME_HOURS = 3  # Maximum obstacle can be projected (dist btwn current and projected positions)
OBSTACLE_EXTEND_TIME_HOURS = 0.5     # Amount obstacles are extended forward (how long the shape is) TODO: max/min
MAX_OBSTACLE_EXTENSION_KM = 2        # Maximum amount obstacles are extended forward (how long the shape is)
MAX_OBSTACLES = 50                   # Only use up to 50 closest obstacles
WEDGE_EXPAND_ANGLE_DEGREES = 10.0
AIS_BOAT_RADIUS_KM = 0.2
AIS_BOAT_LENGTH_KM = 1
AIS_BOAT_CIRCLE_SPACING_KM = AIS_BOAT_RADIUS_KM * 1.5  # Distance between circles that make up an AIS boat
SAILBOT_MAX_SPEED = 14.4  # kmph
SAILBOT_SPEED_BUFFER = 1.1

# Obstacle visualization constants
CURRENT_BOAT_COLOR = "red"
PROJECTED_BOAT_COLOR = "blue"
CURRENT_BOAT_TRANSPARENCY_ALPHA = 0.1  # Current position is more transparent

# Absolute path to the local-pathfinding directory from the absolute path to this file
LOCAL_DIR_ABS_PATH = os.path.abspath(__file__)[:os.path.abspath(__file__).find('python/')]


def getObstacles(state, referenceLatlon):
    '''Creates a list of obstacles in xy coordinates. Uses the obstacle type from the rosparam "obstacle_type"

    Args:
       state (BoatState): State of the sailbot
       referenceLatlon (sailbot_msg.msg._latlon.latlon): Position of the reference point that will be at (0,0)

    Returns:
       list of obstacles that implement the obstacle interface
    '''
    ships, position = state.AISData.ships, state.position
    obstacle_type = rospy.get_param('obstacle_type', 'hybrid_circle')
    obstacles = []
    if obstacle_type == "ellipse":
        for ship in ships:
            obstacles.append(EllipseObstacle(ship, position, referenceLatlon))
    elif obstacle_type == "wedge":
        for ship in ships:
            obstacles.append(WedgeObstacle(ship, position, referenceLatlon))
    elif obstacle_type == "circles":
        for ship in ships:
            obstacles.append(CirclesObstacle(ship, position, referenceLatlon))
    elif obstacle_type == "hybrid_ellipse":
        for ship in ships:
            obstacles.append(HybridEllipseObstacle(ship, position, referenceLatlon))
    elif obstacle_type == "hybrid_circle":
        for ship in ships:
            obstacles.append(HybridCircleObstacle(ship, position, referenceLatlon))

    # Use up to MAX_OBSTACLES closest obstacles
    if len(obstacles) > MAX_OBSTACLES:
        positionXY = utils.latlonToXY(position, referenceLatlon)
        obstacles = sorted(obstacles, key=lambda x: x.clearance(positionXY))[:MAX_OBSTACLES]

    # create a land mass obstacle if land_latlons is not empty
    # path relative to local-pathfinding directory
    land_mass_file = rospy.get_param('land_mass_file', default='')
    land_mass_file_abs_path = os.path.join(LOCAL_DIR_ABS_PATH, land_mass_file)
    if os.path.isfile(land_mass_file_abs_path):
        obstacles.append(GeneralPolygon(land_mass_file_abs_path, referenceLatlon))
    elif land_mass_file:
        rospy.logfatal('Land mass file at {} does not exist'.format(land_mass_file_abs_path))

    return obstacles


''' ObstacleInterface and obstacle implementations, which are used to represent AIS ships as obstacles '''


class ObstacleInterface():
    def __init__(self, aisShip, sailbotPosition, referenceLatlon):
        """ Initialize obstacle """
        # Store member variables
        self.aisShip = aisShip
        self.sailbotPosition = sailbotPosition
        self.referenceLatlon = referenceLatlon

        # Calculate current and projected position
        self.currentX, self.currentY = utils.latlonToXY(latlon(aisShip.lat, aisShip.lon), referenceLatlon)
        self.projectedX, self.projectedY = getProjectedPosition(aisShip, sailbotPosition, referenceLatlon)

    def __str__(self):
        """ String representation of obstacle """
        pass

    def addPatch(self, axes):
        """ Add matplotlib.patches to axes """
        pass

    def isValid(self, xy):
        """ Checks validity of xy"""
        pass

    def clearance(self, xy):
        """ Return distance from obstacle to xy"""
        pass


class EllipseObstacle(ObstacleInterface):
    def __init__(self, aisShip, sailbotPosition, referenceLatlon):
        ObstacleInterface.__init__(self, aisShip, sailbotPosition, referenceLatlon)

        # Create ellipses
        self.currentEllipse = self._createEllipse(self.currentX, self.currentY)
        self.projectedEllipse = self._createEllipse(self.projectedX, self.projectedY)

    def __str__(self):
        return "Current ellipse: {}. Projecte ellipse: {}".format(self.currentEllipse, self.projectedEllipse)

    def _createEllipse(self, aisX, aisY):
        # Calculate width and height of ellipse
        extendBoatLengthKm = min([self.aisShip.speedKmph * OBSTACLE_EXTEND_TIME_HOURS, MAX_OBSTACLE_EXTENSION_KM])
        width = max(extendBoatLengthKm, AIS_BOAT_RADIUS_KM)  # Ensure greater than minimum length
        height = AIS_BOAT_RADIUS_KM
        angle = self.aisShip.headingDegrees

        # Calculate xy center of ellipse, which is has its tip at the actual (aisX, aisY)
        xy = [aisX + extendBoatLengthKm * math.cos(math.radians(angle)) * 0.5,
              aisY + extendBoatLengthKm * math.sin(math.radians(angle)) * 0.5]
        x, y = xy
        return Ellipse(x, y, height, width, angle)

    def isValid(self, xy):
        return self.projectedEllipse.isValid(xy)

    def addPatch(self, axes):
        self.projectedEllipse.addPatch(axes, color=PROJECTED_BOAT_COLOR)
        self.currentEllipse.addPatch(axes, color=CURRENT_BOAT_COLOR, alpha=CURRENT_BOAT_TRANSPARENCY_ALPHA)

    def clearance(self, xy):
        return self.projectedEllipse.clearance(xy)


class WedgeObstacle(ObstacleInterface):
    def __init__(self, aisShip, sailbotPosition, referenceLatlon):
        ObstacleInterface.__init__(self, aisShip, sailbotPosition, referenceLatlon)

        # Create wedges
        self.currentWedge = self._createWedge(self.currentX, self.currentY)
        self.projectedWedge = self._createWedge(self.projectedX, self.projectedY)

    def __str__(self):
        return str((self.x, self.y, self.radius, self.theta1, self.theta2))

    def _createWedge(self, aisX, aisY):
        theta1 = self.aisShip.headingDegrees - WEDGE_EXPAND_ANGLE_DEGREES / 2.0
        theta2 = self.aisShip.headingDegrees + WEDGE_EXPAND_ANGLE_DEGREES / 2.0

        # Ensure theta1 >= 0 and theta2 >= theta1
        if theta1 < 0:
            theta1 += 360
        while theta2 < theta1:
            theta2 += 360

        # Defines how long the wedge should be, ensure greater than minimum length
        extendBoatLengthKm = min([self.aisShip.speedKmph * OBSTACLE_EXTEND_TIME_HOURS, MAX_OBSTACLE_EXTENSION_KM])
        radius = max(extendBoatLengthKm, AIS_BOAT_RADIUS_KM)

        return Wedge(aisX, aisY, radius, theta1, theta2)

    def addPatch(self, axes):
        self.projectedWedge.addPatch(axes, color=PROJECTED_BOAT_COLOR)
        self.currentWedge.addPatch(axes, color=CURRENT_BOAT_COLOR, alpha=CURRENT_BOAT_TRANSPARENCY_ALPHA)

    def isValid(self, xy):
        return self.projectedWedge.isValid(xy)

    def clearance(self, xy):
        return self.projectedWedge.clearance(xy)


class CirclesObstacle(ObstacleInterface):
    def __init__(self, aisShip, sailbotPosition, referenceLatlon):
        ObstacleInterface.__init__(self, aisShip, sailbotPosition, referenceLatlon)

        # Create ellipses
        self.currentCircles = self._createCircles(self.currentX, self.currentY)
        self.projectedCircles = self._createCircles(self.projectedX, self.projectedY)

    def isValid(self, xy):
        for circle in self.projectedCircles:
            if not circle.isValid(xy):
                return False
        return True

    def _createCircles(self, aisX, aisY):
        circles = []

        # Calculate length to extend boat
        extendBoatLengthKm = min([self.aisShip.speedKmph * OBSTACLE_EXTEND_TIME_HOURS, MAX_OBSTACLE_EXTENSION_KM])

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
        return circles

    def addPatch(self, axes):
        for circle in self.projectedCircles:
            circle.addPatch(axes, color=PROJECTED_BOAT_COLOR)
        for circle in self.currentCircles:
            circle.addPatch(axes, color=CURRENT_BOAT_COLOR, alpha=CURRENT_BOAT_TRANSPARENCY_ALPHA)

    def clearance(self, xy):
        return (self.projectedCircles[0].x - xy[0])**2 + (self.projectedCircles[0].y - xy[1])**2


class HybridEllipseObstacle(ObstacleInterface):
    def __init__(self, aisShip, sailbotPosition, referenceLatlon):
        ObstacleInterface.__init__(self, aisShip, sailbotPosition, referenceLatlon)

        self.wedgeObstacle = WedgeObstacle(aisShip, sailbotPosition, referenceLatlon)
        self.ellipseObstacle = EllipseObstacle(aisShip, sailbotPosition, referenceLatlon)

    def __str__(self):
        return str(self.wedgeObstacle) + str(self.ellipseObstacle)

    def addPatch(self, axes):
        self.wedgeObstacle.addPatch(axes)
        self.ellipseObstacle.addPatch(axes)

    def isValid(self, xy):
        return (self.wedgeObstacle.isValid(xy) and self.ellipseObstacle.isValid(xy))

    def clearance(self, xy):
        return self.wedgeObstacle.clearance(xy)


class HybridCircleObstacle(ObstacleInterface):
    def __init__(self, aisShip, sailbotPosition, referenceLatlon):
        ObstacleInterface.__init__(self, aisShip, sailbotPosition, referenceLatlon)

        self.currentCircle = Circle(self.currentX, self.currentY, AIS_BOAT_RADIUS_KM)
        self.projectedCircle = Circle(self.projectedX, self.projectedY, AIS_BOAT_RADIUS_KM)
        self.wedgeObstacle = WedgeObstacle(aisShip, sailbotPosition, referenceLatlon)

    def __str__(self):
        return str(self.currentCircle) + str(self.projectedCircle) + str(self.wedgeObstacle)

    def addPatch(self, axes):
        self.projectedCircle.addPatch(axes, color=PROJECTED_BOAT_COLOR)
        self.currentCircle.addPatch(axes, color=CURRENT_BOAT_COLOR, alpha=CURRENT_BOAT_TRANSPARENCY_ALPHA)
        self.wedgeObstacle.addPatch(axes)

    def isValid(self, xy):
        return (self.wedgeObstacle.isValid(xy) and self.projectedCircle.isValid(xy))

    def clearance(self, xy):
        return self.wedgeObstacle.clearance(xy)


class GeneralPolygon(ObstacleInterface):
    def __init__(self, land_mass_file, referenceLatlon):
        """ Initialize obstacle

        Args:
            land_mass_file (str): path to a file with the bounding latlons of the land mass to visualize
                - Latlons in the format "lat,lon\n"
                - Latlons next to each other in the list are neighbors in the land mass
            referenceLatlon (sailbot_msg.msg._latlon.latlon): Position of the reference point that will be at (0,0)
        """
        with open(land_mass_file, 'r') as f:
            lines = f.readlines()

        self.land_latlons = [[float(coord) for coord in latlon_str.split(',')] for latlon_str in lines]
        self.land_xys = [utils.latlonToXY(latlon(land_latlon_list[0], land_latlon_list[1]), referenceLatlon)
                         for land_latlon_list in self.land_latlons]
        self.shapely_polygon = ShapelyPolygon(self.land_xys)

    def __str__(self):
        return str(self.land_latlons)

    def addPatch(self, axes, color='g', alpha=1.0):
        polygonPatch = patches.Polygon(self.land_xys)
        polygonPatch.set_color(color)
        polygonPatch.set_alpha(alpha)
        axes.add_patch(polygonPatch)

    def isValid(self, xy):
        return not self.shapely_polygon.contains(ShapelyPoint(xy[0], xy[1]))

    def clearance(self, xy):
        return self.shapely_polygon.exterior.distance(ShapelyPoint(xy[0], xy[1]))


def getProjectedPosition(aisShip, sailbotPosition, referenceLatlon):
    """ Calculate position where we project the boat to be if the sailbot moves directly towards it

    Args:
       aisShip (AISShip): State of the ais ship
       sailbotPosition (sailbot_msg.msg._latlon.latlon): Position of sailbot
       sailbotSpeedKmph (float): Speed of sailbot in kmph
       referenceLatlon (sailbot_msg.msg._latlon.latlon): Position of the reference point that will be at (0,0)

    Returns:
       [x, y] projected position of the ais ship
    """
    # calculate projection as if sailbot was moving slightly faster than its maximum speed
    sailbotSpeedKmph = SAILBOT_MAX_SPEED * SAILBOT_SPEED_BUFFER

    # Get positions of ais boat and sailbot
    aisX, aisY = utils.latlonToXY(latlon(aisShip.lat, aisShip.lon), referenceLatlon)
    sailbotX, sailbotY = utils.latlonToXY(latlon(sailbotPosition.lat, sailbotPosition.lon), referenceLatlon)

    # Project ais boat's velocity vector onto the line between the boats.
    # Will be +ve if moving towards sailbot, -ve if moving away form sailbot
    angleBoatToSailbotDegrees = math.degrees(math.atan2(sailbotY - aisY, sailbotX - aisX))
    boatSpeedInDirToSailbotKmph = (aisShip.speedKmph *
                                   math.cos(math.radians(aisShip.headingDegrees - angleBoatToSailbotDegrees)))
    distanceBetweenBoatsKm = distance((aisShip.lat, aisShip.lon), (sailbotPosition.lat, sailbotPosition.lon)).kilometers
    distanceBetweenBoatsMaxRateOfChangeKmph = sailbotSpeedKmph + boatSpeedInDirToSailbotKmph

    # Calculate MINIMUM time it would take for the boats to collide
    # Sailbot can't catch up to other boat, project boat up to max threshold
    if distanceBetweenBoatsMaxRateOfChangeKmph <= 0:
        smallestTimeToLocHours = MAX_PROJECT_OBSTACLE_TIME_HOURS
    else:
        # Ensure projected time to collision stays below threshold (not project too far forwards)
        smallestTimeToLocHours = distanceBetweenBoatsKm / distanceBetweenBoatsMaxRateOfChangeKmph
        smallestTimeToLocHours = min(smallestTimeToLocHours, MAX_PROJECT_OBSTACLE_TIME_HOURS)

    minimumBoatDistanceTravelledKm = smallestTimeToLocHours * aisShip.speedKmph

    # Calculate projected position
    projectedX = aisX + minimumBoatDistanceTravelledKm * math.cos(math.radians(aisShip.headingDegrees))
    projectedY = aisY + minimumBoatDistanceTravelledKm * math.sin(math.radians(aisShip.headingDegrees))
    projectedPosition = [projectedX, projectedY]
    return projectedPosition


''' ShapeInterface and shape implementations, which are used in the ObstacleInterface '''


class ShapeInterface():
    def __str__(self):
        pass

    def isValid(self, xy):
        """ Checks validity of xy """
        pass

    def addPatch(self, axes, color, alpha=1.0):
        """ Add matplotlib.patches to axes with given color and alpha """
        pass

    def clearance(self, xy):
        """ Return distance from obstacle to xy """
        pass


class Circle(ShapeInterface):
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

    def __str__(self):
        return str((self.x, self.y, self.radius))

    def isValid(self, xy):
        x, y = xy
        distance = math.sqrt(pow(x - self.x, 2) + pow(y - self.y, 2))
        return distance > self.radius

    def addPatch(self, axes, color, alpha=1.0):
        circlePatch = plt.Circle((self.x, self.y), radius=self.radius)
        circlePatch.set_color(color)
        circlePatch.set_alpha(alpha)
        axes.add_patch(circlePatch)

    def clearance(self, xy):
        x, y = xy
        distance = math.sqrt(pow(x - self.x, 2) + pow(y - self.y, 2))

        # Ensure clearance >= 0
        return max(distance - self.radius, 0)


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
        isInsideEllipse = (distance_center_to_boat < distance_to_edge
                           or math.fabs(distance_to_edge - distance_center_to_boat) <= delta)
        return not isInsideEllipse

    def _ellipseFormula(self, t):
        init_pt = np.array([self.x, self.y])
        a = 0.5 * self.width
        b = 0.5 * self.height
        rotationCol1 = np.array([math.cos(math.radians(self.angleDegrees)), math.sin(math.radians(self.angleDegrees))])
        rotationCol2 = np.array([-math.sin(math.radians(self.angleDegrees)), math.cos(math.radians(self.angleDegrees))])
        edge_pt = init_pt + a * math.cos(t) * rotationCol1 + b * math.sin(t) * rotationCol2
        return edge_pt

    def addPatch(self, axes, color, alpha=1.0):
        patch = patches.Ellipse((self.x, self.y), self.width, self.height, self.angleDegrees)
        patch.set_color(color)
        patch.set_alpha(alpha)
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
        isInsideWedge = (angle > self.theta1 and angle < self.theta2 and distance <= self.radius)
        return not isInsideWedge

    def addPatch(self, axes, color, alpha=1.0):
        patch = patches.Wedge((self.x, self.y), self.radius, self.theta1, self.theta2)
        patch.set_color(color)
        patch.set_alpha(alpha)
        axes.add_patch(patch)

    def clearance(self, xy):
        # TODO: Make this clearance distance from wedge edge, not tip
        return (self.x - xy[0])**2 + (self.y - xy[1])**2
