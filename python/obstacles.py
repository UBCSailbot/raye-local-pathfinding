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


class ObstacleInterface:
    def __init__(self, aisData, sailbotPosition, speedKmph, referenceLatlon):
        self.aisData = aisData
        self.sailbotPosition = sailbotPosition
        self.speedKmph = speedKmph
        self.referenceLatlon = referenceLatlon
        pass

    def __str__(self):
        pass

    def _extendObstacle(self, aisData, sailbotPosition, speedKmph, referenceLatlon):
        """ Extends obstacle based on speed and heading """
        pass

    def addPatch(self, axes):
        """ Return patch from matplotlib.patches """
        pass

    def isValid(self, xy):
        """ Checks validity of xy"""
        pass

    def clearance(self, xy):
        """ Return distance from obstacle to xy"""
        pass

    def shrink(self, shrinkFactor):
        """ Shrinks the obstacle by the shrink factor"""
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
        delta = 0.001
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

    def shrink(self, shrinkFactor):
        self.width /= shrinkFactor
        self.height /= shrinkFactor


class EllipseObstacle(ObstacleInterface, Ellipse):
    def __init__(self, aisData, sailbotPosition, speedKmph, referenceLatlon):
        ObstacleInterface.__init__(self, aisData, sailbotPosition, speedKmph, referenceLatlon)
        self._extendObstacle(self.aisData, self.sailbotPosition, self.speedKmph, self.referenceLatlon)

    def __str__(self):
        return Ellipse.__str__(self)

    def _extendObstacle(self, aisData, sailbotPosition, sailbotSpeedKmph, referenceLatlon):
        aisX, aisY = utils.latlonToXY(latlon(aisData.lat, aisData.lon), referenceLatlon)

        # Calculate length to extend boat
        distanceToBoatKm = distance((aisData.lat, aisData.lon), (sailbotPosition.lat, sailbotPosition.lon)).kilometers
        if sailbotSpeedKmph == 0 or distanceToBoatKm / sailbotSpeedKmph > OBSTACLE_MAX_TIME_TO_LOC_HOURS:
            timeToLocHours = OBSTACLE_MAX_TIME_TO_LOC_HOURS
        else:
            timeToLocHours = distanceToBoatKm / sailbotSpeedKmph
        extendBoatLengthKm = aisData.speedKmph * timeToLocHours

        if extendBoatLengthKm == 0:
            width = AIS_BOAT_RADIUS_KM
        else:
            width = extendBoatLengthKm
        height = AIS_BOAT_RADIUS_KM
        angle = aisData.headingDegrees
        xy = [
            aisX +
            extendBoatLengthKm *
            math.cos(
                math.radians(angle)) *
            0.5,
            aisY +
            extendBoatLengthKm *
            math.sin(
                math.radians(angle)) *
            0.5]
        self.x, self.y = xy[0], xy[1]
        self.width, self.height = width, height
        self.angle = angle

    def isValid(self, xy):
        return Ellipse.isValid(self, xy)

    def _ellipseFormula(self, t):
        return Ellipse._ellipseFormula(self, t)

    def addPatch(self, axes):
        Ellipse.addPatch(self, axes)

    def shrink(self, shrinkFactor):
        Ellipse.shrink(self, shrinkFactor)

    def clearance(self, xy):
        # TODO: Make this clearance better
        return (self.x - xy[0])**2 + (self.y - xy[1])**2


class Wedge(ObstacleInterface):
    def __init__(self, aisData, sailbotPosition, speedKmph, referenceLatlon):
        ObstacleInterface.__init__(self, aisData, sailbotPosition, speedKmph, referenceLatlon)
        self._extendObstacle(self.aisData, self.sailbotPosition, self.speedKmph, self.referenceLatlon)

    def __str__(self):
        return str((self.x, self.y, self.radius, self.theta1, self.theta2))

    def _extendObstacle(self, aisData, sailbotPosition, sailbotSpeedKmph, referenceLatlon):
        aisX, aisY = utils.latlonToXY(latlon(aisData.lat, aisData.lon), referenceLatlon)
        sailbotPositionX, sailbotPositionY = utils.latlonToXY(latlon(sailbotPosition.lat, sailbotPosition.lon),
                                                              referenceLatlon)

        theta1 = aisData.headingDegrees - WEDGE_EXPAND_ANGLE_DEGREES / 2.0
        theta2 = aisData.headingDegrees + WEDGE_EXPAND_ANGLE_DEGREES / 2.0

        # Ensure theta1 >= 0 and theta2 >= theta1
        if theta1 < 0:
            theta1 += 360
        while theta2 < theta1:
            theta2 += 360

        distanceToBoatKm = distance((aisData.lat, aisData.lon), (sailbotPosition.lat, sailbotPosition.lon)).kilometers
        angleBoatToSailbotDegrees = math.atan2(aisY - sailbotPositionY, aisX - sailbotPositionX)
        boatSpeedInDirToSailbotKmph = (aisData.speedKmph *
                                       math.cos(math.radians(aisData.headingDegrees - angleBoatToSailbotDegrees)))
        if sailbotSpeedKmph + boatSpeedInDirToSailbotKmph == 0:
            smallestTimeToLocHours = 0
        else:
            smallestTimeToLocHours = distanceToBoatKm / (sailbotSpeedKmph + boatSpeedInDirToSailbotKmph)
        distanceTravelledKm = smallestTimeToLocHours * aisData.speedKmph

        timeExtendLengthHours = 0.2
        radius = aisData.speedKmph * timeExtendLengthHours
        self.x = aisX + distanceTravelledKm * math.cos(math.radians(aisData.headingDegrees))
        self.y = aisY + distanceTravelledKm * math.sin(math.radians(aisData.headingDegrees))
        self.origx, self.origy = aisX, aisY
        self.radius = radius
        self.theta1, self.theta2 = theta1, theta2

    def addPatch(self, axes):
        axes.add_patch(patches.Wedge((self.x, self.y), self.radius, self.theta1, self.theta2))
        circ = patches.Wedge((self.origx, self.origy), self.radius, self.theta1, self.theta2)
        circ.set_color('red')
        axes.add_patch(circ)

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

    def shrink(self, shrinkFactor):
        self.radius /= shrinkFactor


class Circles(ObstacleInterface):
    def __init__(self, aisData, sailbotPosition, speedKmph, referenceLatlon):
        ObstacleInterface.__init__(self, aisData, sailbotPosition, speedKmph, referenceLatlon)
        self._extendObstacle(self.aisData, self.sailbotPosition, self.speedKmph, self.referenceLatlon)

    def isValid(self, xy):
        for obstacle in self.obstacles:
            if not obstacle.isValid(xy):
                return False
        return True

    def _extendObstacle(self, aisData, sailbotPosition, sailbotSpeedKmph, referenceLatlon):
        self.obstacles = []
        aisX, aisY = utils.latlonToXY(latlon(aisData.lat, aisData.lon), referenceLatlon)
        # Calculate length to extend boat
        distanceToBoatKm = distance((aisData.lat, aisData.lon), (sailbotPosition.lat, sailbotPosition.lon)).kilometers
        if sailbotSpeedKmph == 0 or distanceToBoatKm / sailbotSpeedKmph > OBSTACLE_MAX_TIME_TO_LOC_HOURS:
            timeToLocHours = OBSTACLE_MAX_TIME_TO_LOC_HOURS
        else:
            timeToLocHours = distanceToBoatKm / sailbotSpeedKmph
        extendBoatLengthKm = aisData.speedKmph * timeToLocHours

        if extendBoatLengthKm == 0:
            self.obstacles.append(Circle(aisX, aisY, AIS_BOAT_RADIUS_KM))

        if aisData.headingDegrees == 90 or aisData.headingDegrees == 270:
            if aisData.headingDegrees == 90:
                endY = aisY + extendBoatLengthKm
                yRange = np.arange(aisY, endY, AIS_BOAT_CIRCLE_SPACING_KM)
            if aisData.headingDegrees == 270:
                endY = aisY - extendBoatLengthKm
                yRange = np.arange(endY, aisY, AIS_BOAT_CIRCLE_SPACING_KM)
            for y in yRange:
                # Multiplier to increase size of circles showing where the boat will be in the future in range [1, 2]
                multiplier = 1 + abs(float(y - aisY) / (endY - aisY))
                self.obstacles.append(Circle(aisX, y, AIS_BOAT_RADIUS_KM * multiplier))
        else:
            isHeadingWest = aisData.headingDegrees < 270 and aisData.headingDegrees > 90
            slope = math.tan(math.radians(aisData.headingDegrees))
            dx = AIS_BOAT_CIRCLE_SPACING_KM / math.sqrt(1 + slope**2)

            if aisX > 0:
                b = aisY + slope * -math.fabs(aisX)
            else:
                b = aisY + slope * math.fabs(aisX)
            xDistTravelled = math.fabs(extendBoatLengthKm * math.cos(math.radians(aisData.headingDegrees)))
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

    def shrink(self, shrinkFactor):
        for obstacle in self.obstacles:
            obstacle.shrink(shrinkFactor)

    def addPatch(self, axes):
        for obstacle in self.obstacles:
            obstacle.addPatch(axes)

    def clearance(self, xy):
        # TODO: Make this clearance better
        return (self.obstacles[0].x - xy[0])**2 + (self.obstacles[0].y - xy[1])**2


class Circle():
    """ Helper class for Circles representation"""

    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

    def isValid(self, xy):
        x, y = xy
        if math.sqrt(pow(x - self.x, 2) + pow(y - self.y, 2)) - self.radius <= 0:
            return False
        return True

    def addPatch(self, axes):
        axes.add_patch(plt.Circle((self.x, self.y), radius=self.radius))

    def shrink(self, shrinkFactor):
        self.radius /= shrinkFactor


class HybridEllipse(ObstacleInterface):
    def __init__(self, aisData, sailbotPosition, speedKmph, referenceLatlon):
        ObstacleInterface.__init__(self, aisData, sailbotPosition, speedKmph, referenceLatlon)
        self._extendObstacle(self.aisData, self.sailbotPosition, self.speedKmph, self.referenceLatlon)
        self.xy = utils.latlonToXY(latlon(aisData.lat, aisData.lon), referenceLatlon)
        self.ellipse = Ellipse(self.xy[0], self.xy[1], AIS_BOAT_RADIUS_KM, AIS_BOAT_LENGTH_KM,
                               aisData.headingDegrees)

    def __str__(self):
        return str(self.ellipse) + str(self.wedge)

    def _extendObstacle(self, aisData, sailbotPosition, speedKmph, referenceLatlon):
        self.wedge = Wedge(aisData, sailbotPosition, speedKmph, referenceLatlon)

    def addPatch(self, axes):
        self.wedge.addPatch(axes)
        self.ellipse.addPatch(axes)

    def isValid(self, xy):
        return (self.wedge.isValid(xy) and self.ellipse.isValid(xy))

    def clearance(self, xy):
        return (self.xy[0] - xy[0])**2 + (self.xy[1] - xy[1])**2

    def shrink(self, shrinkFactor):
        self.ellipse.shrink(shrinkFactor)
        self.wedge.shrink(shrinkFactor)


class HybridCircle(ObstacleInterface):
    def __init__(self, aisData, sailbotPosition, speedKmph, referenceLatlon):
        ObstacleInterface.__init__(self, aisData, sailbotPosition, speedKmph, referenceLatlon)
        self._extendObstacle(self.aisData, self.sailbotPosition, self.speedKmph, self.referenceLatlon)
        self.xy = utils.latlonToXY(latlon(aisData.lat, aisData.lon), referenceLatlon)
        self.circle = Circle(self.xy[0], self.xy[1], AIS_BOAT_RADIUS_KM)

    def __str__(self):
        return str(self.circle) + str(self.wedge)

    def _extendObstacle(self, aisData, sailbotPosition, speedKmph, referenceLatlon):
        self.wedge = Wedge(aisData, sailbotPosition, speedKmph, referenceLatlon)

    def addPatch(self, axes):
        self.wedge.addPatch(axes)
        self.circle.addPatch(axes)

    def isValid(self, xy):
        return (self.wedge.isValid(xy) and self.circle.isValid(xy))

    def clearance(self, xy):
        return (self.xy[0] - xy[0])**2 + (self.xy[1] - xy[1])**2

    def shrink(self, shrinkFactor):
        self.circle.shrink(shrinkFactor)
        self.wedge.shrink(shrinkFactor)
