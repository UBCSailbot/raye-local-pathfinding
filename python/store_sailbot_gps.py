#!/usr/bin/env python

import rospy
import numpy as np
import os
from geopy.distance import distance
import matplotlib.pyplot as plt
from sailbot_msg.msg import AISMsg, GPS
from datetime import datetime
from datetime import date

UPDATE_TIME_SECONDS = 1.0

dateStr = date.today().strftime("%b-%d-%Y")
timeStr = datetime.now().strftime('%H-%M-%S')
ABS_PATH_TO_THIS_FILE_DIR = os.path.dirname(os.path.abspath(__file__))
ABS_PATH_TO_OUTPUT_DIR = os.path.join(ABS_PATH_TO_THIS_FILE_DIR, "../output/{}_{}".format(dateStr, timeStr))
ABS_PATH_TO_OUTPUT_SAILBOT_GPS_DISPLACEMENT = os.path.join(ABS_PATH_TO_OUTPUT_DIR, "sailbot_gps_plot.png")

class SailbotGPSData:

    def __init__(self):
         #Subscribe to the GPS data, invoke the callback function every time the data is changed
        rospy.Subscriber('/GPS', GPS, self.GPS_callback)

        #Get initial values for Lat and Lon and simulation start time
        self.latInitial = rospy.wait_for_message('/GPS', GPS).lat
        self.lonInitial = rospy.wait_for_message('/GPS', GPS).lon

        #array to hold all the Lat and Lon values
        self.latArray = list()
        self.lonArray = list()

        #place initial values into the Lat and Lon array
        self.latArray.append(self.latInitial)
        self.lonArray.append(self.lonInitial)

        #wait some time for the other processes to start up
        rospy.sleep(5)

    def GPS_callback(self, data):
        #Update the values for Lat and Lon
        self.lat = data.lat
        self.lon = data.lon

    def Find_Distance(self):
        return distance((self.latInitial, self.lonInitial), (self.lat, self.lon)).km

    def StoreSailbotGPS(self):
        #sleep for 1 second, so that lat and lon values are updated every second
        rospy.sleep(UPDATE_TIME_SECONDS)

        #add the new values for the lat and lon to the array
        self.latArray.append(self.lat)
        self.lonArray.append(self.lon)

        #using geopy, return distance from start position in km and log to constole
        distance = str(self.Find_Distance())
        rospy.loginfo("Total displacement from start (km): {}".format(distance))

        plt.plot(self.lonArray, self.latArray, 'ro')
        plt.title('displacement in km:' + distance)
        plt.grid()
        plt.xlabel('Longitude')
        plt.ylabel('Latitude')
        plt.draw()
        plt.pause(0.0001)
        plt.clf()

    
if __name__ == '__main__':
    rospy.init_node('SailbotGPSData')
    myClass = SailbotGPSData()
    while not rospy.is_shutdown():
        myClass.StoreSailbotGPS()
    
    rospy.loginfo("hi")
    plt.savefig(ABS_PATH_TO_OUTPUT_SAILBOT_GPS_DISPLACEMENT)
    rospy.loginfo("Successfully saved cost breakdown plot to {}".format(ABS_PATH_TO_OUTPUT_SAILBOT_GPS_DISPLACEMENT))
        
