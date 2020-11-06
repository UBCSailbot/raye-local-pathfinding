#!/usr/bin/env python

import rospy
from local_pathfinding.msg import AISMsg, GPS

class SailbotGPSData:
    def StoreSailbotGPS():
        while not rospy.is_shutdown():   
            rospy.loginfo("test")

if __name__ == '__main__':
    SailbotGPSData.StoreSailbotGPS()


'''
class SailbotGPSData:
    def callback(data):
        bool flag = false
        if flag = false 
        {
            initialLocationLat = data.lat
            initialLocationLong = data.long
            flag = true

        }
        #get data from gps subscriber
        locationLat = data.lat
        locationLong = data.long

        #find displacement
        displacementLat = locationLat - initialLocationLat
        displacementLong = locationLong - initialLocationLong

        #log displacement
        rospy.loginfo("Latitude displacement: {}".format(displacementLat))

    def StoreSailbotGPS():
        #create a node called GPS Listener
        rospy.init_node('GPSListener', anonymous=True)

        #Subscribe to the GPS data, invoke the callback function every time the data is changed
        rospy.Subscriber("StoreSailbotGPS", GPS, callback)

    def StoreSailbotGPS1():
        rospy.init_node('GPSListener', anonymous=True)
        rospy.loginfo("test")
    
if name == '__main__':
    while not rospy.is_shutdown():
        StoreSailbotGPS1()
'''
