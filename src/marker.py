#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import rospy
from geometry_msgs.msg import PointStamped, Point 
from visualization_msgs.msg import Marker, MarkerArray 


class myMarker(object):


    def __init__(self):
        rospy.init_node('vGraph')
        self.publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size = 100)
        self.markerArr = MarkerArray()
        self.currentID = 0
        self.r = rospy.Rate(30)
        rospy.sleep(5)
        rospy.loginfo("this is some example stuff")

    
    def marker(self, vertices):

        #basic marker stuff
        line = Marker()
        line.header.frame_id = 'map'
        line.ns = "vGraph"
        line.type = line.LINE_STRIP
        line.action = line.ADD
        line.id = self.currentID #increment each time when marker() gets called
        self.currentID += 1

        #more color, scale, point
        line.color.a = 1.0
        line.color.r = 5.0
        line.scale.x = 0.02
        line.scale.y = 0.02
        line.scale.z = 0.02

        #create points and do line.points.append(point)
        print("drawing line: " + str(vertices))
        for vertex in vertices:
            point = Point()
            point.x = vertex[0]/100.0
            point.y = vertex[1]/100.0
            line.points.append(point) #add points to line to create a connected shape

            #append line to markerarray and publish it to rviz
            self.markerArr.markers.append(line)

            
            self.publisher.publish(self.markerArr) #takes an array of list
            self.r.sleep()
