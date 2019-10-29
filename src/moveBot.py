#!/usr/bin/env python

import rospy
import math 
import pickle
import numpy as np
from geometry_msgs.msg import Twist

class Robot():

    def __init__(self):

        # Give the node the name 'moveBot'
        rospy.init_node('moveBot', anonymous=False)

        # Set rospy to execute a shutdown function when exiting
        rospy.on_shutdown(self.shutdown)

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.rate = 50
        self.r = rospy.Rate(self.rate)

    def translate(self, distance):

        vel = .2

        # Create a message object
        move_cmd = Twist()

        # Set the speed to be 0.2 m/s
        move_cmd.linear.x = vel if distance > 0 else -vel

        # Time taken by robot to travel given distance
        duration = abs(distance) / vel
        print(duration)

        # Keep publishing for that duration
        ticks = int(duration * self.rate)
        for _ in range(ticks):
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()

        # Stop the robot
        self.cmd_vel.publish(Twist())
        for _ in range(5):
            self.r.sleep()
        
    def rotate(self, angle):

        omega = .05

        # Create a message object
        move_cmd = Twist()

        # Set the angular speed
        move_cmd.angular.z = omega if angle > 0 else -omega

        # Time taken by robot to rotate for given angle
        duration = abs(angle) / omega

        # Keep publishing for that duration
        ticks = int(duration * self.rate)
        for _ in range(ticks):
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()

        # Stop the robot
        self.cmd_vel.publish(Twist())
        for _ in range(5):
            self.r.sleep()

    def shutdown(self):
        print ("Exiting..")
        self.cmd_vel.publish(Twist())

def dot(a,b):
    return a[0]*b[0]+a[1]*b[1]

def mag(a):
    return math.sqrt(a[0]**2+a[1]**2)

def sub(a,b):
    return [a[0]-b[0], a[1]-b[1]]

def run():

    print("loading data")
    lines = []
    with open('path.data', 'rb') as filehandle:
        lines = pickle.load(filehandle)
        
    print("init robot")
    robot = Robot()

    print("moving")
    theta = 0
    for [pos,dest] in lines:
        pos =  [float( pos[0]),float( pos[1])]
        dest = [float(dest[0]),float(dest[1])]
        dif = sub(dest,pos)
        print(pos, "->", dest)

        theta_vec = [math.cos(theta),math.sin(theta)]
        delta = math.acos(dot(theta_vec,dif)/mag(dif))
        if math.atan2(dif[1],dif[0]) < theta:
            delta = -delta

        print("oldtheta", theta)
        theta = math.atan2(dif[1], dif[0])
        print("moving ", str(delta), " degrees")
        print("theta", theta)
        print("dif", dif)
        robot.rotate(delta)

        dist = mag(dif)/100
        print("moving ", str(dist), " units")
        robot.translate(dist)

if __name__ == "__main__":
        run()
