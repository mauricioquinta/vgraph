#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pi

class Robot():


    def __init__(self):

        # Give the node the name 'out_and_back'
        rospy.init_node('out_and_back', anonymous=False)

        # Set rospy to execute a shutdown function when exiting
        rospy.on_shutdown(self.shutdown)

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.rate = 50
        self.r = rospy.Rate(self.rate)

    def translate(self, distance):

        # Create a message object
        move_cmd = Twist()

        # Set the speed to be 0.2 m/s
        move_cmd.linear.x = 0.2 if distance > 0 else -0.2

        # Time taken by robot to travel given distance
        duration = abs(distance) / 0.2

        # Keep publishing for that duration
        ticks = int(duration * self.rate)
        for _ in range(ticks):
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()

        # Stop the robot
        self.cmd_vel.publish(Twist())
        
    def rotate(self, angle):
        
        angle = pi * angle / 180.

        # Create a message object
        move_cmd = Twist()

        # Set the angular speed
        move_cmd.angular.z = 1. if angle > 0 else -1.

        # Time taken by robot to rotate for given angle
        duration = abs(angle) / 1.

        # Keep publishing for that duration
        ticks = int(duration * self.rate)
        for _ in range(ticks):
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()

        # Stop the robot
        self.cmd_vel.publish(Twist())

    def shutdown(self):
        print ("Exiting..")
        self.cmd_vel.publish(Twist())

if __name__ == '__main__':

    robot = Robot()

    while True:

        try:
            case = raw_input("t for Translation, r for Rotation, q for Quitting: ")

            # Translation
            if case.lower() == 't':
                distance = float(raw_input("Enter distance in meters: "))
                robot.translate(distance)

            # Rotation
            elif case.lower() == 'r':
                rotation = float(raw_input("Enter rotation in degrees: "))
                robot.rotate(rotation)

            # Exit
            elif case.lower() == 'q':
                exit()

        except KeyboardInterrupt as e:
            robot.shutdown()
            break
