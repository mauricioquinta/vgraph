#!/usr/bin/env python

import numpy as np # I update the numpy to 1.15.1 using sudo pip install --upgrade numpy
import cv2
from scipy.spatial import ConvexHull, convex_hull_plot_2d
from marker import myMarker

#read return obstacle objects from file
def load_obstacles(object_path):
    obstacles = []
    obstacle = []
    with open(object_path) as f:
        numObstacles = int(f.readline())
        coordinates = int(f.readline())
        for i in range(coordinates):
            line = f.readline()
            obstacle.append(list(map(int, line.strip().split(' '))))
        for line in f:
            coordinates = list(map(int, line.strip().split(' ')))
            if len(coordinates) == 1:
                obstacles.append(obstacle)
                obstacle = []
            else:
                obstacle.append(coordinates)
    obstacles.append(obstacle)
    assert len(obstacles)==numObstacles, "number of obstacles does not match the first line"
        
    return obstacles

#return goal from file
def load_goal(goal_path):
    with open(goal_path) as f:
        line = f.readline()
        goal = list(map(int, line.strip().split(' ')))
    return goal


#make an expanded set of points with margin w
def extraPointsMod(obsticle):
        newObsticle = []
        for point in obsticle:
                x = point[0]  #---------x 
                y = point[1]  #---------y
                w = 18        #---------robot val assuming it's in the middle (width 36)
                #all points with a sqaure at every corner of orgininal vertex 
                newObsticle.append([x-w, y+w])
                newObsticle.append([x-w, y])
                newObsticle.append([x-w, y-w])
                
                newObsticle.append([x, y+w])
                newObsticle.append([x,y])
                newObsticle.append([x, y-w])
                
                newObsticle.append([x+w, y+w])
                newObsticle.append([x+w, y])
                newObsticle.append([x+w, y-w])

        return newObsticle

#make all non-colliding lines between the vertices of the hulls
def createLines(obsHulls, obsEdges):

    n = len(obsHulls)
    lines = []
    for i in range(n):
        hull1 = obsHulls[i]
        for a in hull1:
            for j in range(i+1,n):
                hull2 = obsHulls[j]
                for b in hull2:
                    elem = [a,b]
                    if not isCollision(a,b,obsEdges) :
                        lines.append(elem)
    return lines


#check if line [p1,p2] collides with objects defined by obsEdges
def isCollision(p1,p2, obsEdges):
    for edge in obsEdges:
        q1 = edge[0]
        q2 = edge[1]
        #checks if two lines intersect ie collision 
        if isIntersect(p1, p2, q1, q2):
            return True
            
    #print("no collision detected")
    return False

#check whether two lines intersect
def isIntersect(p1, p2, q1, q2):
        #conditions for intersection

        o1 = orientation(p1, p2, q1)
        o2 = orientation(p1, p2, q2)
        o3 = orientation(q1, q2, p1)
        o4 = orientation(q1, q2, p2)

        if (equalVerts(p1, q1) or equalVerts(p1, q2) or equalVerts(p2, q1) or equalVerts(p2, q2)):
                return False
        if (o1 != o2 and o3 != o4):
                return True
        return False

#utility fn
def equalVerts(v1, v2):
    return (v1[0] == v2[0] and v1[1] == v2[1])


#https://stackoverflow.com/questions/17592800/how-to-find-the-orientation-of-three-points-in-a-two-dimensional-space-given-coo
def orientation(p, q, r):
    # find orientation of ordered triplet (p, q, r)
    #---0 --> p, q and r are colinear  
    #---1 --> Clockwise  
    #---2 --> Counterclockwise 
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1]) 
    
    #return val
    if val == 0:
            return 0
    elif val > 0:
            return 1
    return 2

#given an ordered list of some object's vertices, returns its edges
def getEdges(obsVerts):
        n = len(obsVerts)
        lines = []
        
        for i  in range(n-1):
                a = obsVerts[i]
                b = obsVerts[i+1]
                elem = [a,b]
                lines.append(elem)

        lastLine = [obsVerts[0], obsVerts[n-1]]
        lines.append(lastLine)
        return lines
