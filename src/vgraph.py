#!/usr/bin/env python

import numpy as np # I update the numpy to 1.15.1 using sudo pip install --upgrade numpy
import cv2
from scipy.spatial import ConvexHull, convex_hull_plot_2d
from marker import myMarker
import create_map as cm
import djikstra as dj
import rospy

def main():

    #load Obstacles and goals 
    obstacles = cm.load_obstacles("../data/world_obstacles.txt")
    goal = cm.load_goal("../data/goal.txt")
    start = [0., 0.]

    #sublist holding start and end points for later 
    strtEnd = [start, goal]

    #some list we will need 
    obstacleHulls = [] #<------list of all obsticle hulls as list of vertices
    obsVertices = []   #<------list of all objectHull vertices
    obsEdges = []      #<------list of all edges of obsticleHulls

    #this is for now 
    mk = myMarker()
        
    # draw obstacles
    for ob in obstacles:

        #get the points for the hull
        obMod = cm.extraPointsMod(ob)

        #this is where the points of the hull will be stored
        obHull = []
                
        #returns indexes of hull vertex points in obMod 
        obHullInx = ConvexHull(obMod).vertices
        #append the vertex to the list 
        for i in obHullInx:
            obHull.append(obMod[i])

        #extends the list of all hulls 
        obstacleHulls.append(obHull)
        obsVertices.extend(obHull)
        
        #adds edges to list of edges
        obsLines = cm.getEdges(obHull)
        obsEdges.extend(obsLines)

        for line in obsLines:
            mk.marker(line)

    #all vertices including start and end
    allVertices = obsVertices + strtEnd

    #list of all edges connecting vertices of object hulls avoid collisions
    #add start and end to hull 
    obstacleHulls.append(strtEnd)
    lines = cm.createLines(obstacleHulls, obsEdges)
        
	#get and print path
    path = dj.getPath(allVertices,lines,start,goal)
    dj._print(path)

	#draw edges(red)
    for line in lines:
        mk.marker(line)
                
	#draw path(green)
    for line in path:
        mk.marker(line,True)
        
if __name__ == '__main__':
    main()
