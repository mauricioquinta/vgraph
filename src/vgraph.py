#!/usr/bin/env python

import numpy as np # I update the numpy to 1.15.1 using sudo pip install --upgrade numpy
import cv2
from scipy.spatial import ConvexHull, convex_hull_plot_2d
from marker import myMarker
from create_map import load_obstacles, load_goal, orientation, extraPointsMod, createLines, isCollision, getEdges, isIntersect, map2img, img2map
import djikstra
import rospy

def main():

    
    #load Obstacles and goals 
    obstacles = load_obstacles("../data/world_obstacles.txt")
    goal = load_goal("../data/goal.txt")
    start = [0, 0]

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

        #----------------------------------------work for the hull and list info--------------------
        #get the points for the hull
        obMod = extraPointsMod(ob)

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
        obsLines = getEdges(obHull)
        obsEdges.extend(obsLines)
                
        #print("the hull is: ")
        #print(obHull)



        #-------------------------------actual drawing hopefully publishing each hull---------------
        for line in obsLines:
            #hull = map2img(obHull)
            #line = map2img(line)
            print("line: " + str(line))
            #print(str(line))
            mk.marker(line)
     

        
    allVertices = obsVertices + strtEnd#<--------------------list of all vertices including start and end
    #list of all edges connecting vertices of object hulls avoid collisions
    #add start and end as hull 
    obstacleHulls.append(strtEnd)
    lines = createLines(obstacleHulls, obsEdges)
        
    path = djikstra.setup(allVertices,lines,start,goal)
    print(path)
    #-------------------draws lines onto the map------------
    for line in lines:
        #lineImg = map2img(line)
        mk.marker(line)
        #line = 0
        #cv2.drawContours(img, [lineImg.reshape(-1, 1, 2)],-1, (255,255, 10))
                
    for line in path:
        mk.marker(line,True)
        
        

if __name__ == '__main__':
    main()

        
