#!/usr/bin/env python

import numpy as np # I update the numpy to 1.15.1 using sudo pip install --upgrade numpy
import cv2
from scipy.spatial import ConvexHull, convex_hull_plot_2d
from marker import myMarker


		
def load_obstacles(object_path):
	'''
	Function to load a list of obstacles.
	The obstacle txt file show points in clockwise order

	Return:
		3d list [[[1, 2], [3, 4], [5, 6]], 
						[[7, 8], [9, 10], [10, 11]]]
	'''
	obstacles = []
	obstacle = []
	with open(object_path) as f:
		numObstacles = int(f.readline()) #5 
		coordinates = int(f.readline())  #3 
                for i in range(coordinates):     #in range 3 
			line = f.readline()
			obstacle.append(list(map(int, line.strip().split(' '))))
		for line in f:
			coordinates = list(map(int, line.strip().split(' ')))
			if len(coordinates) == 1:
				obstacles.append(obstacle)
                                #print("this obsticle: " + str(obstacle))-------------------------------print
				obstacle = []
			else:
				obstacle.append(coordinates)
	obstacles.append(obstacle)
 	assert len(obstacles)==numObstacles, "number of obstacles does not match the first line"
        
        #-----------------------------------------------printing for my sanity------------
        #print("the list of obsticles: ")
        #print(obstacles)
	return obstacles

def load_goal(goal_path):
	with open(goal_path) as f:
		line = f.readline()
		goal = list(map(int, line.strip().split(' ')))
	return goal #some point (600, 0) in our case





        
        
#-----------------------------------getting all points for convex hull--------------

def extraPointsMod(obsticle):
        #print("appending shit")-------------------------------------------------------------------print
        newObsticle = []
        for point in obsticle:
                #print("dealing with point: " + str(point))----------------------------------------print
                a = point[0]  #----------------x 
                b = point[1]  #---------------y
                w = 18        #---------------robot val assuming its in the middle
                #all points with a sqaure at every corner of orgininal vertex 
                newObsticle.append([a-w, b+w])
                newObsticle.append([a-w, b])
                newObsticle.append([a-w, b-w])
                
                newObsticle.append([a, b+w])
                newObsticle.append([a,b])
                newObsticle.append([a, b-w])
                
                newObsticle.append([a+w, b+w])
                newObsticle.append([a+w, b])
                newObsticle.append([a+w, b-w])

        return newObsticle





#-----------------------------------------------creating the lines between vertices-----------------------
def createLines(obsHulls, obsEdges):

        #print("obsHulls: " + str(obsHulls))--------------------------------------------------print 
        n = len(obsHulls)
        lines = []
        #print( "obsVerts: " + str(obsVerts))
        #print( "allVerts: " + str(allVerts))
        #print("all hulls: " + str(obsHulls))
        for i in range(n):
                hull1 = obsHulls[i]
                
                for a in hull1:
                        #print("point a: " + str(a))----------------------------------------print
                        for j in range(i+1,n):
                                hull2 = obsHulls[j]
                                for b in hull2:
                                        #print("point b: " + str(b))------------------------print 
                                        elem = [a,b]
                                        #print(elem)-----------------------------------------print
                                        if not isCollision(a,b,obsEdges) :
                                                lines.append(elem)
                

        return lines



#-----------------------------------------checks if there are collisions between obsticle hull and lines between points
def isCollision(p1,p2, obsEdges):
        for edge in obsEdges:
                #print("edge: " + str(edge))-------------------------------------------------------------------print
                q1 = edge[0]
                q2 = edge[1]
                #checks if two lines intersect ie collision 
                if isIntersect(p1, p2, q1, q2):
                        return True
                
        #print("no collision detected")
        return False



#-------------------------------------------- helper function to see if lines intersect----------------------------------
def isIntersect(p1, p2, q1, q2):
        #conditions for intersection
        '''
        print("p1: " + str(p1))
        print("p2: " + str(p2))
        print("q1: " + str(q1))
        print("q2: " + str(q2))
        '''

        o1 = orientation(p1, p2, q1)
        o2 = orientation(p1, p2, q2)
        o3 = orientation(q1, q2, p1)
        o4 = orientation(q1, q2, p2)

        if (equalVerts(p1, q1) or equalVerts(p1, q2) or equalVerts(p2, q1) or equalVerts(p2, q2)):
                return False
        if (o1 != o2 and o3 != o4):
                return True
        return False

#----------------------------------------used above for is Intersect 
def equalVerts(v1, v2):
    return (v1[0] == v2[0] and v1[1] == v2[1])




#https://stackoverflow.com/questions/17592800/how-to-find-the-orientation-of-three-points-in-a-two-dimensional-space-given-coo

def orientation(p, q, r):
        #------------find orientation of ordered triplet (p, q, r)
     
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




#---------------------given a list of vertices for some object returns its edges as set of 2 points--------
def getEdges(obsVerts):
        n = len(obsVerts)
        lines = []
        #print( "obsVerts: " + str(obsVerts))----------------------------------------------------------print
        
        for i  in range(n-1):
                a = obsVerts[i]
                b = obsVerts[i+1]
                elem = [a,b]
                lines.append(elem)

        lastLine = [ obsVerts[0], obsVerts[n-1]]
        lines.append(lastLine)
        return lines










""" 
------------------------------------------
(0, 0) is at the center of the map;
(0, 0) is at the top left of the image

Thus, we need some transformation
------------------------------------------
""" 

def map2img(ob):
	""" transform an obstacle in map frame to img frame """
	ob_tranformed = []
	t = np.array([[1, 0, 0, 300], 
					[0, -1, 0, 300], 
					[0, 0, -1, 0],
					[0, 0, 0, 1]])
	for p in ob:
		p = np.array(p) # in case it is not already numpy array
		p = np.hstack((p, [0, 1]))
		p = t.dot(p).astype(int)
		ob_tranformed.append(p[:2])
	return np.array(ob_tranformed)

def img2map(ob):
	""" transform an obstacle in img frame to map frame """
	ob_tranformed = []
	t = np.array([[1, 0, 0, -300], 
					[0, -1, 0, 300],
					[0, 0, -1, 0],
					[0, 0, 0, 1]])
	for p in ob:
		p = np.array(p) # in case it is not already numpy array
		p = np.hstack((p, [0, 1]))
		p = t.dot(p).astype(int)
		ob_tranformed.append(p[:2])
	return np.array(ob_tranformed)

if __name__ == "__main__":


        
	# Create a black image
	img = np.full((600,1200,3), 255, np.uint8)
	obstacles = load_obstacles("../data/world_obstacles.txt")
	goal = load_goal("../data/goal.txt")
	start = [0, 0]

        #sublist holding start and end points for later 
        strtEnd = [start, goal]

        #some list we will need 
        obstacleHulls = [] #<------list of all obsticle hulls as list of vertices
        #obsVertices = []   #<------list of all objectHull vertices
        obsEdges = []      #<------list of all edges of obsticleHulls
       
        
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
                #obsVertices.extend(obHull)
                #adds edges to list of edges
                obsLines = getEdges(obHull)
                obsEdges.extend(obsLines)
                #obsEdgeLst.append(obsLines)
                



                #-------------------------------actual drawing---------------
                #convert orginal obstcl to img
		ob = map2img(ob)
                #convert hull to img 
                hull = map2img(obHull)
                
                #print("the hull is: " + str(hull))
		# print(len(ob))
		# print(ob)
                
                #actually takes the obsticles as points to display 
		cv2.fillConvexPoly(img, ob.reshape(-1, 1, 2), (255,255,0))

                
                


        
        #allVertices = obsVertices + strtEnd#<--------------------list of all vertices including start and end
        #list of all edges connecting vertices of object hulls avoid collisions
        obstacleHulls.append(strtEnd)
        lines = createLines(obstacleHulls,obsEdges)
        
                
        
        

	# draw start and goal point
	goal = tuple(map2img([goal])[0])
	start = tuple(map2img([start])[0])
	cv2.circle(img, goal, 7, (100, 0, 0), -1)
	cv2.circle(img, start, 7, (0, 0, 100), -1)

	# import pdb; pdb.set_trace()
	# cv2.polylines(img,[pts],True,(0,255,255), thickness=1)

	cv2.imshow( "Display window", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()
	cv2.imwrite('../maps/map.png',img)
