#!/usr/bin/env python

import numpy as np # I update the numpy to 1.15.1 using sudo pip install --upgrade numpy
import cv2
from scipy.spatial import ConvexHull, convex_hull_plot_2d

		
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
                                print("this obsticle: " + str(obstacle))
				obstacle = []
			else:
				obstacle.append(coordinates)
	obstacles.append(obstacle)
	assert len(obstacles)==numObstacles, "number of obstacles does not match the first line"

        #for my sanity 
        print("the list of obsticles: ")
        print(obstacles)
	return obstacles

def load_goal(goal_path):
	with open(goal_path) as f:
		line = f.readline()
		goal = list(map(int, line.strip().split(' ')))
	return goal #some point (600, 0) in our case





#--------------------------------------jarvis Hull Algorithm Implementation----------------------------------

def leftMostPointIndex(points):
        #------------Finding the left most point---------------
        print("doing leftMost")

        mIndex = 0
        for i in range(len(points)):
            #points[i][0]--->x value of point
            if points[i][0] < points[mIndex][0]: 
                mIndex = i
        #handles ties in y ---want highest 
            elif points[i][0] == points[mIndex][0]: 
                if points[i][1] > points[mIndex][1]: 
                    mIndex = i
                    
        return mIndex




#https://stackoverflow.com/questions/17592800/how-to-find-the-orientation-of-three-points-in-a-two-dimensional-space-given-coo

def orientation(p, q, r):
        #------------find orientation of ordered triplet (p, q, r)
     

        val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1]) 
  
        return val



def convexHull(points):
        print("doing Hull shit")
        n = len(points)

        if n < 3:
                return

        
        j = leftMostPointIndex(points)
        p = j
        q = 0
        hull = []
        Done = False 
        while not Done :
                hull.append(points[p])

                #find a point q s.t orientation(p,x,q) is counterClockwise forAll x
                #keep track of most counter clockwise update q accordingly 
                q = (p+1) % n

                for i in range(n):
                        if(orientation(points[p],points[i], points[q]) == 2):
                                q = i

                #p is updated 
                p = q 

                if p == j:
                        Done = True
                        
        return hull 
        
        
#-----------------------------------getting all points for convex hull--------------

def extraPointsMod(obsticle):
        print("appending shit")
        newObsticle = []
        for point in obsticle:
                print("dealing with point: " + str(point))
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
def createLines(obsVerts, allVerts,obsEdges):
        
        n = len(allVerts)
        lines = []
        print( "obsVerts: " + str(obsVerts))
        print( "allVerts: " + str(allVerts))
        
        for i  in range(n):
                a = allVerts[i]
                for j in range(i+1,n):
                        b = allVerts[j]
                        elem = [a,b]
                        if not isCollision(a,b,obsEdges) :
                                lines.append(elem)
                

        return lines


#--------------------------------------------function to see if lines intersect----------------------------------
def doIntersect(p1, p2, q1, q2):
        #conditions for intersection
        '''
        print("p1: " + str(p1))
        print("p2: " + str(p2))
        print("q1: " + str(q1))
        print("q2: " + str(q2))
        '''
        A = min(p1[1],p2[1]) < max(q1[1],q2[1])
        B = min(q1[1],q2[1]) < max(p1[1],p2[1])
        C = min(q1[0],q2[0]) < max(p1[0],p2[0])
        D = min(p1[0],p2[0]) < max(q1[0],q2[0])
        E = orientation(p1,p2,q1) * orientation(p1,p2,q2) < 0
        F = orientation(q1,q2,p1) * orientation(q1,q2,p2) < 0

        
        if A and B and C and D and E and F :
                return True
        return False


def isCollision(p1,p2, obsEdges):
        for v1, v2 in obsEdges:
                #checks if two lines intersect ie collision 
                if doIntersect(p1, p2, v1, v2):
                        return True
                
        #print("no collision detected")
        return False


def getEdges(obsVerts):
        n = len(obsVerts)
        lines = []
        print( "obsVerts: " + str(obsVerts))
        
        for i  in range(n):
                a = obsVerts[i]
                for j in range(i+1,n):
                        b = obsVerts[j]
                        elem = [a,b]
                        lines.append(elem)
                

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

        strtEnd = [start, goal]

        #list of all vertex points for obstacle hulls 
        obstacleHulls = []
        obsVertices = []
        obsEdges = []
       
        
        
	# draw obstacles
	for ob in obstacles:
                
                #get the points for the hull
                obMod = extraPointsMod(ob)

                #this is where the points of the hull will be stores
                obHull = []
                #returns indexes of hull vertex points in obMod 
                obHullInx = ConvexHull(obMod).vertices
                #append the vertex to the thing 
                for i in obHullInx:
                        obHull.append(obMod[i])

                #extends the list of all hulls 
                obstacleHulls.append(obHull)
                obsVertices.extend(obHull)
                obsLines = getEdges(obHull)
                obsEdges.extend(obsLines)
                
                
                
                #obHull = ConvexHull(obMod)
                print("the hull is: ")
                print(obHull)

                #convert orginal obstcl to img
		ob = map2img(ob)
                #convert hull to img 
                hull = map2img(obHull)
                #print("the hull is: " + str(hull))
		# print(len(ob))
		# print(ob)
                
                #actually takes the obsticles as points to display 
		cv2.fillConvexPoly(img, ob.reshape(-1, 1, 2), (255,255,0))
                #outline of the thing 
                cv2.drawContours(img, [hull.reshape(-1, 1, 2)],-1, (255,255, 10))


        #obsVertices = obstacleHulls
        allVertices = obsVertices + strtEnd
        lines = createLines(obsVertices,allVertices,obsEdges)
        print(lines)
        #lemme check dis shit 
        for line in lines:
                lineImg = map2img(line)
                cv2.drawContours(img, [lineImg.reshape(-1, 1, 2)],-1, (255,255, 10))
                
        
        

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
