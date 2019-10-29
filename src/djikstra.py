#!/usr/bin/env python

import math
import numpy as np # I update the numpy to 1.15.1 using sudo pip install --upgrade numpy
import cv2
import pickle
from scipy.spatial import ConvexHull, convex_hull_plot_2d


def main():
	
	#wait for allLines and allVertices
	print("hi")
                
def setup(allVertices, allLines, start, goal):

    points = []
    startP = None
    endP = None
    pointsD = dict()

	#build list of points, and associate to dic
    for v in allVertices:
        p = Point(v) 
        if v == start:
            startP = p
            startP.dist = 0
        if v == goal:
            endP = p
        points.append(p)
        pointsD[str(v)] = p

   	#connect all neighbors 
    for p in points:
        n = []
        for line in allLines:
            if p.pos == line[0]:
                n.append(line[1])
            if p.pos == line[1]:
                n.append(line[0])
        p.neighbors = n

    print("done setting up!")

    print("calculating path...")
    path = getPath(points,pointsD,startP,endP)
    print('done!')

	#save the path
    with open('path.data', 'wb') as filehandle:
        pickle.dump(path, filehandle)
	
	return path

class Point:
    def __init__(self,pos=[0,0], dist=float('inf'),neighbors=[]):
        self.pos = pos
        self.dist = dist
        self.neighbors = neighbors
        self.par = None

    def __str__(self):
        return "" + str(self.pos) + ": d=" + str(self.dist) + ", n=" + str(len(self.neighbors))


def _print(points):
    for p in points:
        print(str(p))

def getPath(points,dic,start,end):
    if not start in points:
        points.insert(0,start)
    if not end in points:
        points.append(end)

    #make list "unchecked" of all points and their dists
    unchecked = [] 
    for p in points:
        unchecked.append(p)

    path = []
    lines = []
    found = False
    while not found and len(unchecked) > 0:

        small, unchecked = getMin(unchecked)
        for n in small.neighbors:
            n = dic[str(n)]
            if n in unchecked:
                small_to_n = math.sqrt(
                    (n.pos[0]-small.pos[0])**2 + 
                    (n.pos[1]-small.pos[1])**2 )
                if n.dist > small.dist+small_to_n:
                    n.dist = small.dist + small_to_n
                    n.par = small
        if (small == end):
            print("found end!")
            curr = small

            while not found:
                lines.append([curr.pos, curr.par.pos])
                path.append(curr)
                curr = curr.par
                if curr is start:
                    path.append(curr)
                    found = True 
	
	lines.reverse()
	for arm in lines:
			arm.reverse()

    return lines

def getMin(points):
    smallest = points[0]
    for p in points:
        if p.dist < smallest.dist:
            smallest = p
    points.remove(smallest)
    return smallest, points

if __name__ == "__main__":
    main()
