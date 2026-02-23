#	circumference.py
#
#	This script calculates the circumference of a bone based on a cutting
#	plane using Andrew's monotone chain convex hull algorithm in Autodesk
#	Maya. First run this script and then cut a bone with a plane using a 
#	Boolean operation. Select the resulting slice and execute the command
#	'circumference()' in the Python command line in Maya. 
#
#	Written by Oliver Demuth 
#	Last updated 23.02.2026 - Oliver Demuth 
#
#
#	IMPORTANT notes:
#		
#	(1) This script requires several modules for Python (see README files). Make sure
#	    to have the following external modules installed for the mayapy application:
#
#		- 'numpy' 	NumPy:			https://numpy.org/about/
#
#	    For further information regarding them, please check the website(s) referenced 
#	    above.


# ========== load plugins ==========

import maya.api.OpenMaya as om
import numpy as np

# ========== function definitions ==========

def circumference():

	# get dag object from selection
	
	sel = om.MGlobal.getActiveSelectionList() # get selected mesh (i.e., boolean slice through bone)
	dag = sel.getDagPath(0)
	shape = dag.extendToShape()

	# get mesh object
	
	fnMesh = om.MFnMesh(shape)

	# get vertices from mesh
	
	vertices = np.array(fnMesh.getPoints(4)) # get world position of all vertices (om.MSpace.kWorld = 4)
	meanPos = vertices.mean(axis = 0)

	# calculate vectors from meanPoint to any two vertices to define plane

	x = vertices[0] - meanPos
	x = x / np.linalg.norm(x) # normalised
	z = np.cross(vertices[1] - meanPos, x)
	z = z / np.linalg.norm(z) # normalised
	y = np.cross(z, x)

	# assemble transformation matrix (i.e., inverse of slice coordinate system)
	
	transMat = np.eye(4)
	transMat[0:3,0] = x
	transMat[0:3,1] = y
	transMat[0:3,2] = z
	transMat[3,0:3] = -meanPos @ transMat[0:3,0:3]
	
	# calculate new coordinates and extract them

	uvCoords = (vertices @ transMat)[:,0:2] # only need first two values (i.e., u and v coordinates)

	# calculate convex hull

	hullPoints = MC_convex_hull(uvCoords)

	# calculate perimeter/circumference of convex hull

	diff = hullPoints[1:] - hullPoints[:-1]
	
	return np.sqrt((diff * diff).sum(axis = 1)).sum()


def MC_convex_hull(points):

	# Input variables:
	#	points = points on mesh slice (2D coordinates U and V) as a np.array
	# ======================================== #

	if points.shape[0] <= 1: 
		return points.copy()

	# sort points based on u and v coordinates
	
	idx = np.lexsort((points[:,1], points[:,0])) 
	points = points[idx]

	# get lower hull 

	lower = []
	for p in points:
		while len(lower) >= 2 and ccw(lower[-2], lower[-1], p) <= 0:
			lower.pop()
		lower.append(p)

	# get upper hull

	upper = []
	for p in points[::-1]:
		while len(upper) >= 2 and ccw(upper[-2], upper[-1], p) <= 0:
			upper.pop()
		upper.append(p)

	# Concatenation of the lower and upper hulls gives the convex hull.
	# Last point of lower list is omitted because it is repeated at the beginning of the upper list.
	# Start point is duplicated for circumference calculation.

	return np.vstack((lower[:-1],upper))


def ccw(A, B, C):

	# Input variables:
	#	A = first corner of triangle (2D coordinates U and V)
	#	B = second corner of triangle (2D coordinates U and V)
	#	C = third corner of triangle (2D coordinates U and V)
	# ======================================== #

	return (B[0] - A[0]) * (C[1] - A[1]) - (B[1] - A[1]) * (C[0] - A[0])
	
