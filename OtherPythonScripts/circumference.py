# circumference.py
#
#	This script calculates the circumference of a bone based on a cutting
#	plane using Andrew's monotone chain convex hull algorithm in Autodesk
#	Maya. First run this script and then cut a bone with a plane using a 
#	Boolean operation. Select the resulting slice and execute the command
#	'circumference()' in the Python command line in Maya. 
#
#	Written by Oliver Demuth 
#	Last updated 28.04.2025 - Oliver Demuth 
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
	
	vertices = fnMesh.getPoints(4) # get world position of all vertices (om.MSpace.kWorld = 4)

	# get coordinates and transform them into matrices
	
	vtcPos = np.array(vertices)
	vtxArr = np.stack([np.eye(4)] * vtcPos.shape[0], axis = 0)
	vtxArr[:,3,:] = vtcPos # append coordinates to rotation matrix array

	# get centre position of mesh (i.e., centre of bounding box)

	fnMeshMat = np.eye(4)
	fnMeshMat[3,:] = np.array(fnMesh.boundingBox.center)
	transMat = np.array(dag.inclusiveMatrix()).reshape(4,4) # mesh transofrmation matrix

	meanPos = om.MPoint(np.dot(fnMeshMat,transMat)[3,:]) # bounding box centre poosition

	# calculate vectors from meanPoint to any two vertices to define plane

	ADir = meanPos - vertices[0]
	BDir = meanPos - vertices[1]

	Offset = ADir.length()

	# calculate cut plane direction

	uDir = ADir.normal() 
	wDir = (BDir.normal() ^ uDir).normal() # cross product to get z axis
	vDir = (wDir ^ uDir).normal() # cross product to get y axis 

	uDirNorm = uDir * Offset
	vDirNorm = vDir * Offset
	wDirNorm = wDir * Offset

	# get transformation matrix for cutting plane

	rotMat = np.array([[uDirNorm.x, uDirNorm.y, uDirNorm.z, 0],
			   [vDirNorm.x, vDirNorm.y, vDirNorm.z, 0],
			   [wDirNorm.x, wDirNorm.y, wDirNorm.z, 0],
			   [meanPos[0],meanPos[1],meanPos[2],1]]) # centre position around midpoint

	# calculate new coordinates and extract them

	uvCoords = np.dot(vtxArr,np.linalg.inv(rotMat))[:,3,0:2] # only need first two values (i.e., u and v coordinates)

	# calculate convex hull

	hullPoints = MC_convex_hull(uvCoords.tolist())

	# calculate perimeter/circumference of convex hull

	return sum(np.sqrt(sum(np.diff(hullPoints,axis=0)**2)))


def MC_convex_hull(points):

	# Input variables:
	#	points = points on mesh slice (2D coordinates U and V) as a Python list
	# ======================================== #

	# sort points based on u and v coordinates
	
	points.sort(key=lambda point:[point[0],point[1]]) 

	# if equal to or less than 1 point return points list
	
	if len(points) <= 1:
		return np.array(points)

	# get lower hull 

	lower = []

	for p in points:
		while len(lower) >= 2 and ccw(lower[-2], lower[-1], p) <= 0:
			lower.pop()
		lower.append(p)

	# get upper hull

	upper = []

	for p in reversed(points):
		while len(upper) >= 2 and ccw(upper[-2], upper[-1], p) <= 0:
			upper.pop()
		upper.append(p)

	# Concatenation of the lower and upper hulls gives the convex hull.
	# Last point of lower list is omitted because it is repeated at the beginning of the upper list.
	# Start point is duplicated for circumference calculation.

	return np.array(lower[:-1] + upper)


def ccw(A, B, C):

	# Input variables:
	#	A = first corner of triangle (2D coordinates U and V)
	#	B = second corner of triangle (2D coordinates U and V)
	#	C = third corner of triangle (2D coordinates U and V)
	# ======================================== #

	# Return 1 if counter clockwise, -1 if clock wise, 0 if collinear

	area = int(((B[0] - A[0]) * (C[1] - A[1]) - (B[1] - A[1]) * (C[0] - A[0]))*10**10)/10.0**10 # prevents floating point errors
		
	if area > 0:
		return 1
	elif area < 0:
		return -1
	else:
		return 0
