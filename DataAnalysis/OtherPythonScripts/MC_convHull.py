# MC_convHull.py
#
#	This script calculates the circumference of a bone based on a cutting
#	plane using Andrew's monotone chain convex hull algorithm in Autodesk
#	Maya. Cut a bone with a plane using a Boolean operation and select the
#   	resulting slice and run this script. 
#
#	Written by Oliver Demuth 28.11.2022


# ========== load plugins ==========

import pymel.core as pm
import pymel.core.datatypes as dt
import gc

# ========== constants plugins ==========

FP_TOLERANCE = 10 # used to address floating point errors and truncate floating point numbers to a certain tolerance 
T = 10**FP_TOLERANCE
T2 = 10.0**FP_TOLERANCE

# ========== object definitions ==========

class uvPoint(object):
	def __init__(self, uv = None, ID = None):
		self.uv = uv  # 2D position (u,v,0)
		self.ID = ID # vertex ID for debugging

def MC_convHull(selection):

	# Input variables:
	#	mesh = selected mesh, i.e., boolean slice through bone
	# ======================================== #

	# get mesh and all its vertices

	Mesh = pm.listRelatives(selection)
	MeshShape = pm.nodetypes.Mesh(Mesh[0])
	vertices = MeshShape.vtx
	vertices = pm.filterExpand(vertices, sm = 31)

	vtcPos = []

	# go through vertices and get global 3D coordinates

	for i in range(len(vertices)):
		vtcPos.append(dt.Vector(pm.xform(vertices[i],  # get vertex position
			query = True, worldSpace = True, translation = True)))

	# get centre position of mesh, i.e., centre of bounding box

	meanPos = dt.Vector(pm.objectCenter(Mesh))

	# calculate vectors from meanPoint to any two vertices to define plane

	pointA = vtcPos[0]
	pointB = vtcPos[1]

	ADir = dt.Vector(meanPos[0] - pointA[0],
			 meanPos[1] - pointA[1],
			 meanPos[2] - pointA[2])

	BDir = dt.Vector(meanPos[0] - pointB[0],
			 meanPos[1] - pointB[1],
			 meanPos[2] - pointB[2])

	Offset = ADir.length()

	# calculate cut plane direction

	uCross = ADir.cross(BDir).normal()
	vCross = ADir.cross(uCross).normal()

	uDirNorm = ADir.normal() / Offset
	vDirNorm = vCross / Offset

	uvPoints = []

	# go through vertices and calculate UV coordinates

	for j in range(len(vertices)):
		
		vtxID = vertices[j]

		tempVtx = vtcPos[j]
		pOffset = dt.Vector(meanPos[0] - tempVtx[0], # calculate offset from origin
				    meanPos[1] - tempVtx[1],
				    meanPos[2] - tempVtx[2])
							
		u = pOffset.dot(uDirNorm) # calculate u value
		v = pOffset.dot(vDirNorm) # calculate v value
		pointUV = (u, v, 0)
		uvPoints.append(uvPoint(uv = pointUV, ID = vtxID))

	# calculate convex hull

	hull = convex_hull(uvPoints)

	# calculate perimeter/circumference of convex hull

	circumference = perimeter(hull) * Offset

	return circumference


def convex_hull(points):

	# Input variables:
	#	points = points on mesh slice (2D coordinates are stored in uv attribute in Point class)
	# ======================================== #

	points.sort(key=lambda point:[point.uv[0],point.uv[1]]) # sort points based on u and v coordinates

	if len(points) <= 1:
		return points

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
	# Last point of each list is omitted because it is repeated at the beginning of the other list. 

	return lower[:-1] + upper[:-1]


def perimeter(hull_points):

	# Input variables:
	#	hull_points = points on the convex hull (2D coordinates are stored in uv attribute in Point class)
	# ======================================== #

	perimeter = 0

	# Find the distance between adjacent points on the convex hull and sum them up

	for k in range(len(hull_points)-1):

		p0 = hull_points[k]
		p1 = hull_points[k+1]

		perimeter += dt.Vector(p0.uv[0] - p1.uv[0], p0.uv[1] - p1.uv[1], 0).length()
		
	# Add distance between the first and last point to the perimeter

	start = hull_points[0]
	end = hull_points[len(hull_points)-1]

	perimeter += dt.Vector(start.uv[0] - end.uv[0], start.uv[1] - end.uv[1], 0).length();

	gc.collect()

	return perimeter


def ccw(A, B, C):

	# Input variables:
	#	A = first corner of triangle (2D coordinates are stored in uv attribute in Point class)
	#	B = second corner of triangle (2D coordinates are stored in uv attribute in Point class)
	#	C = third corner of triangle (2D coordinates are stored in uv attribute in Point class)
	# ======================================== #

	area = int(((B.uv[0] - A.uv[0]) * (C.uv[1] - A.uv[1]) - (B.uv[1] - A.uv[1]) * (C.uv[0] - A.uv[0])) * T) / T2

	# Return 1 if counter clockwise, -1 if clock wise, 0 if collinear

	if area > 0:
		return 1
	elif area < 0:
		return -1
	else:
		return 0


selection = pm.ls(sl=True)

circ = MC_convHull(selection)

print(circ)
