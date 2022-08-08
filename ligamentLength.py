#	ligamentLength.py 
#
#	This script calculates the shortest path (length) of a ligament from origin to
#	insertion wrapping around the proximal and distal bone meshes. It approximates
#	the ligament length by reducing the 3D problem into a 2D Euclidean shortest path 
#	problem. It roughly follows the RayScan approach by Hechenberger et al. 2020, in 
#	that it creates a partial on-the-fly visibility graph on an A* search, adding  
#	elements to its priority queue based on arc sectors, to find the shortest path. 
#
#	Written by Oliver Demuth and Vittorio la Barbera 09.05.2022
#	Last updated 08.08.2022 - Oliver Demuth
#
#	SYNOPSIS:
#
#		INPUT params:
#			string  origin:			Name of the origin point, i.e. the name of a locator at the position of the ligament origin
#			string  insertion:		Name of the insertion point, i.e. the name of a locator at the position of the ligament insertion
#			string  jointcentre:	Name of the joint centre, i.e. the name of a locator or joint, e.g. "myJoint" if following the ROM mapping protocol of Manafzadeh & Padian 2018
#			string  proximal:		Name of the proximal bone mesh, e.g. the meshes that were used for the boolean if following the ROM mapping protocol of Manafzadeh & Padian 2018
#			string  distal:			Name of the distal bone mesh, e.g. the meshes that were used for the boolean if following the ROM mapping protocol of Manafzadeh & Padian 2018
#			bool 	reverse:		Boolean value, if true path will be computed from insertion to origin. If none is given, false will be assumed and path computed from origin to insertion
#
#		RETURN params:
#			float	pathLength[]:	Return value is a float array with two elements in the form of:
#
#			element 0: 
#				0	| False:		failure, no shortest path (end point cannot be reached from start point through Pointset and NeighborSet): [0.0,-1.0]
#				1	| True:			shortest path found: [1.0,..]
#
#			element 1:
#				pathLength:			Length of path from starting point to end point
#
#
#	Note, the accuracy of the reported length depends on the resolution of the curve 
#	approximating the slices through the bone meshes. The lower the resolution, the
#	faster the script runs; however, accuracy is also reduced. 


# ========== load plugins ==========

import pymel.core as pm
import pymel.core.datatypes as dt
import gc

# ========== constants ==========

CCW = 1
CW = -1
COLLINEAR = 0
FP_TOLERANCE = 10 # used to address floating point errors and truncate floating point numbers to a certain tolerance 
T = 10**FP_TOLERANCE
T2 = 10.0**FP_TOLERANCE


# ========== object definitions ==========

#A Point class for A* Pathfinding

class nodePoint(object):
	def __init__(self, uv = None, ID = -1, neighbors = None, angle = None, dist = None, parent = None, g = 0, h = 0, f = 0):
		self.uv = uv  # 2D position (u,v,0)
		self.ID = ID  # touple of (polygonID, vtxID)
		self.neighbors = neighbors # nodeEdge with two neighbors
		self.angle = angle # angle in rad
		self.dist = dist # distance to other point
		self.parent = parent # parent node

		# g, h and f values for A* search

		self.g = g
		self.h = h
		self.f = f


# An Edge class for A* Pathfinding

class nodeEdge(object):

	def __init__(self, point1, point2):
		self.point1 = point1
		self.point2 = point2

	def __contains__(self, point):
		return self.point1 == point or self.point2 == point

	def __eq__(self, edge):
		if self.point1 == edge.point1 and self.point2 == edge.point2:
			return True
		if self.point1 == edge.point2 and self.point2 == edge.point1:
			return True
		return False



################################################
# ========== MAYA specific functions ========= #
################################################

# ========== ligament length function ==========

def ligLength( origin, insertion, jointCentre, ligPlane, booObject, reverse = False ):

	# Input variables:
	#	origin = name of ligament origin (represented by object, e.g. locator, in Maya scene)
	#	insertion = name of ligament insertion (represented by object, e.g. locator, in Maya scene)
	#	jointCentre = name of joint centre (represented by object, e.g. joint or locator, in Maya scene)
	#	ligPlane = name of plane which acts as cutting plane
	#	booObject = name of boolean object to get obstacles
	#	reverse = boolean attribute to define the direction of the ligament origin to insertion or insertion to origin
	# ======================================== #

	# get positions of points of interest

	if reverse == True or reverse == 1:
		oPos = pm.xform(insertion, query = True, worldSpace = True, translation = True)
		iPos = pm.xform(origin, query = True, worldSpace = True, translation = True)
	else:
		oPos = pm.xform(origin, query = True, worldSpace = True, translation = True)
		iPos = pm.xform(insertion, query = True, worldSpace = True, translation = True)

	jPos = pm.xform(jointCentre, query = True, worldSpace = True, translation = True)

	# calculate vectors from origin to insertion

	LigDir = dt.Vector(oPos[0] - iPos[0],
					   oPos[1] - iPos[1],
					   oPos[2] - iPos[2])

	JointDir = dt.Vector(oPos[0] - jPos[0],
						 oPos[1] - jPos[1],
						 oPos[2] - jPos[2])

	Offset = LigDir.length()

	# calculate cut plane direction

	uCross = LigDir.cross(JointDir).normal()
	vCross = LigDir.cross(uCross).normal()

	cutAim = pm.angleBetween(euler = True, 
		v1 = (0.0, 1.0, 0.0),
		v2 = (uCross[0], uCross[1], uCross[2]))

	planeRotate = ligPlane + '.rotate'
	planeTranslate = ligPlane + '.translate'

	pm.setAttr(planeRotate, cutAim[0], cutAim[1], cutAim[2], type="double3")
	pm.setAttr(planeTranslate, jPos[0], jPos[1], jPos[2], type="double3")

	# normalize vectors by distance between origin and insertion

	LigDirNorm = LigDir.normal() / Offset
	vCrossNorm = vCross / Offset

	# get obstacle slices, their vertices and edges

	VtxSet, EdgeSet = getObstacles( oPos, booObject, cutAim, LigDirNorm, vCrossNorm )  # get vertices and edges of boo

	# run A* search

	ligLength = astar( Offset, VtxSet, EdgeSet )

	# garbage collection

	for nodeEdge in EdgeSet:
		del nodeEdge
	del EdgeSet
	
	for nodePoint in VtxSet:
	    del nodePoint
	del VtxSet

	gc.collect()


	return ligLength



	# ========== polygon slice approximation function ==========

def getObstacles( oPos, mesh, cutDir, uDir, vDir,):

	# Input variables:
	#	oPos = origin position
	#	mesh = boolean mesh to be approximated
	#	cutDir = direction of cutting plane
	#	uDir = direction uf U axis on cutting plane
	#	vDir = direction of V axis on cutting plane
	# ======================================== #

	# get number of faces

	numFace = pm.polyEvaluate(mesh, f=True) # number of faces
	numF = numFace - 1
	meshFaces = mesh + '.f[0:' + str(numF) + ']'  # get string of all faces in the mesh
	meshFaces = pm.filterExpand(meshFaces, sm =  34)
	numShell = pm.polyEvaluate(mesh, s=True) # number of shells

	# create empty lists

	tempShellFaces = meshFaces
	shellFaces = []

	vtcs = []
	edges = []
	vtxNodes = []
	edgeNodes = []

	# get vertices and edges of mesh slice

	for i in range(numShell):

		# get faces of individual shells (disconnected face islands), i.e. the obstacles, from the first face of temShellFaces

		shellFace = pm.general.MeshFace(tempShellFaces[0])
		connectedShellFaces = shellFace.connectedFaces()

		# check if shell, i.e. obstacle, consists of multiple faces 

		if len(connectedShellFaces) > 0:

			openShellFaces = []
			closedShellFaces = []

			openShellFaces.append(shellFace)

			# cycle through all connected faces of shell

			while len(openShellFaces) > 0:

				# get all connected faces to the current face

				current_face = pm.filterExpand(openShellFaces[0],sm=34)
				current_face = current_face[0]
				connectedFaces = pm.general.MeshFace(current_face).connectedFaces()
				connectedFaces = pm.filterExpand(connectedFaces, sm=34)

				# pop current face from openShellFaces and add it to closedShellFaces

				openShellFaces.pop(0)
				closedShellFaces.append(current_face)

				# go through connected faces and add them to openShellFaces if they haven't been referenced yet

				for connectedFace in connectedFaces:
					connectedFace = pm.filterExpand(connectedFace,sm=34) 
					connectedFace = connectedFace[0]

					# only add faces that have not yet been added (doesn't work 100%); required to use 'set' below to filter them out

					openShellFaces.append(connectedFace) if connectedFace not in closedShellFaces else None 

			shellFaces = set(closedShellFaces) # get all faces of shell and filter out duplicates
		else:
			shellFaces = shellFace # shell is only made up of a single face

		shellFaces = pm.filterExpand(shellFaces, sm =  34)

		# remove shell faces from tempShellFaces, i.e. already checked faces will not be checked again when cycling through shells

		for face in shellFaces:	
			tempShellFaces.remove(face)

		tempSliceEdges = pm.polyListComponentConversion(shellFaces, ff = True, te = True) # get all edges from the slices
		tempPlane = mesh + '_plane_' + str(i) # name of the slice 

		vertices = pm.polyListComponentConversion(shellFaces, ff = True, tv = True)
		vertices = pm.filterExpand(vertices, sm = 31)

		# define variables for nodes and their neighbors

		nodePoints = []
		vtxNeighbors = []
		vertexIndices = []

		# go through vertices and create nodePoints

		for j in range(len(vertices)):

			# extract vertex index and get name for node

			vtxID = pm.general.MeshVertex(vertices[j]).index()
			vertexIndices.append(vtxID)
			vtxNeighbors.append(pm.general.MeshVertex(vertices[j]).connectedVertices())

			# get position of 2D plane (uv) from 3D position (xyz)

			vtcPos = pm.xform(vertices[j], query = True, worldSpace = True, translation = True) # get vertex position

			pOffset = dt.Vector(oPos[0] - vtcPos[0], # calculate offset from origin
								oPos[1] - vtcPos[1],
								oPos[2] - vtcPos[2])

			u = pOffset.dot(uDir) # calculate u value
			v = pOffset.dot(vDir) # calculate v value
			nodeUV = (u, v, 0)

			# create nodes for each vertex

			nodePoints.append(nodePoint(uv = nodeUV, ID = (tempPlane, vtxID)))

		# get neighboring nodePoints

		for j in range(len(nodePoints)):

			neighbors = pm.filterExpand(vtxNeighbors[j], sm = 31)

			# if more than 2 neighbors, remove the ones that are not part of the face
			getNeighbors =[]

			if len(neighbors) > 2:
				for neighbor in neighbors:
					if pm.general.MeshVertex(neighbor).index() in vertexIndices:
						getNeighbors.append(neighbor)
			else: getNeighbors = neighbors

			neighbor1ID = pm.general.MeshVertex(getNeighbors[0]).index() # get neighbor 1 vertex ID
			neighbor1Index = vertexIndices.index(neighbor1ID) # get neighbor 1 vertex index

			neighbor2ID = pm.general.MeshVertex(getNeighbors[1]).index() # get neighbor 2 vertex ID
			neighbor2Index = vertexIndices.index(neighbor2ID) # get neighbor 2 vertex index

			nodePoints[j].neighbors = nodeEdge(point1 = nodePoints[neighbor1Index], point2 = nodePoints[neighbor2Index])

		vtxNodes += nodePoints

		# get all edges from the slices

		edges = pm.polyListComponentConversion(shellFaces, ff = True, te = True)
		edges = pm.filterExpand(edges, sm = 32)

		# go through each edge to create nodeEdge

		nodeEdges = []

		for j in range(len(edges)):

			edgeVertices = pm.polyListComponentConversion(edges[j], fe = True, tv = True)
			edgeVertices = pm.filterExpand(edgeVertices, sm = 31)

			id_vtx1 = pm.general.MeshVertex(edgeVertices[0]).index() # get vertex ID of vertex 1
			vtx1IDIndex = vertexIndices.index(id_vtx1) # get index of vertex 2

			id_vtx2 = pm.general.MeshVertex(edgeVertices[1]).index() # get vertex ID of vertex 2
			vtx2IDIndex = vertexIndices.index(id_vtx2) # get index of vertex 2

			# get nodePoints for vertices

			nodeEdges.append(nodeEdge(point1 = nodePoints[vtx1IDIndex], point2 = nodePoints[vtx2IDIndex]))

		edgeNodes += nodeEdges

	return vtxNodes, edgeNodes


################################################
# ========== Path finding functions ========== #
################################################

# ========== check intersection function ==========

def intersect( p0, p1, edge ):

	# Input variables:
	#	p0 = start point of line segment
	#	p1 = end point of line segment
	#	edge = line segment to check if it crosses (intersects) segment p0p1
	# ======================================== #

	# check if either point is in edge

	if p1 in edge:
		intersection = 1
		return intersection

	if p0 in edge:
		intersection = 0 # this is technically incorrect, however it stops the script from getting stuck on u
		return intersection

	# get edge points from nodeEdge class

	p2 = edge.point1
	p3 = edge.point2

	# extract UV coordinates

	p0pos = p0.uv
	p1pos = p1.uv
	p2pos = p2.uv
	p3pos = p3.uv

	# segment calculations

	s1x = p1pos[0] - p0pos[0]
	s1y = p1pos[1] - p0pos[1]
	s2x = p3pos[0] - p2pos[0]
	s2y = p3pos[1] - p2pos[1]

	if (((-s2x * s1y + s1x * s2y) * T) / T2) == 0: # line segments are parallel

		intersection = -1
		return intersection

	else: # line segments not parallel

		v = (-s1y * (p0pos[0] - p2pos[0]) + s1x * (p0pos[1] - p2pos[1])) / (-s2x * s1y + s1x * s2y)
		w = (s2x * (p0pos[1] - p2pos[1]) - s2y * (p0pos[0] - p2pos[0])) / (-s2x * s1y + s1x * s2y)

	if v > 0 and v < 1 and w > 0 and w < 1: # segments intersect

		intersection = 1

	else: # they don't intersect

		intersection = 0

	return intersection


# ========== get intersection point function ==========

def intersect_point(p0, p1, edge):

	# Input variables:
	#	p0 = start point of line segment
	#	p1 = end point of line segment
	#	edge = line segment to calculate intersection point with
	# ======================================== #

		# check if either point is in intersection

	if p1 in edge:
		intersect_dist = dt.Vector(p0.uv[0] - p1.uv[0], p0.uv[1] - p1.uv[1], 0).length()

		p1.dist = intersect_dist

		return p1

	# get name, polygon and neighbors

	intersect_Poly = edge.point1.ID[0] # get name of polygon from tuple

	# get intersection point uv coordinates and return nodePoint of intersection point

	if edge.point1.uv[0] == edge.point2.uv[0]:
		if p0.uv[0] == p1.uv[0]:
			return None

		pslope = (p0.uv[1] - p1.uv[1]) / (p0.uv[0] - p1.uv[0])
		intersect_u = edge.point1.uv[0]
		intersect_v = pslope * (intersect_u - p0.uv[0]) + p0.uv[1]
		intersect_UV = (intersect_u,intersect_v,0)

		intersect_dist = dt.Vector(p0.uv[0] - intersect_u, p0.uv[1] - intersect_v, 0).length()

		return nodePoint(uv = intersect_UV, ID = (intersect_Poly, -1), neighbors = edge, dist = intersect_dist)

	if p0.uv[0] == p1.uv[0]:
		eslope = (edge.point1.uv[1] - edge.point2.uv[1]) / (edge.point1.uv[0] - edge.point2.uv[0])
		intersect_u = p0.uv[0]
		intersect_v = eslope * (intersect_u - edge.point1.uv[0]) + edge.point1.uv[1]
		intersect_UV = (intersect_u,intersect_v,0)

		intersect_dist = dt.Vector(p0.uv[0] - intersect_u, p0.uv[1] - intersect_v, 0).length()

		return nodePoint(uv = intersect_UV, ID = (intersect_Poly, -1), neighbors = edge, dist = intersect_dist)

	pslope = (p0.uv[1] - p1.uv[1]) / (p0.uv[0] - p1.uv[0])
	eslope = (edge.point1.uv[1] - edge.point2.uv[1]) / (edge.point1.uv[0] - edge.point2.uv[0])

	if eslope == pslope:
		return None

	intersect_u = (eslope * edge.point1.uv[0] - pslope * p0.uv[0] + p0.uv[1] - edge.point1.uv[1]) / (eslope - pslope)
	intersect_v = eslope * (intersect_u - edge.point1.uv[0]) + edge.point1.uv[1]
	intersect_UV = (intersect_u,intersect_v,0)

	intersect_dist = dt.Vector(p0.uv[0] - intersect_u, p0.uv[1] - intersect_v, 0).length()

	return nodePoint(uv = intersect_UV, ID = (intersect_Poly, -1), neighbors = edge, dist = intersect_dist)


# ========== angle direction function ==========

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


# ========== shoot ray function ==========

def shoot_ray( point, target, edges ):

	# Input variables:
	#	point = origin point of ray
	#	target = target point of ray
	#	edgeSet = edges to be checked if they intersect ray
	# ======================================== #

	intersectP = []

	for edge in edges:
		if intersect(point, target, edge) == 1: # if segments intersect get intersection point
			intersectP.append(intersect_point(point,target,edge))

	if len(intersectP) > 0: # get closest intersection point

		intersectP.sort(key = lambda point:point.dist)
		return intersectP[0]

	else: # if no intersection found returns target

		return target


# ========== scan function ==========

def scan(u, I, points, edges, d, accwDir, acwDir):

	# Input variables:
	#	u = starting point
	# 	I = target point and/or intersection point
	# 	points = set of all points within path finding problem
	# 	edges = set of all edges within path finding problem
	# 	d = direction of scan, either CCW or CW
	# 	accwDir = normalised vector of the direction of the counter clockwise angle sector border, i.e. accwDir = dt.Vector(accw.uv-u.uv).normal(). Variable is optional and if not suplied is assumed to point into opposite direction from u to I and thus the arc being a half circle
	# 	acwDir = normalised vector of the direction of the clockwise angle sector border, i.e. acwDir = dt.Vector(acw.uv-u.uv).normal(). Variable is optional and if not suplied is assumed to point into opposite direction from u to I and thus the arc being a half circle
	# ======================================== #

	# make direction fool proof

	if d > 0:
		d = 1
	if d < 0:
		d =- 1

	# get subset of points that are part of intersection polygon

	polygonSubset = []
	if u.ID[0] == None:
		polygonSubset = points
	elif I.ID[0] == None:
		polygonSubset = points
	else:
		for point in points:
			if point.ID[0] == I.ID[0]:
				polygonSubset.append(point)

	# get subset of points on polygon I in direction d

	pointsDir = []
	for point in polygonSubset: # get points of pointsSubset in direction d
		if ccw(u,I,point) == d: # check if point is in the same direction as d
			pointsDir.append(point) # if so, add it to the pointsDir subset (in direction d)

	# get direction of I

	iDir = dt.Vector(I.uv[0] - u.uv[0], I.uv[1] - u.uv[1], 0).normal()

	# get arc sector angles based on direction d

	if d == CW:
		dirAngle = dt.Vector(acwDir).angle(iDir) # arc sector angle in direction d
		endAngle = dt.Vector(acwDir).angle(dt.Vector(1 - u.uv[0], 0 - u.uv[1], 0).normal())

	else: # d == CCW or d == 0, i.e. colinear
		dirAngle = dt.Vector(accwDir).angle(iDir) # arc sector angle in direction d
		endAngle = dt.Vector(accwDir).angle(dt.Vector(1 - u.uv[0], 0 - u.uv[1], 0).normal())

	# loop through points in direction d to find turning point, however, if any angle falls outside the arc sector break loop

	turningPoints = [] # define turning point

	# get angle between vectors u to p and u to I

	for point in pointsDir:

		pDir = dt.Vector(point.uv[0] - u.uv[0], point.uv[1] - u.uv[1], 0).normal()
		point.angle = dt.Vector(pDir).angle(iDir) 

	# sort points by angle to in direction d

	pointsDir.sort(key = lambda point:point.angle) 

	# go through sorted points and find turning points

	for point in pointsDir:

		if point.angle > dirAngle: # check if point.angle falls outside the angle sector, if so break loop because we leave the angle sector
			break

		if ccw(u, I, point) == ccw(u, I, point.neighbors.point1) == ccw(u, I, point.neighbors.point2): # all points are on the same side of scan line
			if point.neighbors.point1.angle < point.angle and point.neighbors.point2.angle < point.angle: # point angle is bigger than both neighboring angles
				turningPoints.append(point)

		elif u.ID == point.neighbors.point1.ID: # neighbor 1 is u
			if ccw(u, point, point.neighbors.point2) != ccw(u, I, point) and ccw(u, point, point.neighbors.point2) != 0: # points are not colinear and change in direction, i.e turning point
				turningPoints.append(point)

		elif u.ID == point.neighbors.point2.ID: # neighbor 2 is u
			if ccw(u, point, point.neighbors.point1) != ccw(u, I, point) and ccw(u, point, point.neighbors.point1) != 0: # points are not colinear and change in direction, i.e turning point
				turningPoints.append(point)

		elif ccw(u, I, point.neighbors.point1) == ccw(u, I, point.neighbors.point2) != ccw(u, I, point): # both neighbors are on the other side of the scan line from point
		 	turningPoints.append(point)

		elif ccw(u, I, point.neighbors.point1) != ccw(u, I, point): # neighbor 1 is on the opposite side of the scan line from point
			if point.neighbors.point2.angle < point.angle and ccw(u, I, point) == ccw(u, I, point.neighbors.point2): # point angle is bigger than neighbor 2 ange AND point and neighbor 2 are on the same side
				turningPoints.append(point)

		elif ccw(u, I, point.neighbors.point2) != ccw(u, I, point): # neighbor 2 is on the opposite side of the arc sector 
			if point.neighbors.point1.angle < point.angle and ccw(u, I, point) == ccw(u, I, point.neighbors.point1): # point angle is bigger than neighbor 1 ange AND point and neighbor 1 are on the same side
				turningPoints.append(point)

	successors = []

	# if a turning point was found, check if it is visible

	if len(turningPoints) != 0 :

		for turningPoint in turningPoints:
			
			n = shoot_ray(u, turningPoint, edges)

			if n.ID[1] != -1:

				if n == turningPoint: # turningPoint is visible from u

					successors.append(n) # add it to successor list

				else: # turning point is not visible from u, need to recurse and scan again with subset of arc sector

					nDir = dt.Vector(n.uv[0] - u.uv[0], n.uv[1] - u.uv[1], 0).normal() # vector from u to n

					intersectionTurningPointsCW = scan(u, n, points, edges, CW, nDir, acwDir,) # scan from n to acw to find potential successors
					intersectionTurningPointsCCW = scan(u, n, points, edges, CCW, accwDir, nDir) # scan from n to accw to find potential successors

					# add potential successors to successors list

					successors.extend(intersectionTurningPointsCW)
					successors.extend(intersectionTurningPointsCCW)

	return successors


# ========== A* search algorithm ==========

def astar( Offset, points, edges ):

	# Input variables:
	#	start = 3D position of the starting point
	#	end = 3D position of the end point
	#	points = set of all points within path finding problem
	#	edges = set of all edges within path finding problem
	# ======================================== #

	# Create start and end node

	start_node = nodePoint(uv = (0, 0, 0), ID = (None, 'start_node'))
	start_node.g = start_node.h = start_node.f = 0

	end_node = nodePoint(uv = (1, 0, 0), ID = (None, 'end_node'))
	end_node.g = end_node.h = end_node.f = 0

	# add nodes to VtxSet

	points.append(start_node)
	points.append(end_node)

	# Initialize both open and closed list

	open_list = []
	closed_list = []

	# Add the start node

	open_list.append(start_node)

	# Loop until you find the end

	while len(open_list) > 0:

		# Get the current node

		current_node = open_list[0]
		current_index = 0

		for index, item in enumerate(open_list):

			if item.f < current_node.f:

				current_node = item
				current_index = index

		# Pop current off open list, add to closed list

		open_list.pop(current_index)
		closed_list.append(current_node)

		# Found the end point

		if current_node == end_node:

			pathLength = current_node.g * Offset
			return [1, pathLength]

		# shoot ray from current_node to end_node

		hitPoint = shoot_ray(current_node,end_node,edges)

		successors = []

		# check if end_node is visible

		if hitPoint == end_node: # end_node is visible from current_node

			hitPoint.parent = current_node
			successors.append(hitPoint)

		else: # end_node is not visible from current_node, thus need to define accwDir and acwDir for scan

			if current_node.parent is None: # current node has no parent, i.e. is starting point, thus angle sector is a circle

				accwDir = acwDir = (-1,0,0)

			elif current_node.parent.ID[0] != current_node.ID[0]:

				accwDir = acwDir = (-1,0,0)

			else: # get accw and acw based on previous path point to define projection field

				# get parent_node of current_node and get previous direction

				parent_node = current_node.parent
				parentDir = dt.Vector(current_node.uv[0] - parent_node.uv[0], current_node.uv[1] - parent_node.uv[1], 0).normal()

				# get neighboring nodes of current_node and get their orientation in relation to the parent_node

				neighborOne_node = current_node.neighbors.point1
				neighborOneOrient = ccw(parent_node,current_node,neighborOne_node)
				neighborOneDir = dt.Vector(neighborOne_node.uv[0] - current_node.uv[0], neighborOne_node.uv[1] - current_node.uv[1], 0).normal()
				
				neighborTwo_node = current_node.neighbors.point2
				neighborTwoOrient = ccw(parent_node,current_node,neighborTwo_node)
				neighborTwoDir = dt.Vector(neighborTwo_node.uv[0] - current_node.uv[0], neighborTwo_node.uv[1] - current_node.uv[1], 0).normal()

				# check if path was following edge of polygon in previous step and set acwDir and accwDir accordingly

				end_nodeDir = dt.Vector(end_node.uv[0] - current_node.uv[0], end_node.uv[1] - current_node.uv[1], 0).normal()

				if parent_node == neighborOne_node: # neighbor one was previous step
					if neighborTwoOrient == CW:
						acwDir = end_nodeDir
						accwDir = parentDir
					else:
						acwDir = parentDir
						accwDir = end_nodeDir

				elif parent_node == neighborTwo_node: # neighbor two was previous step
					if neighborOneOrient == CW:
						acwDir = end_nodeDir
						accwDir = parentDir
					else:
						acwDir = parentDir
						accwDir = end_nodeDir

				else: # neither neighbor was previous step, therefore need to check which way around the obstacle the path has to wrap

					neighborOneAngle = neighborOneDir.angle(end_nodeDir)
					neighborTwoAngle = neighborTwoDir.angle(end_nodeDir)

					if neighborOneOrient == CCW and neighborTwoOrient == CCW:

						# both neighboring nodes are on the CCW side and the acw direction can, therefore, be restricted to the previous ray direction

						acwDir = parentDir

						# find which angle is smaller and thus going to be acw dir

						if neighborOneAngle < neighborTwoAngle:
							accwDir = neighborOneDir
						else:
							accwDir = neighborTwoDir

					elif neighborOneOrient == CW and neighborTwoOrient == CW:

						# both neighboring nodes are on the CW side and the accw direction can, therefore, be restricted to the previous ray direction

						accwDir = parentDir

						# find which angle is smaller and thus going to be acw dir

						if neighborOneAngle < neighborTwoAngle:
							acwDir = neighborOneDir
						else:
							acwDir = neighborTwoDir

			# find turningpoints on polygon hitPoint.ID[0] and if not visible on polygon hitPoint'.ID[0]

			turningPointsCCW = scan(current_node, hitPoint, points, edges, CCW, accwDir, acwDir)
			turningPointsCW = scan(current_node, hitPoint, points, edges, CW, accwDir, acwDir)

			turningPoints = []
			turningPoints.extend(turningPointsCCW)
			turningPoints.extend(turningPointsCW)

			for point in turningPoints:
				successors.append(point)

		# Loop through successors

		for successor in successors:

			if successor.ID[0] != current_node.ID[0] and successor.ID[0] != None: # successor and current point are on different meshes

				recurseHitPoints = []

				# go back through closed list and check if there is a better point that was missed because only one mesh was checked

				for closedPoint in closed_list:
					recurseHitPoint = shoot_ray(successor,closedPoint,edges)
					if recurseHitPoint == closedPoint: # an already closed point is visible from successor
						recurseHitPoints.append(point)

				if len(recurseHitPoints) > 0:
					recurseHitPoints.sort(key = lambda point:point.g) # sort visible points (from current successor) in closed list by g value
					
					if recurseHitPoints[0].ID != current_node.ID: # check if the best visible one is already the current_node
						successor.parent = recurseHitPoints[0]
						tempG = successor.parent.g + dt.Vector(current_node.uv[0] - successor.uv[0], current_node.uv[1] - successor.uv[1], 0).length()

			# check if successor is in closed_list

			for closed_successors in closed_list:
				if successor == closed_successors:
					continue

			# calculate temporary g value

			tempG = current_node.g + dt.Vector(current_node.uv[0] - successor.uv[0], current_node.uv[1] - successor.uv[1], 0).length()

			if successor in open_list: # if successor is already in open_list compare g values
				if tempG < successor.g:
					successor.g = tempG
					successor.parent = current_node # get parent node to trace back path

			else: # successor is not yet in open_list, add it to open_list and set g value
				successor.g = tempG
				successor.parent = current_node # get parent node to trace back path
				open_list.append(successor)

			# calculate h and f values

			successor.h = dt.Vector(successor.uv[0] - end_node.uv[0], successor.uv[1] - end_node.uv[1], 0).length()
			successor.f = successor.h + successor.g

	return [0, -1] # no path found

