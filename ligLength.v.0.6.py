#	ligLength.v.0.6.py 
#
#	This script calculates the shortest path (length) of a ligament from origin to
#	insertion wrapping around the proximal and distal bone meshes. It roughly follows
#	the RayScan approach by Hechenberger et al. 2020, in that it creates a partial 
#	on-the-fly visibility graph on an A* search, adding elements to its priority
#	queue based on arc sectors to find the shortest distance. 
#
#	Written by Oliver Demuth and Vittorio la Barbera 09.05.2022
#	Last updated 05.07.2022 - Oliver Demuth
#
#	SYNOPSIS:
#
#		INPUT params:
#			string  origin:			Name of the origin point, i.e. the name of a locator at the position of the ligament origin
#			string  insertion:		Name of the insertion point, i.e. the name of a locator at the position of the ligament insertion
#			string  jointcentre:	Name of the joint centre, i.e. the name of a locator or joint, e.g. "myJoint" if following the ROM mapping protocol of Manafzadeh & Padian 2018
#			string  proximal:		Name of the proximal bone mesh, e.g. the meshes that were used for the boolean if following the ROM mapping protocol of Manafzadeh & Padian 2018
#			string  distal:			Name of the distal bone mesh, e.g. the meshes that were used for the boolean if following the ROM mapping protocol of Manafzadeh & Padian 2018
#			int	 	res:			Integer value to define the resolution of the curve approximating the slices.
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
#	approximating the slices through the bone meshes. The lower the resolution 'res', 
#	the faster the script runs; however, accuracy is also reduced. 


# ========== load pymel ==========

import pymel.core as pm
import pymel.core.datatypes as dt


# ========== constants ==========

INF = 10000
CCW = 1
CW = -1
COLLINEAR = 0
FP_TOLERANCE = 10 # used to address floating point errors and truncate floating point numbers to a certain tolerance 
T = 10**FP_TOLERANCE
T2 = 10.0**FP_TOLERANCE
counter = 0


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

	def get_adjacent(self, point):
		if point == self.point1:
			return self.point2
		return self.point1

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

# ========== face area function ==========

def polyFacesTotArea( faces ): # Requires faces to be selected
	# Input variables:
	#	faces = selected faces for which surface area is to be calculated
	# ======================================== #

	# triangulation of faces

	pm.polyTriangulate(faces)
	vertIndicesStr = pm.polyInfo(fv=True)
	pm.select(clear=True)

	mesh = faces.split('.')

	# get number of faces

	faceNum = len(vertIndicesStr)
	faceTotArea = 0

	# area calculations
	for i in range(faceNum):  # get ID of each triangle vertex

		vertIndices = vertIndicesStr[i].split()

		VtxA = mesh[0] + '.vtx[' + vertIndices[2] + ']'
		VtxB = mesh[0] + '.vtx[' + vertIndices[3] + ']'
		VtxC = mesh[0] + '.vtx[' + vertIndices[4] + ']'

		# query position of each vertex and calculate distances between them

		VtxAPos = pm.xform(VtxA, query = True, worldSpace = True, translation = True)
		VtxBPos = pm.xform(VtxB, query = True, worldSpace = True, translation = True)
		VtxCPos = pm.xform(VtxC, query = True, worldSpace = True, translation = True)

		DistA = dt.Vector(VtxBPos[0] - VtxCPos[0],
							VtxBPos[1] - VtxCPos[1],
							VtxBPos[2] - VtxCPos[2]).length()
		DistB = dt.Vector(VtxAPos[0] - VtxCPos[0],
							VtxAPos[1] - VtxCPos[1],
							VtxAPos[2] - VtxCPos[2]).length()
		DistC = dt.Vector(VtxAPos[0] - VtxBPos[0],
							VtxAPos[1] - VtxBPos[1],
							VtxAPos[2] - VtxBPos[2]).length()

		# calculate area each triangle and sum it up
		s = (DistA + DistB + DistC) / 2
		faceTotArea += dt.sqrt(s * (s - DistA) * (s - DistB) * (s - DistC))

	return faceTotArea


# ========== polygon slice approximation function ==========

def polyApproxSlice( oPos, cutDir, uDir, vDir, mesh, res ):

	# Input variables:
	#	oPos = origin position
	#	cutDir = direction of cutting plane
	#	uDir = direction uf U axis on cutting plane
	#	vDir = direction of V axis on cutting plane
	#	mesh = mesh to be cut
	#	res = resolution for mesh slice approximation
	# ======================================== #

	# get number of faces

	numFace = pm.polyEvaluate(mesh, f=True)
	numF = numFace - 1
	meshF = mesh + '.f[0:' + str(numF) + ']'  # maya stuff to get the faces

	# cut mesh
	pm.polyCut(meshF,  # mesh
					 pc = (oPos[0], oPos[1], oPos[2]),  # origin pos
					 ro = (cutDir[0], cutDir[1], cutDir[2]),  # cut direction
					 ps = (1, 1),  # size of cut plane
					 eo = (0, 0, 0),  # offset of the plane
					 ef = 1)  # extract ???

	# get difference in face numbers of pre and post cutting
	numPreSlice = pm.polyEvaluate(mesh, f = True)

	pm.select(mesh)
	pm.polyCloseBorder()

	numPostSlice = pm.polyEvaluate(mesh, f = True)

	pm.select(clear=True)

	faceDiff = (numPostSlice - numPreSlice) // 2  # get number of newly created faces

	vtcs = []
	edges = []
	vtxNodes = []
	edgeNodes = []

	# get vertices and edges of mesh slice
	for i in range(faceDiff):
		newFaces = mesh + '.f[' + str(numPostSlice - i - 1) + ']'
		tempCRV = mesh + '_CRV_' + str(i)
		tempPlane = mesh + '_plane_' + str(i)
		tempSliceEdges = pm.polyListComponentConversion(newFaces, ff = True, te = True)

		# get slice area and translate it into resolution for curve circumscribing slice
		
		if res is not None:

			pm.select(newFaces)
			sliceArea = polyFacesTotArea(newFaces)
			log = dt.log(sliceArea)
			resolution = dt.ceil(res + (res * log))

			if resolution < 4:
				resolution = 4

		# approximate slice surface and reduce number of vertices and edges to a more reasonable number based on slice area
		pm.select(tempSliceEdges)
		pm.polyToCurve(form = 2, degree = 1, conformToSmoothMeshPreview = 0, n = tempCRV)
		pm.select(clear = True)

		# rebuild curve and approximate slice

		if res is not None:
			pm.rebuildCurve(tempCRV, ch = 1, rpo = 1, rt = 0, end = 1,
				kr = 0, kcp = 0, kep = 1, kt = 1, s = resolution,
				d = 1, tol = 1e-08)

		pm.nurbsToPolygonsPref(pt = 1, pc = 1)
		pm.planarSrf(tempCRV, ch = 1, d = 1, ko = 0, tol = 1e-01,
			rn = 0, po = 1, n = tempPlane)

		# add curve and plane to temporary group
		tempGRP = 'TempGRP'
		if not pm.objExists(tempGRP):  # check if temporary group exists
			pm.group(em = True, n = tempGRP)

		pm.parent(tempCRV, tempGRP)
		pm.parent(tempPlane, tempGRP)

		# get vertices and edges from new slice

		tempPlaneF = tempPlane + '.f[0]'

		nodePoints = []
		vertices = pm.polyListComponentConversion(tempPlaneF, ff = True, tv = True)
		vertices = pm.filterExpand(vertices, sm = 31)
		vtxNeighbors = []

		for j in range(len(vertices)):
			# extract vertex index and get name for node
			vtxID = pm.general.MeshVertex(vertices[j]).index()
			vtxNeighbors.append(pm.general.MeshVertex(vertices[j]).connectedVertices())

			# calculate UV values
			vtcPos = pm.xform(vertices[j], query = True, worldSpace = True, translation = True)
			pOffset = dt.Vector(oPos[0] - vtcPos[0],
								oPos[1] - vtcPos[1],
								oPos[2] - vtcPos[2])
			u = pOffset.dot(uDir)
			v = pOffset.dot(vDir)
			nodeUV = (u, v, 0)

			# get neighbors of each vertex
			nodePoints.append(nodePoint(uv = nodeUV, ID = (tempPlane, vtxID)))

		#get neighboring nodePoints

		for j in range(len(nodePoints)):
			neighbors = pm.filterExpand(vtxNeighbors[j], sm = 31)
			neighbor1ID = pm.general.MeshVertex(neighbors[0]).index()
			neighbor2ID = pm.general.MeshVertex(neighbors[1]).index()
			nodePoints[j].neighbors = nodeEdge(point1 = nodePoints[neighbor1ID], point2 = nodePoints[neighbor2ID])

		vtxNodes += nodePoints

		# triangulate plane to get all edges
		pm.select(tempPlaneF)
		pm.polyTriangulate(tempPlaneF)

		newFaces = pm.ls(selection = True)
		pm.select(clear = True)

		# get edges and create their respective nodeEdges
		nodeEdges = []
		edges = pm.polyListComponentConversion(newFaces, ff = True, te = True)
		edges = pm.filterExpand(edges, sm = 32)

		for j in range(len(edges)):
			edgeVertices = pm.polyListComponentConversion(edges[j], fe = True, tv = True)
			edgeVertices = pm.filterExpand(edgeVertices, sm = 31)
			id_vtx1 = pm.general.MeshVertex(edgeVertices[0]).index()
			id_vtx2 = pm.general.MeshVertex(edgeVertices[1]).index()

			nodeEdges.append(nodeEdge(point1 = nodePoints[id_vtx1], point2 = nodePoints[id_vtx2]))

		edgeNodes += nodeEdges

	return vtxNodes, edgeNodes


# ========== ligament length function ==========

def ligLength(origin, insertion, jointCentre, proxMesh, distMesh, resolution = None ):

	# Input variables:
	#	origin = name of ligament origin (represented by object, e.g. locator, in Maya scene)
	#	insertion = name of ligament insertion (represented by object, e.g. locator, in Maya scene)
	#	jointCentre = name of joint centre (represented by object, e.g. joint or locator, in Maya scene)
	#	proxMesh = name of proximal bone mesh to which ligament origin attaches
	#	distMesh = name of distal bone mesh to which ligament insertion attaches
	# 	res = resolution for mesh slice approximation
	# ======================================== #

	global counter 
	counter = 0

	# data house keeping
	duplicate_prox_name = proxMesh + '_duplicate'
	pm.duplicate(proxMesh, n = duplicate_prox_name)

	duplicate_dist_name = distMesh + '_duplicate'
	pm.duplicate(distMesh, n = duplicate_dist_name)

	# get positions of points of interest

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
		v1 = (0.0, 0.0, 1.0),
		v2 = (uCross[0], uCross[1], uCross[2]))

	# normalize vectors by distance between origin and insertion

	LigDirNorm = LigDir.normal() / Offset
	vCrossNorm = vCross / Offset

	# get obstacle slices, their vertices and edges

	VtxSet, EdgeSet = polyApproxSlice(oPos, cutAim, LigDirNorm, vCrossNorm,
		proxMesh, resolution)  # approximated slice of proximal bone mesh

	VtxSet2, EdgeSet2 = polyApproxSlice(oPos, cutAim, LigDirNorm, vCrossNorm,
		distMesh, resolution)  # approximated slice of distal bone mesh

	VtxSet.extend(VtxSet2)
	EdgeSet.extend(EdgeSet2)

	# run A* search

	ligLength = astar(oPos, iPos, VtxSet, EdgeSet)
	# ligLength = (VtxSet,EdgeSet)

	# clean up

	if pm.objExists('TempGRP'): # check if temporary group exists
		pm.delete('TempGRP')

	pm.delete(proxMesh, distMesh)
	pm.rename(duplicate_prox_name, proxMesh)
	pm.rename(duplicate_dist_name, distMesh)

	return ligLength



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

	if p0 in edge or p1 in edge:
		intersection = 0 #this is technically wrong but it stops the script from getting stuck on u. THIS CAUSES ISSUES!
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
		return p1
	if p0 in edge:
		return p0

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

	# Debugging

	global counter
	counter += 1 # increase for each scan for debugging
	print (counter)



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

	iDir = dt.Vector(I.uv[0]-u.uv[0],I.uv[1]-u.uv[1],0).normal()

	# get arc sector angles based on direction d

	if d == CW:
		dirAngle = dt.Vector(acwDir).angle(iDir) # arc sector angle in direction d
	else: # d == CCW or d == 0, i.e. colinear
		dirAngle = dt.Vector(accwDir).angle(iDir) # arc sector angle in direction d

	# loop through points in direction d to find turning point, however, if any angle falls outside the arc sector break loop

	turningPoints = [] # define turning point

	# get angle between vectors u to p and u to I

	for point in pointsDir:

		pDir = dt.Vector(point.uv[0]-u.uv[0],point.uv[1]-u.uv[1], 0).normal()
		point.angle = dt.Vector(pDir).angle(iDir) 

	# sort points by angle to in direction d

	pointsDir.sort(key = lambda point:point.angle) 

	# go through sorted points and find turning points

	for point in pointsDir:

		if point.angle > dirAngle: # check if point.angle falls outside the arc sector, if so break loop because we leave the angle sector
			print('outside arc sector')
			break

		if ccw(u, I, point) == ccw(u, I, point.neighbors.point1) == ccw(u, I, point.neighbors.point2): # all points are on the same side of the arc sector
			if point.neighbors.point1.angle < point.angle and point.neighbors.point2.angle < point.angle:
				turningPoints.append(point)
				print('Case 1', point.ID)

		elif ccw(u, I, point.neighbors.point1) == 0 and ccw(point, point.neighbors.point1, point.neighbors.point2) != 0: # neighbor 1 is u and points are not colinear
			if point.neighbors.point2.angle < point.angle or ccw(u, I, point) != ccw(u, I, point.neighbors.point2):
				turningPoints.append(point)
				print('Case 2', point.ID)

		elif ccw(u, I, point.neighbors.point2) == 0 and ccw(point, point.neighbors.point1, point.neighbors.point2) != 0: # neighbor 2 is u and points are not colinear
			if point.neighbors.point1.angle < point.angle or ccw(u, I, point) != ccw(u, I, point.neighbors.point1): 
				turningPoints.append(point)
				print('Case 3', point.ID)

		elif ccw(u, I, point.neighbors.point1) == ccw(u, I, point.neighbors.point2) != ccw(u, I, point): # both neighbors are on the other side of the arc sector
			turningPoints.append(point)
			print('Case 4', point.ID)

		elif ccw(u, I, point.neighbors.point1) != ccw(u, I, point): # only neighbor 1 is on the opposite side of the arc sector 
			if point.neighbors.point2.angle < point.angle and ccw(u, I, point) == ccw(u, I, point.neighbors.point2):
				turningPoints.append(point)
				print('Case 5', point.ID)

		elif ccw(u, I, point.neighbors.point2) != ccw(u, I, point): # only neighbor 2 is on the opposite side of the arc sector 
			if point.neighbors.point1.angle < point.angle and ccw(u, I, point) == ccw(u, I, point.neighbors.point1):
				turningPoints.append(point)
				print('Case 6', point.ID)

	print ('number of turningPoints',len(turningPoints))

	successors = []

	# if a turning point was found, check if it is visible

	if len(turningPoints) != 0 :

		for turningPoint in turningPoints:

			print ('turning point ID', turningPoint.ID)
			
			n = shoot_ray(u, turningPoint, edges)

			if counter > 100:
				break

			if n.ID[1] != -1:

				if n == turningPoint: # turningPoint is visible from u

					print ('turningPoint', n.ID, 'is visible')

					successors.append(n) # add it to successor list

				else: # turning point is not visible from u, need to recurse and scan again with subset of arc sector

					print ('turningPoint', n.ID, 'is not visible')

					nDir = dt.Vector(n.uv[0] - u.uv[0],n.uv[1] - u.uv[1],0).normal() # vector from u to n

					intersectionTurningPointsCW = scan(u, n, points, edges, CW, nDir, acwDir,) # scan from n to acw to find potential successors
					intersectionTurningPointsCCW = scan(u, n, points, edges, CCW, accwDir, nDir) # scan from n to accw to find potential successors

					# add potential successors to successors list

					successors.extend(intersectionTurningPointsCW)
					successors.extend(intersectionTurningPointsCCW)

	return successors


# ========== A* search algorithm ==========

def astar( start, end, points, edges ):

	# Input variables:
	#	start = 3D position of the starting point
	#	end = 3D position of the end point
	#	points = set of all points within path finding problem
	#	edges = set of all edges within path finding problem
	# ======================================== #

	global counter

	# Create start and end node

	start_node = nodePoint(uv = (0, 0, 0), ID = (None, 'start_node'))
	start_node.g = start_node.h = start_node.f = 0

	end_node = nodePoint(uv = (1, 0, 0), ID = (None, 'end_node'))
	end_node.g = end_node.h = end_node.f = 0

	# get distance between start and end point

	dist = dt.Vector(start[0]-end[0],start[1]-end[1],start[2]-end[2]).length()

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

		if counter > 100:
			break

		# Get the current node

		current_node = open_list[0]
		current_index = 0

		print('open_list:')

		for index, item in enumerate(open_list):

			print(item.ID)

			if item.f < current_node.f:

				current_node = item
				current_index = index

		# Pop current off open list, add to closed list

		print ('current_node', current_node.ID)

		open_list.pop(current_index)
		closed_list.append(current_node)

		# Found the end point

		if current_node == end_node:

			pathLength = current_node.g * dist

			# path might not be needed

			path = []
			current = current_node

			while current is not None:

				path.append(current.ID)
				current = current.parent

			return [1, pathLength, path[::-1]] # Return reversed path

		# shoot ray from current_node to end_node

		hitPoint = shoot_ray(current_node,end_node,edges)
		print('shoot ray from', current_node.ID, 'to', end_node.ID)

		if hitPoint != end_node:
			print('hitPoint neighbors', hitPoint.neighbors.point1.ID, hitPoint.neighbors.point2.ID)

		successors = []

		# check if end_node is visible

		if hitPoint == end_node: # end_node is visible from current_node

			hitPoint.parent = current_node
			successors.append(hitPoint)

		else: # end_node is not visible from current_node, thus need to define accwDir and acwDir for scan

			if current_node.parent is None: # current node has no parent, i.e. is starting point, thus arc sector is a circle

				accwDir = acwDir = (-1,0,0)

			else: # get accw and acw based on previous path point to define projection field

				# get parent_node of current_node and get previous direction

				parent_node = current_node.parent
				print ('parent_node', parent_node.ID)

				parentDir = dt.Vector(current_node.uv[0]-parent_node.uv[0],current_node.uv[1]-parent_node.uv[1],0).normal()

				# get neighboring nodes of current_node and get their orientation in relation to the parent_node

				neighborOne_node = current_node.neighbors.point1
				neighborTwo_node = current_node.neighbors.point2
				neighborOneOrient = ccw(parent_node,current_node,neighborOne_node)
				neighborTwoOrient = ccw(parent_node,current_node,neighborTwo_node)
				neighborOneDir = dt.Vector(neighborOne_node.uv[0]-current_node.uv[0],neighborOne_node.uv[1]-current_node.uv[1],0).normal()
				neighborTwoDir = dt.Vector(neighborTwo_node.uv[0]-current_node.uv[0],neighborTwo_node.uv[1]-current_node.uv[1],0).normal()

				# check if path was following edge of polygon in previous step

				if parent_node == neighborOne_node: # neighbor one was previous step
					if neighborTwoOrient == CW:
						acwDir == neighborTwoDir
						accwDir == parentDir
					else:
						acwDir == parentDir
						accwDir == neighborTwoDir

				elif parent_node == neighborTwo_node: # neighbor two was previous step
					if neighborOneOrient == CW:
						acwDir == neighborOneDir
						accwDir == parentDir
					else:
						acwDir == parentDir
						accwDir == neighborOneDir

				else: # neither neighbor was previous step, therefore need to check which way around the obstacle the path has to wrap

					end_nodeDir = dt.Vector(end_node.uv[0]-current_node.uv[0],end_node.uv[1]-current_node.uv[1],0).normal()
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

			turningPoints = []

			print('scan CCW', accwDir, hitPoint.ID)
			turningPointsCCW = scan(current_node, hitPoint, points, edges, CCW, accwDir, acwDir)

			print('scan CW', acwDir, hitPoint.ID)
			turningPointsCW = scan(current_node, hitPoint, points, edges, CW, accwDir, acwDir)

			turningPoints.extend(turningPointsCCW)
			turningPoints.extend(turningPointsCW)

			print ('number of successors = ', len(turningPoints))

			for point in turningPoints:
				successors.append(point)
				print('successors')
				print(point.ID)

		# Loop through successors

		for successor in successors:

			# check if successor is in closed_list

			for closed_successors in closed_list:
				if successor == closed_successors:
					continue

			# calculate temporary g value

			tempG = current_node.g + dt.Vector(current_node.uv[0]-successor.uv[0],current_node.uv[1]-successor.uv[1],0).length()

			if successor in open_list: # if successor is already in open_list compare g values
				if tempG < successor.g:
					successor.g = tempG
					successor.parent = current_node # get parent node to trace back path

			else: # successor is not yet in open_list, add it to open_list and set g value
				successor.g = tempG
				successor.parent = current_node # get parent node to trace back path
				open_list.append(successor)

			# calculate h and f values

			successor.h = dt.Vector(successor.uv[0]-end_node.uv[0],successor.uv[1]-end_node.uv[1],0).length()
			successor.f = successor.h + successor.g

	return [0, -1, None] # no path found
