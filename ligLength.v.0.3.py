#	ligLength.v.0.3.py 
#
#   This script calculates the shortest path (length) of a ligament from origin to
#   insertion wrapping around the proximal and distal bone meshes. It generates an
#   on-the-fly visibility graph embedded in an A* search algorithm to find the 
#   shortest distance. 
#
#   Written by Oliver Demuth and Vittorio la Barbera 09.05.2022
#   Last updated 11.05.2022 - Oliver Demuth
#
#   SYNOPSIS:
#
#       INPUT params:
#           string  origin:        	Name of the origin point, i.e. the name of a locator at the position of the ligament origin
#           string  insertion:     	Name of the insertion point, i.e. the name of a locator at the position of the ligament insertion
#           string  jointcentre:   	Name of the joint centre, i.e. the name of a locator or joint, e.g. "myJoint" if following the ROM mapping protocol of Manafzadeh & Padian 2018
#           string  proximal:      	Name of the proximal bone mesh, e.g. the meshes that were used for the boolean if following the ROM mapping protocol of Manafzadeh & Padian 2018
#           string  distal:        	Name of the distal bone mesh, e.g. the meshes that were used for the boolean if following the ROM mapping protocol of Manafzadeh & Padian 2018
#           int     res:           	Integer value to define the resolution of the curve approximating the slices.
#
#       RETURN params:
#           float   pathLength[]:	Return value is a float array with three elements in the form of:
#
#           element 0: 
#               0   | False:    	failure, no shortest path (end point cannot be reached from start point through Pointset and NeighborSet): [0.0,-1.0]
#               1   | True:			shortest path found: [1.0,..]
#
#           element 1:
#               pathLength:        	Length of path from starting point to end point
#			
#			element 2:
#				path: 				A list of the elements building the path
#
#
#   Note, the accuracy of the reported length depends on the resolution of the curve 
#   approximating the slices through the bone meshes. The lower the resolution 'res', 
#   the faster the script runs; however, accuracy is also reduced. 
#   
#

# ========== load pymel ==========

import pymel.core as pm
import pymel.core.datatypes as dt


# ========== constants ==========

PI = 3.141592653589793
INF = 10000
CCW = 1
CW = -1
COLLINEAR = 0
FP_TOLERANCE = 10 # used to address floating point errors and truncate floating point numbers to a certain tolerance 
T = 10**FP_TOLERANCE
T2 = 10.0**FP_TOLERANCE


# ========== object definitions ==========

#A Point class for A* Pathfinding

class nodePoint(object):
    __slots__ = ('name', 'parent', 'pos', 'uv', 'dist' 'g', 'h', 'f', 'poly', 'ID')

    def __init__(self, name, parent=None, pos=None, uv=None, dist=0, g=0, h=0, f=0, neighbors=None, poly=-1, ID=-1):
        self.name = name
	self.parent = parent
        self.pos = pos # 3D position (x,y,z)
        self.uv = uv # 2D position (u,v,0)
        self.dist = dist #float value
	self.neighbors = neighbors # neighboring nodes
        self.poly = poly # polygon mesh
	self.ID = ID # vertex ID
        
        # g, h and f values for A* search

        self.g = g
        self.h = h
        self.f = f

    def __eq__(self, other):
        return self.uv == other.uv

    def __ne__(self, other):
        return not self.__eq__(other)


# An Edge class for A* Pathfinding

class nodeEdge(object):
    __slots__ = ('p1', 'p2')

    def __init__(self, point1, point2):
        self.p1 = point1
        self.p2 = point2

    def get_adjacent(self, point):
        if point == self.p1:
            return self.p2
        return self.p1

    def __contains__(self, point):
        return self.p1 == point or self.p2 == point

    def __eq__(self, edge):
        if self.p1 == edge.p1 and self.p2 == edge.p2:
            return True
        if self.p1 == edge.p2 and self.p2 == edge.p1:
            return True
        return False



################################################
# ========== Path finding functions ========== #
################################################


# ========== check intersection function ==========

def intersect( p0, p1, edge ):
	
	# Input variables:
	# 	p0 = start point of line segment
	#	p1 = end point of line segment
	#	edge = line segment to check if it crosses (intersects) segment p0p1
	# ======================================== #

    
	# get edge points from nodeEdge class

	p2 = edge.p1 
	p3 = edge.p2

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

	if (((-s2x * s1y + s1x * s2y)*T)/T2) == 0: # line segments are parallel

		intersection = -1

		return intersection

	else: # line segments not parallel

		v = (-s1y * (p0Pos[0] - p2Pos[0]) + s1x * (p0Pos[1] - p2Pos[1])) / (-s2x * s1y + s1x * s2y)
		w = (s2x * (p0Pos[1] - p2Pos[1]) - s2y * (p0Pos[0] - p2Pos[0])) / (-s2x * s1y + s1x * s2y)

	if v > 0 and v < 1 and w > 0 and w < 1: # segments intersect
		
		intersection = 1

	else: # they don't intersect

		intersection = 0

	return intersection


# ========== get intersection point function ==========

def intersect_point(p0, p1, edge):
	
	# Input variables:
	# 	p0 = start point of line segment
	#	p1 = end point of line segment
	#	edge = line segment to calculate intersection point with 
	# ======================================== #

    
	if p0 in edge: 
		return p0
	if p1 in edge: 
		return p1

	intersect_P = p0 + '_' + p1 + '_' + edge + '_intersection'
	intersect_ID = edge.p1.ID
	intersectNeighbors = [edge.p1, edge.p2]

	if edge.p1.uv[0] == edge.p2.uv[0]:
		if p0.uv[0] == p1.uv[0]:
			return None

		pslope = (p0.uv[1] - p1.uv[1]) / (p0.uv[0] - p1.uv[0])
		intersect_u = edge.p1.[0]
		intersect_v = pslope * (intersect_u - p0.uv[0]) + p0.uv[1]
		intersect_UV = (intersect_u,intersect_v,0)
		intersect_dist = dt.Vector(p0.uv[0] - intersect_u, p0.uv[1] - intersect_v, 0).length()
        
        return nodePoint(intersect_P, uv = intersect_UV, dist=intersect_dist, neighbors=intersectNeighbors, poly=intersect_P, ID=intersect_ID)

    	if p0.uv[0] == p1.uv[0]:
		eslope = (edge.p1.uv[1] - edge.p2.uv[1]) / (edge.p1.uv[0] - edge.p2.uv[0])
		intersect_u = p0.uv[0]
		intersect_v = eslope * (intersect_u - edge.p1.uv[0]) + edge.p1.uv[1]
		intersect_UV = (intersect_u,intersect_v,0)
		intersect_dist = dt.Vector(p0.uv[0] - intersect_u, p0.uv[1] - intersect_v, 0).length()

		return nodePoint(intersect_P, uv=intersect_UV, dist=intersect_dist, neighbors=intersectNeighbors, poly=intersect_P, ID=intersect_ID)

	pslope = (p0.uv[1] - p1.uv[1]) / (p0.uv[0] - p1.uv[0])
	eslope = (edge.p1.uv[1] - edge.p2.uv[1]) / (edge.p1.uv[0] - edge.p2.uv[0])

	if eslope == pslope:
        	return None

	intersect_u = (eslope * edge.p1.uv[0] - pslope * p0.uv[0] + p0.uv[1] - edge.p1.uv[1]) / (eslope - pslope)
	intersect_v = eslope * (intersect_u - edge.p1.uv[0]) + edge.p1.uv[1]
	intersect_UV = (intersect_u,intersect_v,0)
	intersect_dist = dt.Vector(p0.uv[0] - intersect_u, p0.uv[1] - intersect_v, 0).length()

	return nodePoint(intersect_P, uv = intersect_UV, dist=intersect_dist, neighbors=intersectNeighbors, poly=intersect_P, ID=intersect_ID)


# ========== angle direction function ==========

def ccw(A, B, C): 
	
	# Input variables:
	# 	A = first corner of triangle (2D coordinates are stored in uv attribute in Point class)
	#	B = second corner of triangle (2D coordinates are stored in uv attribute in Point class)
	#	C = third corner of triangle (2D coordinates are stored in uv attribute in Point class)
	# ======================================== #

	area = int(((B.uv[0] - A.uv[0]) * (C.uv[1] - A.uv[1]) - (B.uv[1] - A.uv[1]) * (C.uv[0] - A.uv[0]))*T)/T2
	
	# Return 1 if counter clockwise, -1 if clock wise, 0 if collinear 

	if area > 0: 
		return 1
	elif area < 0: 
		return -1
	else:
		return 0


# ========== shoot ray function ==========

def ShootRay(point,target,edgeSet)

	# Input variables:
	# 	point = origin point of ray
	#	target = target point of ray
	#	edgeSet = edges to be checked if they intersect ray
	# ======================================== #

	intersectP = []

	for edge in edgeSet:

		if intersect( point, target, edge ) == 1: 

			intersectP.append = intersect_point( point, target, edge )
			
			# might also need to check for turning points, see Hechenberger et al. 2020 RayScan lines 25-28


	if len(intersectP) > 0: # get closest intersection point
		
		intersectP.sort(key=lambda point:point.dist)
		return intersectP[0]
		
	else: # if no intersection found returns target

		return target


# ========== point in triangle function ==========

def PointInTriangle(p, p0, p1, p2): # check if point 'p' is within or outside the triangle (p0,p1,p2)
	
	# Input variables:
	# 	p = point to querry if it lies within triange (p0,p1,p2), (2D coordinates are stored in uv attribute in Point class)
	#	p0 = first corner of triangle (2D coordinates are stored in uv attribute in Point class)
	#	p1 = second corner of triangle (2D coordinates are stored in uv attribute in Point class)
	#	p2 = third corner of triangle (2D coordinates are stored in uv attribute in Point class)
	# ======================================== #
	
	s = (p0.uv[0] - p2.uv[0]) * (p.uv[1] - p2.uv[1]) - (p0.uv[1] - p2.uv[1]) * (p.uv[0] - p2.uv[0])
	t = (p1.uv[0] - p0.uv[0]) * (p.uv[1] - p0.uv[1]) - (p1.uv[1] - p0.uv[1]) * (p.uv[0] - p0.uv[0])
	
	if (s < 0) != (t < 0) and s != 0 and t != 0:
		return false

	d = (p2.uv[0] - p1.uv[0]) * (p.uv[1] - p1.uv[1]) - (p2.uv[1] - p1.uv[1]) * (p.uv[0] - p1.uv[0])
	return d == 0 or (d < 0) == (s + t <= 0)


# ========== scan function ========== 

def SCAN(u, I, accwDir=None, acwDir=None, points, edges)

	# Input variables:
	# 	u = starting point 
	#	I = target point and/or intersection point
	#	accwDir = normalised vector of the direction of the counter clockwise angle, i.e. accwDir = dt.vector(accw.uv-u.uv).normal(). Variable is optional and if not suplied is assumed to point into opposite direction from u to I and thus the arc being a half circle 
	#	acwDir = normalised vector of the direction of the clockwise angle, i.e. acwDir = dt.vector(acw.uv-u.uv).normal(). Variable is optional and if not suplied is assumed to point into opposite direction from u to I and thus the arc being a half circle
	#	points = set of all points within path finding problem
	#	edges = set of all edges within path finding problem
	# ======================================== #
	
	# Find CCW and CW turning points of intersection polygon mesh 'I.id' from point 'u' within arc defined by accw and acw
	# First, get all points within the arc sector described by 'accwDir' and 'acwDir'; second, get the points that are 
	# visible from u; third, calculate the angles for all visible points and sort them accoridingly to find turning points
	
	# get subset of points that are part of intersection polygon
	
	polygonSubset = []
	if I.poly = -1:
		polygonSubset = points
	else:
		for point in points:
			if point.poly == I.poly:
				polygonSubset.append(point)
	
	# further reduce point set based on arc sector defined by accwDir and acwDir
	
	if not accwDir: # check whether accwDir and acwDir have been supplied
		if not acwDir:
			pointsWAS = polygonSubset # if angle sector is not suplied arc sector is assumed to be a circle and all points in polygonSubSet are assumed to be within
		else:
			accwDir == [-1,0,0] # accwDir points into opposite direction from u to I resulting in a half circle on the ccw side
	else:
		if not acwDir:
			acwDir == [-1,0,0] # acwDir points into opposite direction from u to I resulting in a half circle on the cw side
			
	# check if pointsWAS exists, if it already does skip angle calculations
			
	if not pointsWAS: # if subset of points within angle sector has not yet been defined then continue
		
		angleAS = pm.atan2(acwDir[1], acwDir[0]) - pm.atan2(accwDir[1], accwDir[0]) # angle of the arc sector described by 

		accw = (u.uv+accwDir)*INF # define ccw triangle point 
		acw = (u.uv+acwDir)*INF # define cw triangle point

		# get points in arc sector described by 'accw' and 'acw' and  exclude all other ones

		if angleAS < PI and angleAS > 0 or anlgeAS > -PI and angleAS < 0: # check if angle sector is smaller than 180°
			
			#pointsWAS = [point for point in pointsSubset if PointInTriangle(point,u,accw,acw) == True] # get all points within angle sector
			
			pointsWAS=[]
			
			# if angleAS is smaller than 180° (PI) check within angle sector
			
			for point in pointsSubset: # get points within angle sector
				if PointInTriangle(point,u,accw,acw) == True: # check if point is within angle sector
					pointsWAS.append(point) # if so, add it to points within angle sector subset
				
		elif angleAS > PI and angleAS < 2*PI or anlgeAS < -PI and anlgeAS > -2*PI: # check if angle sector is bigger than 180°
			
			#pointsWAS = [point for point in pointsSubset if PointInTriangle(point,u,accw,acw) == False] # get all points outside angle sector
			
			pointsWAS=[]
			
			# if angleAS is bigger than 180° (PI) reverse angle sector, i.e. check outside angle sector
			
			for point in pointsSubset: # get points outside angle sector
				if PointInTriangle(point,u,accw,acw) == False: # check if point is outside angle sector
					pointsWAS.append(point) # if so, add it to points within angle sector subset
		else:
			pointsWAS = pointsSubset # angle is either 0 or 2*PI and all points should be in subset
	
	#sort visible points by angles
	
	iDir = dt.vector(I.uv-u.uv).normal()
	
	# Currently brute force, could be further sped up with Lee's scan alorigthm, however, since points hopefully already massively reduced, maybe less of an issue?
	 
	visiblePoints = pointsWAS
	
	for point in visiblePoints:
	 	
		pDir = dt.vector(point.uv-u.uv).normal()
		point.angle = pm.atan2(pDir[1], pDir[0]) - pm.atan2(iDir[1], iDir[0]) # get angle from u between point and intersection
			
	visiblePoints.sort(key=lambda point:point.angle) 
	
	# calculate angle only for visible points determined by Lee's algorithm # visible points from u in pointWAS
	#
	# visiblePoints = visible_points(u, pointsWAS, edges)
	
	for point in visiblePoints: 
		
		pDir = dt.vector(point.uv-u.uv).normal()
		point.angle = pm.atan2(pDir[1], pDir[0]) - pm.atan2(iDir[1], iDir[0]) # get angle between each point and intersection at point u

	visiblePoints.sort(key=lambda point:point.angle) 
	
	if visiblePoints[0].angle < 0: # find CCW turningpoint 
		turningPointCCW = visiblePoints[0]
	else:
		turningPointCCW = -1
	
	if visiblePoints[len(visiblePoints)-1].angle > 0:  # find CW turningpoint
		turningPointCW = visiblePoints[len(visiblePoints)-1]
	else:
		turningPointCCW = -1
	
	return [turningPointCCW,turningPointCW] # return turningpoints


# ========== find visible points Lee's scan algorithm ========== // heavy WIP

def visible_points(u, points, edges):
	
	# Input variables:
	# 	u = point from which points are checked if they are visible or not
	#	points = set of all points within path finding problem
	#	edges = set of all edges within path finding problem
	# ======================================== #
	
	points.sort(key=lambda p: (dt.Vector(u.uv.angle(dt.Vector(p.uv))), dt.Vector(u.pos.distanceTo(dt.Vector(p.pos)))))
	
	# Initialize open_edges with any intersecting edges on the half line from
	# point along the positive x-axis
	open_edges = OpenEdges()
	point_inf = Point(INF, point.y)
	for edge in edges:
		if point in edge: continue
		if edge_intersect(point, point_inf, edge):
			if on_segment(point, edge.p1, point_inf): continue
			if on_segment(point, edge.p2, point_inf): continue
			open_edges.insert(point, point_inf, edge)
	


# ========== A* search algorithm ==========

def astar(start, end, points, edges): #, EdgeCrossSet): # EdgeCrossSet probably not needed

	# Input variables:
	# 	start = 3D position of the starting point
	#	end = 3D position of the end point
	#	points = set of all points within path finding problem
	#	edges = set of all edges within path finding problem
	# ======================================== #
	
	# Create start and end node

	start_node = nodePoint('start_node', uv=(0,0,0), pos=start)
	start_node.g = start_node.h = start_node.f = 0
	
	end_node = nodePoint('end_node', uv=(1,0,0), pos=end)
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

			pathLength = current_node.g

			# path might not be needed

			# path = []
			# current = current_node
			#
			# while current is not None:
			#
			#    path.append(current)
			#    current = current.parent

			return [1, pathLength] #, path[::-1]] # Return reversed path

		# shoot ray from current_node to end_node
		
		hitPoint = ShootRay(current_node,end_node,edges)
		
		# check if end_node is visible
		
		if hitPoint == end_node: # end_node is visible from current_node
			
			hitPoint.parent = current_node
			children.append(hitPoint)
		
		else: # end_node is not visible from current_node, thus need to define accwDir and acwDir for scan
			
			if current_node.parent is None: #
				
				accwDir == acwDir == None
			
			else: # 
				
				# get parent_node of current_node and get previous direction

				parent_node = current_node.parent
				parentDir = dt.Vector(current_node.uv-parent_node.uv).normal()

				# get neighboring nodes of current_node and get their orientation in relation to the parent_node

				neighborOne_node = current_node.neighbor[0]
				neighborTwo_node = current_node.neighbor[1]
				neighborOneOrient = ccw(parent_node,current_node,neighborOne_node)
				neighborTwoOrient = ccw(parent_node,current_node,neighborTwo_node)
				neighborOneDir = dt.Vector(neighborOne_node.uv-current_node.uv).normal()
				neighborTwoDir = dt.Vector(neighborTwo_node.uv-current_node.uv).normal()

				# check if path was following edge of polygon in previous step 

				if parent_node == neighborOne_node: # neighbor one was previous step
					if neighborTwoOrient == CW:
						acwDir == neighborTwoDir
						accwDir == parentDir
					else:
						acwDir == parentDir
						accwDir == neighborTwoDir

				elif parent_node == neighborTwo_node: # neighbor two was previous step
					if neighberOneOrient = CW:
						acwDir == neighborOneDir
						accwDir == parentDir
					else:
						acwDir == parentDir
						accwDir == neighborOneDir

				else: # neither neighbor was previous step, therefore need to check which way around the obstacle the path has to wrap 
					
					end_nodeDir = dt.Vector(end_node.uv-current_node.uv).normal()
					neighborOneAngle = neighborOneDir.angle(end_nodeDir)
					neighborTwoAngle = neighborTwoDir.angle(end_nodeDir)

					if neighborOneOrient == CCW and neighborTwoOrient == CCW: 
						
						# both neighboring nodes are on the CCW side and the acw direction can, therefore, be restricted to the previous ray direction
						
						acwDir = parentDir
						
						# find which angle is smaller and thus going to be acw dir
						
						if neighborOneAngle < neighborTwoAngle:
							accwDir = neighborOneDir
						else:
							accwDir = neighborTwoDir
							
					elif neighborOneOrient == CW and neighborTwoOrient == CW:
																	   
						# both neighboring nodes are on the CW side and the accw direction can, therefore, be restricted to the previous ray direction

						accwDir = parentDir
						
						# find which angle is smaller and thus going to be acw dir
						
						if neighborOneAngle < neighborTwoAngle:
							acwDir = neighborOneDir
						else:
							acwDir = neighborTwoDir
			
			# find turningpoints on polygon hitPoint.poly
			
			turningPoints = SCAN(current_node, hitPoint, accwDir, acwDir, points, edges)
			
			for point in turningPoints:
				if not point == -1:
					point.parent = current_node
					children.append(tP)
		
		# Loop through children
		
		for child in children:

			# check if neighbor is in closed_list
			for closed_children in closed_list:
				if child == closed_child:
		    			continue
		
			tempG = current_node.g+dt.Vector(current_node.pos-child.pos).lenght()
			
			if child in open_list: # if neighbor is already in open_list compare g values
				if tempG < child.g:
					child.g = tempG
			else:
				child.g = tempG #double check this
				openlist.append(child)
			
			child.h = dt.vector(child.pos-end_node.pos).length
			child.f = child.h + child.g
		
	return [0, 0] #, 0] # no path found
		



################################################
# ========== MAYA specific functions ========= #
################################################


# ========== face area function ==========

def polyFacesTotArea( faces ): # Requires faces to be selected
	
	# Input variables:
	# 	faces = selected faces for which surface area is to be calculated
	# ======================================== #

	# triangulation of faces
	
	pm.polyTriangulate(faces)
	vertIndicesStr = pm.polyInfo(fv = True)
	pm.select(clear = True)
	
	# get number of faces
	
	faceNum = len(vertIndicesStr)

	faceTotArea = 0
	
	# area calculations

	for i in range(faceNum): # get ID of each triangle vertex

		vertIndices = vertIndicesStr[i].split()

		VtxA = prox + '.vtx[' + vertIndices[2] + ']'
		VtxB = prox + '.vtx[' + vertIndices[3] + ']'
		VtxC = prox + '.vtx[' + vertIndices[4] + ']'

		# querry position of each vertex and calculate distances between them

		VtxAPos = pm.xForm(VtxA, query=True, worldSpace=True, translation=True)
		VtxBPos = pm.xForm(VtxB, query=True, worldSpace=True, translation=True)
		VtxCPos = pm.xForm(VtxC, query=True, worldSpace=True, translation=True)

		DistA = dt.Vector(VtxBPos-VtxCPos).length()
		DistB = dt.Vector(VtxAPos-VtxCPos).length()
		DistC = dt.Vector(VtxAPos-VtxBPos).length()

		# calculate area each triangle and sum it up

		s = (DistA + DistB + DistC)/2
		faceTotArea += pm.sqrt(s * (s-DistA) * (s-DistB) * (s-$istC))

	return faceTotArea


# ========== polygon slice approximation function ==========

def polyApproxSlice( oPos, cutDir, uDir, vDir, mesh, res ):
	
	# Input variables:
	# 	oPos = origin position
	# 	cutDir = direction of cutting plane
	# 	uDir = direction uf U axis on cutting plane
	# 	vDir = direction of V axis on cutting plane
	# 	mesh = mesh to be cut
	# 	res = resolution for mesh slice approximation
	# ======================================== #
	
	# get number of faces

	numFace = pm.polyEvaluate(f=True, mesh)
	numF = numProxFace[0]-1
	meshF = mesh + '.f[0:' + numF + ']'

	# cut mesh

	pm.polyCut( meshF, pc= (oPos[0],oPos[1],oPos[2]), ro = (cutDir[0],cutDir[1],cutDir[2]), ps = (1,1), ef = 1, eo = (0,0,0))

	# get difference in face numbers of pre and post cutting

	numPreSlice = pm.polyEvaluate(f=True, mesh)

	pm.select(mesh)
	pm.polyCloseBorder()

	numPostSlice = pm.polyEvaluate(f=True, mesh)

	pm.select(clear=True)

	faceDiff = (numPostSlice - numPreSlice)/2 

	# define temporary variables

	vtcs = []
	edges = []
	allEdges = []

	# get vertices and edges of mesh slice

	for i in range(faceDiff):

	newFaces[i] = mesh + '.f[' + (numPostSlice[0]-i-1) + ']'
	tempCRV = mesh + '_CRV_' + i
	tempPlane = mesh + '_plane_' + i
	tempSliceEdges = pm.polyListComponentConversion(newFaces[i], ff = True, te = True )

	# get slice area and translate it into resolution of curve circumscribing slice

	pm.select(newFaces[i])
	sliceArea = polyFacesTotArea(newFaces[i])
	log = pm.log(sliceArea)
	resolution = int(pm.ceil(res + (res *log)))

	if resolution < 4:
		resolution = 4

	# approximate slice surface and reduce number of vertices and edges to a more reasonable number based on slice area

	pm.select(tempSliceEdges)
	pm.polyToCurve(form = 2, degree = 1, conformToSmoothMeshPreview = 0, n = tempCRV)
	pm.select(clear=True)

	# rebuild curve and approximate slice

	pm.rebuildCurve(tempCRV, ch = 1, rpo = 1, rt = 0, end = 1, kr = 0, kcp = 0, kep = 1, kt = 1, s = resolution, d = 1 tol = 1e-08)
	pm.nurbsToPolygonsPref(pt = 1, pc = 1)
	pm.planarSrf(tempCRV, ch = 1, d = 1, ko = 0, tol = 1e-01, rn = 0, po = 1, n = tempPlane)

	# add curve and plane to temporary group

	if not pm.objExists('TempGRP'): # check if temporary group exists
		tempGRP = 'TempGRP' # if it doesn't create empty group
		pm.group(em = True, n=tempGRP)

	pm.parent(tempCRV, tempGRP)
	pm.parent(tempPlane, tempGRP)

	# get vertices and edges from new slice

	tempPlaneF = tempPlane + '.f[0]'
	tempVtcs = pm.polyListComponentConversion(tempPlaneF, ff = True, tv = True)
	tVtcs = pm.filterExpand(tempVtcs, sm = 31)
	vtcs += tVtcs

	for j in range(len(tVtcs)):

		# extract vertex index and get name for node

		tVtx = tVtcs[j].split('.[ | ]')
		pad = '{:05d}'.format(tVtx[2])
		nNode = tVtx[0] + '_node_' + pad

		# calculate UV values

		vtcPos = pm.xform(tVtcs[j], query=True, worldSpace=True, translation=True)
		nodePos = (vtcPos[0],vtcPos[1],vtcPos[2])
		pOffset = dt.Vector(oPos[0] - vtcPos[0], oPos[1] - vtcPos[1], oPos[2] - vtcPos[2])
		u = pOffset.dot(uDir)
		v = pOffset.dot(vDir)
		nodeUV = (u,v,0)

		# get neighbors of each vertex
		
		vtxNeighbors = pm.MeshVertex(tVtcs[j]).connectedVertices()
		
		for k in range(len(vtxNeighbors)):
			tVtxNeighbor= vtxNeighbors[k].split('.[ | ]')
			padNeighbor = '{:05d}'.format(tVtxNeighbor[2])
			nNeighbors[k] = tVtxNeighbor[0] + '_node_' + padNeighbor
		 
		nodePoint(nNode, pos=nodePos, uv=nodeUV, neighbors=nNeighbors, poly=tempPlane, ID= pad)

		nodePoint(nNode, pos=nodePos, uv=nodeUV, poly=tempPlane, ID= pad)

    	tempEdges = pm.polyListComponentConversion(tempPlaneF, ff = True, te = True)
    	tEdges = pm.filterExpand(tempEdges, sm = 32)
    	edges += tEdges

    	for j in range(len(tEdges)):

		tEdgeVtcs = pm.polyListComponentConversion(tEdges[j], fe = True, tv = True)
		tEdge = tEdges[j].split('.[ | ]')
		pad0 = '{:05d}'.format(tEdge[2])
		nEdge = tEdge[0] + '_edge_' + pad0

		if len(tEdgeVtcs) == 1:

			tempEdgeVtcs = pm.filterExpand(tEdgeVtcs[0], sm = 31)
			tVtx1 = tempEdgeVtcs[0].split('.[ | ]')
			tVtx2 = tempEdgeVtcs[1].split('.[ | ]')
			pad1 = '{:05d}'.format(tVtx1[2])
			pad2 = '{:05d}'.format(tVtx2[2])

			edgeP1 = tVtx1[0] + '_node_' + pad1
			edgeP2 = tVtx2[0] + '_node_' + pad2

			nodeEdge(nEdge, p1=edgeP1, p2=edgeP2)

		else:

			tVtx1 = tEdgeVtcs[0].split('.[ | ]')
			tVtx2 = tEdgeVtcs[1].split('.[ | ]')
			pad1 = '{:05d}'.format(tVtx1[2])
			pad2 = '{:05d}'.format(tVtx2[2])

			edgeP1 = tVtx1[0] + '_node_' + pad1
			edgeP2 = tVtx1[0] + '_node_' + pad2

			nodeEdge(nEdge, p1=edgeP1, p2=edgeP2)

    	pm.select(tempPlaneF)
    	pm.polyTriangulate(tempPlaneF)
    	tempFaces = pm.ls( selection=True )
    	pm.select(clear=True)

    	tempAllEdges = pm.polyListComponentConversion(tempFaces, ff = True, te = True)
    	tAllEdges = pm.filterExpand(tempAllEdges, sm = 32)
    	allEdges += tAllEdges

    	for j in range(len(tAllEdges)):

		tAllEdgeVtcs = pm.polyListComponentConversion(tAllEdges[j], fe = True, tv = True)
		tAllEdge = tAllEdges[j].split('.[ | ]')
		pad0 = '{:05d}'.format(tAllEdge[2])
		nAllEdge = tAllEdge[0] + '_AllEdge_' + pad0

		if len(tAllEdgeVtcs) == 1:

			tempAllEdgeVtcs = pm.filterExpand(tAllEdgeVtcs[0], sm = 31)
			tVtx1 = tempAllEdgeVtcs[0].split('.[ | ]')
			tVtx2 = tempAllEdgeVtcs[1].split('.[ | ]')
			pad1 = '{:05d}'.format(tVtx1[2])
			pad2 = '{:05d}'.format(tVtx2[2])

			AllEdgeP1 = tVtx1[0] + '_node_' + pad1
			AllEdgeP2 = tVtx2[0] + '_node_' + pad2

			nodeEdge(nAllEdge, p1=AllEdgeP1, p2=AllEdgeP2)

		else:

			tVtx1 = tAllEdgeVtcs[0].split('.[ | ]')
			tVtx2 = tAllEdgeVtcs[1].split('.[ | ]')
			pad1 = '{:05d}'.format(tVtx1[2])
			pad2 = '{:05d}'.format(tVtx2[2])

			AllEdgeP1 = tVtx1[0] + '_node_' + pad1
			AllEdgeP2 = tVtx1[0] + '_node_' + pad2

			nodeEdge(nAllEdge, p1=AllEdgeP1, p2=AllEdgeP2)

    return [vtcs, edges, allEdges]


# ========== ligament length function ==========

def ligLength (origin, insertion, jointCentre, proxMesh, distMesh, resolution )

	# Input variables:
	# 	origin = name of ligament origin (represented by object, e.g. locator, in Maya scene)
	# 	insertion = name of ligament insertion (represented by object, e.g. locator, in Maya scene)
	# 	jointCentre = name of joint centre (represented by object, e.g. joint or locator, in Maya scene)
	# 	proxMesh = name of proximal bone mesh to which ligament origin attaches
	# 	distMesh = name of distal bone mesh to which ligament insertion attaches
	# 	res = resolution for mesh slice approximation
	# ======================================== #
	
	# data house keeping

	proxT = proxMesh.split('|')
	nProxClean = proxT[len(proxT)-1]
	nProxCopy = nProxClean +'_duplicate'
	prox = 'ProxBoneMesh'
	pm.duplicate ('proxMesh', n=prox)
	pm.rename('proxMesh', 'prox')

	distT = distMesh.split('|')
	nDistClean = distT[len(distT)-1]
	nDistCopy = ndistClean +'_duplicate'
	dist = 'dDstBoneMesh'
	pm.duplicate ('distMesh', n=dist)
	pm.rename('distMesh', 'dist')

	# get positions of points of interest

	oPos = pm.xform(origin, query=True, worldSpace=True, translation=True)
	iPos = pm.xform(insertion, query=True, worldSpace=True, translation=True)
	jPos = pm.xform(jointCentre, query=True, worldSpace=True, translation=True)

	# calculate vectors from origin to insertion

	LigDir = dt.Vector(oPos[0] - iPos[0], oPos[1] - iPos[1], oPos[2] - iPos[2])
	JointDir = dt.Vector(oPos[0] - jPos[0], oPos[1] - jPos[1], oPos[2] - jPos[2])
	Offset = LigDir.length()

	# calculate cut plane direction

	uCross = LigDir.cross(JointDir).normal()
	vCross = LigDir.cross(uCross).normal()
	cutAim = pm.angleBetween( euler=True, v1=(0.0, 0.0, 1.0), v2=(uCross[0], uCross[1], uCross[2]) )

	# normalize vectors by distance between origin and insertion

	LigDirNorm = LigDir.nomal()/Offset
	vCrossNorm = vCross/Offset

	# get obstacle slices, their vertices and edges

	proxSlice = polyApproxSlice( oPos, cutAim, LigDirNorm, vCrossNorm, prox, resolution ) # approximated slice of proximal bone mesh
	distSlice = polyApproxSlice( oPos, cutAim, LigDirNorm, vCrossNorm, dist, resolution ) # approximated slice of distal bone mesh

	# combine obstacle edge and vertex arrays

	VtxSet = proxSlice[0]
	EdgeSet = proxSlice[1]
	AllEdgeSet = proxSlice[2]

	VtxSet += distSlice[0]
	EdgeSet = distSlice[1]
	AllEdgeSet += distSlice[2]

	# get edges crossing the polygon

	#EdgeCrossSet = [Edge for Edge in AllEdgeSet if Edge not in EdgeSet]

	# run A* search

	ligLength = astar(oPos, iPos, VtxSet, AllEdgeSet) #, EdgeCrossSet) might not be needed

	# clean up

	if pm.objExists('TempGRP'): # check if temporary group exists
    		pm.delete('TempGRP')

	pm.delete(prox, dist)
	pm.rename(nProxCopy, proxMesh)
	pm.rename(nDistCopy, distMesh)

	return ligLength
