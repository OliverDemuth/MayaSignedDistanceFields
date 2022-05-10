#	ligLength.v.0.1.py 
#
#   This script calculates the shortest path (length) of a ligament from origin to
#   insertion wrapping around the proximal and distal bone meshes. It generates an
#   on-the-fly visibility graph embedded in an A* search algorithm to find the 
#   shortest distance. 
#
#   Written by Oliver Demuth 19.04.2022
#   Last updated 06.05.2022 - Oliver Demuth
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
    __slots__ = ('parent', 'pos', 'uv', 'g', 'h', 'f', 'polyID')

    def __init__(self, parent=None, pos=None, uv=None, dist = 0, g=0, h=0, f=0, neighbors=None, poly=None, ID=-1):
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



# ========== face area function ==========

def polyFacesTotArea( faces ): # Requires faces to be selected.

	pm.polyTriangulate(faces)
	vertIndicesStr = pm.polyInfo(fv = True)
	pm.select(clear = True)

	faceNum = len(vertIndicesStr)

	faceTotArea = 0

	for i in range(faceNum): # get position of each triangle vertex

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

		# calculate area and center position of each triangle

		s = (DistA + DistB + DistC)/2
		faceArea[i] = pm.sqrt(s * (s-DistA) * (s-DistB) * (s-$istC))
		faceTotArea += faceArea[i]

	return faceTotArea


# ========== polygon slice approximation function ==========

def polyApproxSlice( oPos, cutDir, mesh, res ):
	
	# get number of faces

	numFace = pm.polyEvaluate(f=True, mesh)
	numF = numProxFace[0]-1
	meshF = mesh + '.f[0:' + $numF + ']'

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

    vtcs = None
    edges = None
    allEdges = None

    # get vertices and edges of mesh slice

    for i in range(faceDiff):

    	newFaces[i] = mesh + '.f[' + (numPostSlice[0]-i-1) + ']'
    	tempCRV = mesh + '_CRV_' + i
    	tempPlane = mesh + '_plane_' + i # <- needs to be fed into nodeEdge.polyID
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
			u = pOffset.dot(LigDirNorm)
			v = pOffset.dot(vCrossNorm)
			nodeUV = (u,v,0)
			
			# the following might not be needed:
			#
			# vtxNeighbors = pm.MeshVertex(tVtcs[j]).connectedVertices()
			#
			# for k in range(len(vtxNeighbors)):
			#	tVtxNeighbor= vtxNeighbors[k].split('.[ | ]')
			#	padNeighbor = '{:05d}'.format(tVtxNeighbor[2])
			#	nNeighbors[k] = tVtxNeighbor[0] + '_node_' + padNeighbor
			# 
			#nodePoint(nNode, pos=nodePos, uv=nodeUV, neighbors=nNeighbors, poly=tempPlane, ID= pad)
			
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


# ========== check intersection function ==========

def intersect( p0, p1, edge ):
    
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
        
        return nodePoint(intersect_P, uv = intersect_UV, dist=intersect_dist, neighbors=intersectNeighbors, poly=intersect_P, ID=intersect_ID)

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

	counter = 0
	intersectP = []

	for edge in edgeSet:

		if intersect( s, t, edgeSet[i] ) == 1: 

			intersectP[counter] = intersect_point(s, t, edgeSet[i])
			counter += 1

			# might also need to check for turning points, see Hechenberger et al. 2020 RayScan lines 25-28

	intersectP.sort(key = dist)

	# if no intersection found returns target

	if counter > 0:
		return intersectP[0]
	else:
		return target


# ========== scan function ==========

def isOnPolygon(point,polygon): # check if point is part of polygon
    return bool(point.poly == polygon)

def SCAN(u, p, I, accw, acw, points)  # Find CCW and CW turningpoints of intersection polygon mesh 'p' from point 'u' within arc defined by accw and acw

	# this appears to be rather brute force-ish as all angles of the pointSubset are calculated and then checked against the projection field defined by accw and acw
	
	#get subset of points that lay on intersection polygon
	
	pointsSubset = [point for point in points if isOnPolygon(point,p)]
	
	iDir = dt.vector(I.uv-u.uv).normal()
	pointAnlgeSet = []
	
	#sort by angles
	
	for point in pointsSubset:
		
		# CCW = 1
		# CW = -1
		
		pDir = dt.vector(point.uv-u.uv).normal()
		angle = CCW(u,I,point)*pm.dot(iDir,pDir) # does this work for angles 90° or does the sign become an issue??
		
		if angle <= accw or angle >= acw
			point.angle = sign*angle
			pointAngleSet.append(point)
		else
			point.angle = None
		
	pointAngleSet.sort(key=lambda x:x.angle) 
	
	if pointAngleSet[0].angle < 0: # find CW turningpoint 
		turningPointCW = pointAngleSet[0]
	else:
		turningPointCW = None
	
	if pointAngleSet[len(pointsSubsetCCW)-1].angle > 0:  # find CCW turningpoint
		turningPointCCW = pointsSubsetCCW[len(pointsSubsetCCW)-1]
	else:
		turningPointCCW = None
	
	return [turningPointCW,turningPointCCW] # return turningpoints


# ========== A* search algorithm ========== // WIP

def astar(origPos, insPos, VtxSet, AllEdgeSet, EdgeCrossSet):

	# Create start and end node

    start_node = Node(None, 'origin_node')
    start_node.g = start_node.h = start_node.f = 0
    start_node.uv = [0,0,0]
    start_node.pos = origPos

    end_node = Node(None, 'insertion_node')
    end_node.g = end_node.h = end_node.f = 0
    end_node.uv = [1,0,0]
    end_node.pos = insPos

    # add nodes to VtxSet

	VtxSet[len(VtxSet)] = 'origin_node'
	VtxSet[len(VtxSet)] = 'insertion_node'

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

        # Found the goal

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

        Points = VtxSet

        Points.sort(key=lambda p: (dt.Vector(point.uv.angle(dt.Vector(p.uv))), dt.Vector(point.pos.distanceTo(dt.Vector(p.pos)))))


        HitPoint = ShootRay(current,end_node,AllEdgeSet)
        	if HitPoint == end_node:


        # WIP below


        for point in Points:

		    # start ray scanning

		    open_edges = OpenEdges()
		    point_inf = Point(INF, point.y)

		    for edge in edges:
		        if point in edge: continue
		        if intersect(point, point_inf, edge):
	            

	    visible = []
	    prev = None
	    prev_visible = None
	    for p in points:
	        if p == point: continue
	        if scan == 'half' and dt.Vector(point.uv.angle(dt.Vector(p.uv))) > pi: break

        # MISSING PARTS 
        #
        # -> on the fly visability graph, 
        # draw inspiration from RayScan and/or Lee's scan algorithm etc. 

    return [0, 0, 0] # no path found


# ========== ligament length function ==========

def ligLength (origin, insertion, jointCentre, proxMesh, distMesh, resolution )

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

	proxSlice = polyApproxSlice( oPos, cutAim, prox, resolution ) # approximated slice of proximal bone mesh
	distSlice = polyApproxSlice( oPos, cutAim, dist, resolution ) # approximated slice of distal bone mesh

	# combine obstacle edge and vertex arrays

	VtxSet = proxSlice[0]
	EdgeSet = proxSlice[1]
	AllEdgeSet = proxSlice[2]

	VtxSet += distSlice[0]
	EdgeSet = distSlice[1]
	AllEdgeSet += distSlice[2]

	# get edges crossing the polygon

	EdgeCrossSet = [Edge for Edge in AllEdgeSet if Edge not in EdgeSet]

	# run A* search

	ligLength = astar(oPos, iPos, VtxSet, AllEdgeSet, EdgeCrossSet)

	# clean up

	if pm.objExists('TempGRP'): # check if temporary group exists
    	pm.delete('TempGRP')

    pm.delete(prox, dist)
    pm.rename(nProxCopy, proxMesh)
    pm.rename(nDistCopy, distMesh)

    return ligLength

























