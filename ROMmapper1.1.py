#	ROMmapper1.1.py
#
#	This script simulates contact-based positions from meshes and across a set of 
#	rotational poses. It is an implementation of the Marai et al., 2006. approach for 
#	Autodesk Maya. It creates signed distance fields for the proximal and distal bone 
#	meshes, which are then used to caculate intersections between the meshes and the
#	distance between them. 
#
#	Written by Oliver Demuth 
#	Last updated 13.11.2024 - Oliver Demuth
#
#	SYNOPSIS:
#
#		INPUT params:
#			string  jointName:		Name of the joint centre, i.e. the name of a locator or joint (e.g., 'myJoint' if following the ROM mapping protocol of Manafzadeh & Padian 2018)
#			string  meshes:			Name(s) of the bone meshes, i.e., several individual meshes (e.g., in the form of ['prox_mesh','dist_mesh'])
#			string	artSurfMeshes:	Name(s) of the articular surface meshes, i.e., several individual meshes (e.g., in the form of ['prox_art_surf','dist_art_surf'])
#			int		gridSubdiv:		Integer value for the subdivision of the cube, i.e., number of grid points per axis (e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points)
#			float	gridSize:		Float value indicating the size of the cubic grid, i.e., the length of a side (e.g., 10 will result ing a cubic grid with the dimensions 10 x 10 x 10)
#			float	thickness:		Float value indicating the thickness value which correlates with the joint spacing
#			
#		RETURN params:
#			array 	coords:			Return value is a an array of 3D coordinates of the distal bone relative to the proximal one
#			boolean	viable:			Return value is a boolean whether the rotational poses are viable or inviable (i.e, intersection or disarticulation of the bone meshes)
#			object 	results: 		Return value is an object of the scipy.optimize.OptimizeResult class. It represents the output of the scipy.optimize.minimize() function
#
#
#	IMPORTANT notes:
#		
#	(1) Meshes need realtively uniform face areas, otherwise large faces might skew 
#		vertex normals in their direction. It is, therefore, important to extrude 
#		edges around large faces prior to closing the hole at edges with otherwise 
#		acute angles to circumvent this issue, e.g., if meshes have been cut to reduce
#		polycount, prior to executing the Python scripts.
#
#	(2) For each bone (*) two sets of meshes are required, the articular surface and the
#		bone mesh
#
#	(3)	This script requires several modules for Python, see README file. Make sure to
#		have the following external modules installed for the mayapy application:
#
#			- 'numpy' 		NumPy:				https://numpy.org/about/
#			- 'scipy'		SciPy:				https://scipy.org/about/
#			- 'tricubic' 	Daniel Guterding: 	https://github.com/danielguterding/pytricubic
#				
#		For further information regarding them, please check the website(s) referenced 
#		above.


# ========== load modules ==========

import maya.api.OpenMaya as om
import maya.cmds as cmds
import numpy as np
import scipy as sp

from tricubic import tricubic



################################################
# ========== Maya specific functions ========= #
################################################


# ========== translation optimisation function ==========
def optimisePosition(proxCoords, distCoords, ipProx, ipDist, gridRotMat, rotMat, thickness):

	# Input variables:
	#	proxCoords = Coordinates of proximal articular surface vertices
	#	distCoords = Coordinates of distal bone mesh
	#	ipProx = proximal signed distance field in tricubic form
	#	ipDist = distal signed distance field in tricubic form
	#	gridRotMat = rotation matrix of default cubic grid
	#	rotMat = array with the transformation matrices of the joint and its parent
	#	thickness = thickness measure correlated with joint spacing
	# ======================================== #

	# optimise translation for specific rotational pose

	results = posOptMin(proxCoords, distCoords, ipProx, ipDist, gridRotMat, rotMat, thickness)
	
	transMat = getTransMat(rotMat[1],getPosInOS(rotMat[2].inverse(),results.x))
	
	signDist = []

	# check for disarticulation 

	for i in range(len(proxCoords)):

		# transform relative grid position into default cubic grid space for tricubic interpolation using rotation matrices

		relWSProx = getPosInWS(rotMat[0], proxCoords[i])
		relPosProx = getPosInOS(gridRotMat, getPosInOS(transMat,relWSProx))
		
		# calculate distance 

		tempSignDist = ipDist.ip([relPosProx[0], relPosProx[1], relPosProx[2]]) # get smaller of the two values
		signDist.append(tempSignDist)

	avgdist = np.mean(signDist)
	
	# if disarticulated or any points penetrate meshes the position becomes inviable

	if avgdist < (1.05 * thickness) and all(n > 0 for n in signDist):
		viable = 1
	else:
		viable = 0

	# gather results

	return results.x, viable, results


# ========== signed distance field per joint function ==========

def sigDistField(jointName, meshes, subdivision, size):

	# Input variables:
	#	jointName = name of joint centre (represented by object, e.g. joint or locator, in Maya scene)
	#	meshes = name(s) of the mesh(es) for which the signed distance field is calculated
	#	subdivision = number of elements per axis, e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points
	#	size = grid size of the cubic grid
	# ======================================== #

	# get joint centre position

	jPos = getWSPos(jointName)
	jDag = dagObjFromName(jointName)[1]
	jInclTransMat = om.MTransformationMatrix(jDag.inclusiveMatrix()) # world transformation matrix of joint
	jExclTransMat = om.MTransformationMatrix(jDag.exclusiveMatrix()) # world transformation matrix of parent of joint

	# normalize matrices by size

	jInclTransMat.setScale([size,size,size],om.MSpace.kWorld) # set scale in world space
	jExclTransMat.setScale([size,size,size],om.MSpace.kWorld) # set scale in world space

	# get rotation matrices

	rotMat = []
	rotMat.append(om.MMatrix(jExclTransMat.asMatrix())) # parent rotMat (prox)
	rotMat.append(om.MMatrix(jInclTransMat.asMatrix())) # child rotMat (dist)

	# create variables

	sigDistances = []
	localPoints = []
	worldPoints = []

	# cycle through all meshes

	if len(meshes) > 2:
		error('Too many meshes in array. Please specify only the distal and proximal bone mesh in the mesh array.')
	elif len(meshes) < 2:
		error('Too few meshes specified. Please specify TWO meshes in the mesh array.')
	
	for j,mesh in enumerate(meshes):
		meshSigDist, osPoints, wsPoints = sigDistMesh(mesh, rotMat[j], subdivision) # get signed distance field for each mesh
		sigDistances.append(np.array(meshSigDist).reshape(subdivision + 1, subdivision + 1, subdivision + 1))
		localPoints = osPoints  # osPoints are identical for both cubic grids
		worldPoints.append(wsPoints)
	
	return sigDistances, localPoints, worldPoints, rotMat


# ========== signed distance field per mesh function ==========

def sigDistMesh(mesh, rotMat, subdivision):

	# Input variables:
	#	mesh = name of the mesh for which the signed distance field is calculated
	#	rotMat = transformation matrix of parent (joint) of the mesh
	#	subdivision = number of elements per axis, e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points
	# ======================================== #

	# get dag paths

	dag = dagObjFromName(mesh)[1]

	# create MObject

	shape = dag.extendToShape()
	mObj = shape.node()
	
	# get the meshs transformation matrix

	meshMat = om.MMatrix(dag.inclusiveMatrix())

	# create the intersector

	polyIntersect = om.MMeshIntersector()
	polyIntersect.create(mObj,meshMat)
	ptON = om.MPointOnMesh()

	# create 3D grid where all axes are 1.5 times the size

	elements = np.linspace(-1.5, 1.5, num = subdivision + 1, endpoint=True, dtype=float)
	
	points = [[i, j, k] for i in elements  # length along x
						for j in elements  # length along y
						for k in elements] # length along z

	# go through grid points and calculate signed distance for each of them

	signedDist = []
	gridPoints = []

	for point in points:

		# get 3D position of grid points 

		gridPoint = om.MPoint(getPosInWS(rotMat, point))
		gridPoints.append(gridPoint)
		
		# get grid point in local coordinate system of mesh

		localPoint = getPosInOS(meshMat, gridPoint)
		
		# find closest points on mesh and get their distance to the grid points
							  
		ptON = polyIntersect.getClosestPoint(gridPoint) # get point on mesh
		ptNorm = om.MVector(ptON.normal)  # get normal
		
		# get vector from localPoint to ptON

		diff = om.MVector(ptON.point.x - localPoint[0], 
						  ptON.point.y - localPoint[1],
						  ptON.point.z - localPoint[2])

		# get distance from localPoint to ptON    
	
		dist = diff.length()

		# calculate dot product between the normal at ptON and vector to check if point is inside or outside of mesh
		
		dot = ptNorm.normal() * diff.normal() 

		# get sign for distance

		if dot >= 0: # point is inside mesh
			signedDist.append(dist * -1)
		else: # point is outside of mesh
			signedDist.append(dist)

	return signedDist, points, gridPoints


# ========== get vertices position in reference frame ==========

def relVtcPos(mesh, rotMat):

	# Input variables:
	#	mesh = name of the mesh for which the signed distance field is calculated
	#	rotMat = transformation matrix of parent (joint) of the mesh
	# ======================================== #

	# get dag paths

	dag = dagObjFromName(mesh)[1]

	# create mesh object

	MFnMesh = om.MFnMesh(dag)

	# go through vertices and store vertex coordinates

	vertexPos = []

	for i in range(MFnMesh.numVertices):

		worldPos = MFnMesh.getPoint(i, space = om.MSpace.kWorld) # get world position of vertex
		vertexPos.append(getPosInOS(rotMat, worldPos))

	return vertexPos


# ========== get ws pos ==========

def getWSPos(name):

	# Input variables:
	#	name = string representing object name
	# ======================================== #
	
	dag = dagObjFromName(name)[1]

	translate = om.MTransformationMatrix(dag.inclusiveMatrix()).translation(om.MSpace.kWorld) # extract translation in world space from transformation matrix (3x faster than xform)
	
	return translate


# ========== get point in world space function ==========	

def getPosInWS(rotMat,point):

	# Input variables:
	#	rotMat = rotation matrix
	#	point = 3D coordinates (translation) of point in object space (i.e., relative to parent)
	# ======================================== #

	localMat = om.MMatrix(((1, 0, 0, 0),
			    		   (0, 1, 0, 0),
			    		   (0, 0, 1, 0),
			    		   (point[0], point[1], point[2], 1))) # position of point

	worldPos = localMat * rotMat

	return om.MVector(round(worldPos[12],10), round(worldPos[13],10), round(worldPos[14],10)) # round to remove floating point errors


# ========== get point in object space function ==========	
	
def getPosInOS(rotMat, point):

	# Input variables:
	#	rotMat = rotation matrix
	#	point = 3D coordinates (translation) of point in world space
	# ======================================== #

	ptWS = om.MMatrix(((1, 0, 0, 0),
					   (0, 1, 0, 0),
					   (0, 0, 1, 0),
					   (point[0], point[1], point[2], 1))) # world space matrix of point

	ptPosOS = ptWS * rotMat.inverse() # multiply with inverse of rotation matrix

	return om.MVector(round(ptPosOS[12],10), round(ptPosOS[13],10), round(ptPosOS[14],10)) # round to remove floating point errors


# ========== get add translation to rotmat function ==========	
	
def getTransMat(rotMat, point):

	# Input variables:
	#	rotMat = rotation matrix
	#	point = 3D coordinates (translation) of point in world space
	# ======================================== #

	trans = om.MMatrix(((rotMat[0], rotMat[1], rotMat[2], 0),
						(rotMat[4], rotMat[5], rotMat[6], 0),
						(rotMat[8], rotMat[9], rotMat[10], 0),
						(point[0], point[1], point[2], 1))) # object space coordinates of point

	return trans


# ========== get dag path function ==========

def dagObjFromName(name):

	# Input variables:
	#	name = string representing object name
	# ======================================== #
	
	sel = om.MSelectionList()
	sel.add(name)

	dag = sel.getDagPath(0)
	mobj = sel.getDependNode(0)

	return mobj, dag


# ========== get mean distance from center ==========

def meanRad(mesh):

	# Input variables:
	#	mesh = name of the mesh in scene
	# ======================================== #

	dag = dagObjFromName(mesh)[1]

	# get mesh object and center positon

	MFnMesh = om.MFnMesh(dag)
	centerPos = meshCenter(mesh)

	# loop through each vertex and get distance to center

	vtcDist = []

	for i in range(MFnMesh.numVertices):

		worldPos = MFnMesh.getPoint(i, space = om.MSpace.kWorld)  # get world position of vertex
		vtcDist.append(om.MVector(worldPos[0] - centerPos[0], 
								  worldPos[1] - centerPos[1],
								  worldPos[2] - centerPos[2]).length())
			
	meanRad = np.mean(vtcDist)
	
	return meanRad


# ========== get bounding box center of mesh ==========

def meshCenter(mesh):

	# Input variables:
	#	mesh = name of the mesh in scene
	# ======================================== #

	# get dag paths

	dag = dagObjFromName(mesh)[1]
	shape = dag.extendToShape()

	transMat = om.MMatrix(dag.inclusiveMatrix()) 

	# get mesh object

	fnMesh = om.MFnMesh(shape)

	# calculate bounding box center in world space coordinates

	center = getPosInWS(transMat,fnMesh.boundingBox.center)

	return center



################################################
# ============ optimiser functions =========== #
################################################

def posOptMin(proxCoords, distCoords, ipProx, ipDist, gridRotMat, rotMat, thickness):

	# Input variables:
	#	proxCoords = Coordinates of proximal articular surface vertices
	#	distCoords = Coordinates of distal articular surface vertices
	#	ipProx = proximal signed distance field in tricubic form
	#	ipDist = distal signed distance field in tricubic form
	#	gridRotMat = rotation matrix of default cubic grid
	#	rotMat = array with the transformation matrices of the joint and its parent
	#	thickness = thickness measure correlated with joint spacing
	# ======================================== #

	# define initual guess condition

	initial_guess = np.zeros(3)

	# create tuple for arguments passed to both constraints and cost functions

	arguments = (proxCoords, distCoords, ipProx, ipDist, rotMat, gridRotMat, thickness)

	# set constraints functions

	cons = ({'type': 'ineq',			# set type to inequality, which means that it is to be non-negative
			 'fun': cons_fun,	# set constraint function
			 'args': arguments})		# pass arguments to constrain function

	# set bounds

	boundsList = [(-10,10)] * (len(initial_guess)) # set y and z coordinate boundaries to 10 times euclidean distance between  origin and insertion
	bnds = tuple(boundsList)

	# set options

	options = {"maxiter": 15} # if it doesn't solve within 12 iterations it usually won't solve

	# optimization using SLSQP

	res = sp.optimize.minimize(cost_fun, initial_guess, args = arguments, bounds = bnds, method = 'SLSQP', constraints = cons, options = options)

	# get results: res.X = joint centre point coordinates, res.fun = mean articular distance

	return res


# ========== signed distance field constraint function ==========

def cons_fun(params, proxCoords, distCoords, ipProx, ipDist, rotMat, gridRotMat, thickness):

	# Input variables:
	#	params = array of X, Y and Z coordinates of distal element position
	#	proxCoords = Coordinates of proximal articular surface vertices
	#	distCoords = Coordinates of distal articular surface vertices
	#	ipProx = tricubic interpolation function from tricubic.tricubic() for the signed distance data on the proximal cubic grid
	#	ipDist = tricubic interpolation function from tricubic.tricubic() for the signed distance data on the distal cubic grid
	#	rotMat = array with the transformation matrices of the joint and its parent
	#	gridRotMat = rotation matrix of default cubic grid coordinate system
	#	thickness = thickness measure correlated with joint spacing. Passed through args, not part of this constraint function
	# ======================================== #

	# extract coordinates from params

	transMat = getTransMat(rotMat[1],getPosInOS(rotMat[2].inverse(),params))

	proxSignDist = []
	distSignDist = []

	# go through each distal articular surface point and check if any of them intersect with a proximal mesh (i.e., sigDist < 0)

	for i in range(len(distCoords)):	

		# transform relative grid position into default cubic grid space for tricubic interpolation using rotation matrices

		relWSDist = getPosInWS(rotMat[1], getPosInOS(transMat, distCoords[i]))
		relPosDist = getPosInOS(gridRotMat, getPosInOS(rotMat[0],relWSDist))
		tempSignDist = ipProx.ip([relPosDist[0], relPosDist[1], relPosDist[2]]) # get smaller of the two values
		distSignDist.append(tempSignDist) # enforces thickness threshold on top of min distance (i.e., always min thickness distance in between)

		# check if current point is negative, i.e., inside a mesh, and break loop if so

		if tempSignDist < -1.0e-05: # added minimum value for break condition (below 0 to consider other points as well)
			return(tempSignDist)
		
	distMin = np.min(distSignDist)

	# go through each proximal articular surface point and check if any of them intersect with a distal mesh (i.e., sigDist < 0)

	for i in range(len(proxCoords)):

		# transform relative grid position into default cubic grid space for tricubic interpolation using rotation matrices

		relWSProx = getPosInWS(rotMat[0], proxCoords[i])
		relPosProx = getPosInOS(gridRotMat, getPosInOS(transMat,relWSProx))
		tempSignDist = ipDist.ip([relPosProx[0], relPosProx[1], relPosProx[2]])
		proxSignDist.append((tempSignDist))
		
		# check if current point is negative, i.e., inside a mesh, and break loop if so
		
		if tempSignDist < -1.0e-05: # added minimum value for break condition (below 0 to consider other points as well)
			return(tempSignDist)
	
	proxMin = np.min(proxSignDist)

	return np.min([proxMin,distMin]) # return minimal value, if any of the points is inside a mesh it will be negative


# ========== cost function for optimisation ==========

def cost_fun(params, proxCoords, distCoords, ipProx, ipDist, rotMat, gridRotMat, thickness):

	# Input variables:
	#	params = array of X, Y and Z coordinates of distal element position
	#	proxCoords = Coordinates of proximal articular surface vertices
	#	distCoords = Coordinates of distal articular surface vertices. Passed through args, not part of cost function
	#	ipProx = tricubic interpolation function from tricubic.tricubic() for the signed distance data on the proximal cubic grid. Passed through args, not part of cost function
	#	ipDist = tricubic interpolation function from tricubic.tricubic() for the signed distance data on the distal cubic grid
	#	rotMat = array with the transformation matrices of the joint and its parent
	#	gridRotMat = rotation matrix of default cubic grid coordinate system
	#	thickness = thickness measure correlated with joint spacing
	#
	# ======================================== #
	
	# extract coordinates from params

	transMat = getTransMat(rotMat[1],getPosInOS(rotMat[2].inverse(),params))
	
	signDist = []

	for i in range(len(proxCoords)):

		# transform relative grid position into default cubic grid space for tricubic interpolation using rotation matrices

		relWSProx = getPosInWS(rotMat[0], proxCoords[i])
		relPosProx = getPosInOS(gridRotMat, getPosInOS(transMat,relWSProx))
		tempSignDist = ipDist.ip([relPosProx[0], relPosProx[1], relPosProx[2]])
		signDist.append((tempSignDist)) 

	avgdist = np.mean(signDist) # average distance enforces largest possible overlap
	
	cost = abs(avgdist-thickness)
		
	return cost






