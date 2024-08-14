#	ligamentCalculation1.1.py
#
#	This script calculates the shortest distance of a ligament from origin to insertion 
#	wrapping around the bone meshes. It is an implementation of the Marai et al., 2004.
#	approach for Autodesk Maya. It creates signed distance fields for the proximal and
#	distal bone meshes, which are then used to approximate the 3D path of each ligament
#	accross them to calculate their lengths.
#
#	Written by Oliver Demuth and Vittorio la Barbera
#	Last updated 14.08.2024 - Oliver Demuth
#
#	SYNOPSIS:
#
#		INPUT params:
#			string  jointName:		Name of the joint centre, i.e. the name of a locator or joint (e.g., 'myJoint' if following the ROM mapping protocol of Manafzadeh & Padian 2018)
#			string  meshes:			Name(s) of the bone meshes, i.e., several individual meshes in the form of e.g., ['prox_mesh','dist_mesh']
#			int	gridSubdiv:		Integer value for the subdivision of the cube, i.e., number of grid points per axis, e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points
#			int 	ligSubdiv:		Integer value for the number of ligament points, e.g., 20 will divide the ligament into 20 equidistant segments, see Marai et al., 2004 for details
#
#		RETURN params:
#			list 	ligamentNames:		Return value is a list of the ligament names as designated as custom attributes in the 'jointName'
#			list	pathLengths:		Return value is a list with the path lengths for all ligaments designated as custom attributes in the 'jointName'
#			list 	pathPoints:		Return value is a list of lists with the 3D coordinates of the path points in world space for all ligaments designated as custom attributes in the 'jointName'
#			list 	results: 		Return value is a list of objects of the scipy.optimize.OptimizeResult class. They represent the outputs of the scipy.optimize.minimize() function
#
#
#	IMPORTANT notes:
#		
#	(1) 	Meshes need realtively uniform face areas, otherwise large faces might skew 
#		vertex normals in their direction. It is, therefore, important to extrude 
#		edges around large faces prior to closing the hole at edges with otherwise 
#		acute angles to circumvent this issue, e.g., if meshes have been cut to reduce
#		polycount, prior to executing the Python scripts.
#
#	(2) 	For each ligament create a float attribute at 'jointName' and name it 
#		accordingly. Make sure that the naming convention for the ligament origins 
#		and insertions is correct, i.e. the locators should be named 'ligament*_orig' 
#		and 'ligament*_ins' for an attribute in 'jointName' called 'ligament*'.
#
#	(3)	This script requires several modules for Python, see README file. Make sure to
#		have the following external modules installed for the mayapy application:
#
#			- 'numpy' 	NumPy:			https://numpy.org/about/
#			- 'scipy'	SciPy:			https://scipy.org/about/
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


# ========== ligament length calculation function ==========
def ligCalc(ipProx, ipDist, gridRotMat, rotMat, LigAttributes ,ligSubdiv):

	# Input variables:
	#	ipProx = proximal signed distance field in tricubic form
	#	ipDist = distal signed distance field in tricubic form
	#	gridRotMat = rotation matrix of default cubic grid
	#	rotMat = array with the transformation matrices of the joint and its parent
	#	LigAttributes = array consisting of names of ligaments
	#	ligSubdiv = number of ligament points, e.g., 20 will divide the ligament into 20 equidistant segments, see Marai et al., 2004 for details
	# ======================================== #

	# create variables

	ligNames = []
	ligLengths = []
	ligPoints = []
	results = []

	for i in range(len(LigAttributes)):

		# get names of ligaments and their origins and insertions

		ligNames.append(LigAttributes[i]) 
		ligOrigin = LigAttributes[i] + '_orig'
		ligInsertion = LigAttributes[i] + '_ins'

		# get ligament transformation matrices

		ligRotMat, Offset = getLigTransMat(origin, insertion, jPos)

		# minimise ligament length through optimiser

		res = ligLengthOptMin(ipProx, ipDist, gridRotMat, rotMat, ligRotMat, ligSubdiv)
		
		# get x, y and z coordinates

		x = np.linspace(0.0, 1.0, num = ligSubdiv + 1, endpoint=True)
		y = res.x[0::2] # extract y coordinates from optimiser results
		z = res.x[1::2] # extract z coordinates from optimiser results

		# get world position of ligament points

		coords = []
		for j in range(ligSubdiv + 1):
			coords.append(getPosInWS(rotMat, [x[j],y[j],z[j]])) 

		# correct relative ligament length by linear distance between origin and insertion to get actual ligament length

		ligLengths.append(res.fun * Offset)

		# gather results

		ligPoints.append(coords)
		results.append(res)

	return ligNames, ligLengths, ligPoints, results


# ========== signed distance field per joint function ==========

def sigDistField(jointName, meshes, subdivision):

	# Input variables:
	#	jointName = name of joint centre (represented by object, e.g. joint or locator, in Maya scene)
	#	meshes = name(s) of the mesh(es) for which the signed distance field is calculated
	#	subdivision = number of elements per axis, e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points
	# ======================================== #

	# get joint centre position

	jPos = getWSPos(jointName)
	jDag = dagObjFromName(jointName)[1]
	jInclTransMat = om.MTransformationMatrix(jDag.inclusiveMatrix()) # world transformation matrix of joint
	jExclTransMat = om.MTransformationMatrix(jDag.exclusiveMatrix()) # world transformation atrix of parent of joint

	# get number and names of ligaments

	LigAttributes = cmds.listAttr(jointName, ud = True) # get user defined attributes of 'jointName', i.e. the float attributes that will contain the ligament lengths

	# cycle through ligaments and extract their information

	distArr = []

	for i in range(len(LigAttributes)):

		# get names of ligaments and their origins and insertions

		ligOrigin = LigAttributes[i] + '_orig'
		ligInsertion = LigAttributes[i] + '_ins'

		# get positions of points of interest

		oPos = cmds.xform(ligOrigin, query = True, worldSpace = True, translation = True)
		iPos = cmds.xform(ligInsertion, query = True, worldSpace = True, translation = True)

		# get distances from origin and insertion to joint centre
					
		distArr.append(om.MVector(jPos[0] - iPos[0],
					  jPos[1] - iPos[1],
					  jPos[2] - iPos[2]).length()) # Euclidean distance from insertion to joint centre, i.e., scale factor for grid point positions

		distArr.append(om.MVector(jPos[0] - oPos[0],
					  jPos[1] - oPos[1],
					  jPos[2] - oPos[2]).length()) # Euclidean distance from origin to joint centre, i.e., scale factor for grid point positions

	maxDist = max(distArr) # maximal distance from joint centre to ligament attachment

	# normalize matrices by maxDist

	jInclTransMat.setScale([maxDist,maxDist,maxDist],om.MSpace.kWorld) # set scale in world space
	jExclTransMat.setScale([maxDist,maxDist,maxDist],om.MSpace.kWorld) # set scale in world space

	# get rotation matrices

	rotMat = []
	rotMat.append(om.MMatrix(jExclTransMat.asMatrix())) # parent rotMat (prox)
	rotMat.append(om.MMatrix(jInclTransMat.asMatrix())) # child rotMat (dist)

	# create variables

	sigDistances = []
	localPoints = []
	worldPoints = []

	# cycle through all meshes

	if length(meshes) > 2
		error('Too many meshes in array. Please specify only the distal and proximal bone mesh in the mesh array.')
	else if length(mmeshes) < 2
		error('Too few meshes specified. Please specify TWO meshes in the mesh array.')
	
	for j,mesh in enumerate(meshes):
		meshSigDist, osPoints, wsPoints = sigDistMesh(mesh, rotMat[j], subdivision) # get signed distance field for each mesh
		sigDistances.append(np.array(meshSigDist).reshape(subdivision + 1, subdivision + 1, subdivision + 1))
		localPoints.append(osPoints)
		worldPoints.append(wsPoints)
	
	return sigDistances, localPoints, worldPoints, rotMat, LigAttributes


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
	meshFn = om.MFnMesh(dag)
	
	# get the meshs transformation matrix

	meshMat = om.MMatrix(dag.inclusiveMatrix())

	# create the intersector

	polyIntersect = om.MMeshIntersector()
	polyIntersect.create(mObj,meshMat)
	ptON = om.MPointOnMesh()

	# create 3D grid where all axes are 1.5 times the distance between origin and insertion

	elements = np.linspace(-1.5, 1.5, num = subdivision + 1, endpoint=True, dtype=float)
	
	points = [[i, j, k] for i in elements  # length along x
			    for j in elements  # length along y
			    for k in elements] # length along z

	# go through grid points and calculate signed distance for each of them

	signedDist = []
	gridPoints = []

	meshMat = om.MMatrix(cmds.xform(mesh, q=True, matrix=True, ws=True)) # get world matrix of mesh

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


# ========== ligament rotation matrix function ==========

def getLigTransMat(origin, insertion, jPos):

	# Input variables:
	#	origin = name of ligament origin (represented by object, e.g. locator, in Maya scene)
	#	insertion = name of ligament insertion (represented by object, e.g. locator, in Maya scene)
	#	jPos = joint centre position
	# ======================================== #

	# get positions of points of interest

	oPos = getWSPos(origin)
	iPos = getWSPos(insertion)

	# calculate vectors from origin to insertion and to joint centre
				
	LigDir = om.MVector(iPos[0] - oPos[0],
			    iPos[1] - oPos[1],
			    iPos[2] - oPos[2])

	JointDir = om.MVector(jPos[0] - oPos[0],
			      jPos[1] - oPos[1],
			      jPos[2] - oPos[2])

	Offset = LigDir.length() # linear distance from origin to insertion, i.e., scale factor for grid point positions

	# calculate planes relative to ligaments

	uDir = LigDir.normal() * Offset
	wDir = (JointDir.normal() ^ uDir).normal() * Offset# cross product to get z axis
	vDir = (wDir ^ uDir).normal() * Offset # cross product to get y axis 

	# get transformation matrix from liagament plane directions

	transMat = om.MMatrix([uDir.x, uDir.y, uDir.z, 0,
			       vDir.x, vDir.y, vDir.z, 0,
			       wDir.x, wDir.y, wDir.z, 0,
			       oPos[0],oPos[1],oPos[2],1]) # centre position around origin
		
	return transMat, Offset 


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



################################################
# ============ optimiser functions =========== #
################################################

def ligLengthOptMin(ipProx, ipDist, gridRotMat, rotMat, ligRotMat, ligSubdiv):

	# Input variables:
	#	ipProx = proximal signed distance field in tricubic form
	#	ipDist = distal signed distance field in tricubic form
	#	gridRotMat = rotation matrix of default cubic grid
	#	rotMat = array with the transformation matrices of the joint and its parent
	#	ligRotMat = transformation matrix of the ligament
	#	ligSubdiv = number of ligament points, e.g., 20 will divide the ligament into 20 equidistant segments, see Marai et al., 2004 for details
	# ======================================== #

	# define initual guess condition

	xCoords = np.linspace(0.0, 1.0, num = ligSubdiv + 1, endpoint = True)
	initial_guess = np.zeros(2 * (ligSubdiv + 1))

	# create tuple for arguments passed to both constraints and cost functions

	arguments = (xCoords, ipProx, ipDist, rotMat, ligRotMat, gridRotMat)

	# set constraints functions

	cons = ({'type': 'ineq',		# set type to inequality, which means that it is to be non-negative
		 'fun': sigDist_cons_fun,	# set constraint function
		 'args': arguments},		# pass arguments to constrain function
		{'type': 'ineq',		# set type to inequality, which means that it is to be non-negative
		 'fun': path_cons_fun,		# set constraint function
		 'args': arguments})		# pass arguments to constrain function

	# set bounds

	boundsList = [(-10,10)] * (len(initial_guess)) # set y and z coordinate boundaries to 10 times euclidean distance between  origin and insertion
	boundsList[0] = boundsList[1] = boundsList[len(initial_guess) - 2] = boundsList[len(initial_guess) - 1] = (0,0) # set y and z bounds of origin and insertion to zero
	bnds = tuple(boundsList)

	# set options

	options = {"maxiter": 500} # if it doesn't solve within 500 iterations it usually won't solve

	# optimization using SLSQP

	res = sp.optimize.minimize(cost_fun, initial_guess, args = arguments, bounds = bnds, method = 'SLSQP', constraints = cons, options = options)

	# get results: res.X = ligament point coordinates, res.fun = relative ligament length

	return res


# ========== signed distance field constraint function ==========

def sigDist_cons_fun(params, x, ipProx, ipDist, rotMat, ligRotMat, gridRotMat):

	# Input variables:
	#	params = array of Y and Z coordinates of ligament points, i.e., params = [(y_0, z_0),(y_1, z_1), ... ,(y_n-1, z_n-1)]. They are, however, flattened into a single array, i.e., [y0, z0, y1, z1, y2, z2, ... , y_n-1, z_n-1], and therefore need to be extracted.
	#	x = constant X coordinates for ligament points
	#	ipProx = tricubic interpolation function from tricubic.tricubic() for the signed distance data on the proximal cubic grid
	#	ipDist = tricubic interpolation function from tricubic.tricubic() for the signed distance data on the distal cubic grid
	#	rotMat = array with the transformation matrices of the joint and its parent
	#	ligRotMat = transformation matrix of the ligament
	#	gridRotMat = rotation matrix of default cubic grid coordinate system
	# ======================================== #

	# extract coordinates from params

	y = params[0::2]
	z = params[1::2]

	signDist = []

	# go through each ligament point (except origin and insertion) and check if any of them intersect with a mesh (i.e., sigDist < 0)

	n = len(x)-2

	for i in range(n):

		# transform relative grid position into default cubic grid space for tricubic interpolation using rotation matrices

		ligWSPoint = getPosInWS(ligRotMat,[x[i+1], y[i+1], z[i+1]])

		relPosProx = getPosInOS(gridRotMat, ligWSPoint * rotMat[0].inverse())
		relPosDist = getPosInOS(gridRotMat, ligWSPoint * rotMat[1].inverse())
		
		# tricubic interpolation of signed distance at postion relative to default cubic grid

		tempSignDist = min(ipProx.ip([relPosProx[0], relPosProx[1], relPosProx[2]]), ipDist.ip([relPosDist[0], relPosDist[1], relPosDist[2]])) # get smaller of the two values
		signDist.append(tempSignDist)

		# check if current point is negative, i.e., inside a mesh, and break loop if so

		if 1e-05 <= tempSignDist < 0: # added minimum value for break condition because below threshold optimizer will end without considering other ligament points
			break
	
	return min(signDist) # return minimal value, if any of the points is inside a mesh it will be negative


# ========== path constraint function ==========

def path_cons_fun(params, x, ipProx, ipDist, rotMat, ligRotMat, gridRotMat):

	# Input variables:
	#	params = array of Y and Z coordinates of ligament points, i.e., params = [(y_0, z_0),(y_1, z_1), ... ,(y_n-1, z_n-1)]. They are, however, flattened into a single array, i.e., [y0, z0, y1, z1, y2, z2, ... , y_n-1, z_n-1], and therefore need to be extracted.
	#	x = constant X coordinates for ligament points
	#	ipProx = tricubic interpolation function from tricubic.tricubic() for the signed distance data on the proximal cubic grid. Passed through args, not part of this constraint function
	#	ipDist = tricubic interpolation function from tricubic.tricubic() for the signed distance data on the distal cubic grid. Passed through args, not part of this constraint function
	#	rotMat = array with the transformation matrices of the joint and its parent. Passed through args, not part of this constraint function
	#	ligRotMat = transformation matrix of the ligament. Passed through args, not part of this constraint function
	#	gridRotMat = rotation matrix of default cubic grid coordinate system. Passed through args, not part of this constraint function
	# ======================================== #

	# extract coordinates from params

	y = params[0::2]
	z = params[1::2]

	# get number of ligament segments

	n = len(x)-1

	# calculate distance between subsequent points

	dist = []

	for i in range(n):
		dist.append(((y[i+1] - y[i])**2 + (z[i+1] - z[i])**2)**0.5) # diagonal YZ distance between subsequent points

	maxDist = max(dist) # get maximum diagonal YZ distance 

	distDiff = 1.5 / n - maxDist # mediolateral offset can be maximally 1.5 times the distance between path segment along X-axis. This is to avoid path penetrating through thin parts of the mesh, where path points would be on either side of mesh
	

	return distDiff


# ========== cost function for optimisation ==========

def cost_fun(params, x, ipProx, ipDist, rotMat, ligRotMat, gridRotMat):

	# Input variables:
	#	params = array of Y and Z coordinates of ligament points, i.e., params = [[y_0, z_0],[y_1, z_1], ... ,[y_n-1, z_n-1]]. They are, however, flattened into a single array, i.e., [y0, z0, y1, z1, y2, z2, ... , y_n-1, z_n-1], and therefore need to be extracted.
	#	x = constant X coordinates for ligament points.
	#	ipProx = tricubic interpolation function from tricubic.tricubic() for the signed distance data on the proximal cubic grid. Passed through args, not part of this constraint function
	#	ipDist = tricubic interpolation function from tricubic.tricubic() for the signed distance data on the distal cubic grid. Passed through args, not part of this constraint function
	#	rotMat = array with the transformation matrices of the joint and its parent. Passed through args, not part of this constraint function
	#	ligRotMat = transformation matrix of the ligament. Passed through args, not part of this constraint function
	#	gridRotMat = rotation matrix of default cubic grid coordinate system. Passed through args, not part of this constraint function
	# ======================================== #
	
	# extract coordinates from params

	y = params[0::2]
	z = params[1::2]
	
	# get number of ligament segments

	n = len(x)-1 
	
	# set value of constant, i.e., the fraction of the ligament 
	
	const = (1/n)**2 
	
	# cost, i.e., ligament length to be minimized
	
	cost = 0
	
	for i in range(n):
		
		# sum up the distances between the individiual ligament points, i.e., the length of the individual segments
		
		cost += (const + (y[i+1] - y[i])**2 + (z[i+1] - z[i])**2)**0.5
		
	return cost