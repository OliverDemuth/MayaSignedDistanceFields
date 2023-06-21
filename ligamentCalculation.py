#	ligamentCalculation.py
#
#	This script calculates the shortest distance of a ligament from origin to insertion 
#	wrapping around the bone meshes. It is an implementation of the Marai et al., 2004.
#	approach for Autodesk Maya. It creates a signed distance field of the bone meshes
#	for each ligament, which is then used to approximate its 3D path and calculate its
#	length. 
#
#	Written by Oliver Demuth and Vittorio la Barbera
#	Last updated 20.06.2023 - Oliver Demuth
#
#	SYNOPSIS:
#
#		INPUT params:
#			string  jointName:		Name of the joint centre, i.e. the name of a locator or joint, e.g., 'myJoint' if following the ROM mapping protocol of Manafzadeh & Padian 2018
#			string  meshes:			Name(s) of the bone meshes, e.g., the boolean object if following the ROM mapping protocol of Manafzadeh & Padian 2018, i.e., 'boo', or several individual meshes in the form of e.g., ['prox_mesh','dist_mesh']
#			int	gridSubdiv:		Integer value for the subdivision of the cube, i.e., number of grid points per axis, e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points
#			int 	ligSubdiv:		Integer value for the number of ligament points, e.g., 20 will divide the ligament into 20 equidistant segments, see Marai et al., 2004 for details
#
#		RETURN params:
#			list 	ligamentNames:		Return value is a list of the ligament names designated as custom attributes in the 'jointName'
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

def ligCalc(jointName, meshes, gridSubdiv ,ligSubdiv):

	# Input variables:
	#	jointName = name of joint centre (represented by object, e.g. joint or locator, in Maya scene)
	#	meshes = name(s) of the mesh(es) for which the signed distance field is calculated
	#	gridSubdiv = number of elements for X-axis, e.g., 20 will result in a cube grid with 21 x 21 21 grid points
	#	ligSubdiv = number of ligament points, e.g., 20 will divide the ligament into 20 equidistant segments, see Marai et al., 2004 for details
	# ======================================== #

	# get joint centre position

	jPos = cmds.xform(jointName, query = True, worldSpace = True, translation = True)

	# check if gridSubdiv is even or odd

	if int(gridSubdiv) % 2 == 0: subDiv = int(gridSubdiv) # subdivision is even
	else: subDiv = int(gridSubdiv) + 1 # make odd numbers even

	# get number of ligaments

	LigAttributes = cmds.listAttr(jointName, ud = True) # get user defined attributes of 'jointName', i.e. the float attributes that will contain the ligament lengths
	
	# create variables

	ligNames = []
	ligLengths = []
	ligPoints = []
	results=[]

	for i in range(len(LigAttributes)):

		# get names of ligaments and their origins and insertions

		ligNames.append(LigAttributes[i]) 
		ligOrigin = LigAttributes[i] + '_orig'
		ligInsertion = LigAttributes[i] + '_ins'

		# get grid points relative to ligament

		sigDistances, osPoints, wsPoints, Offset, rotMat = sigDistField(ligOrigin, ligInsertion, jPos, meshes, subDiv)
		
		# minimise ligament length through optimiser

		sigDistFieldArray = sigDistances.reshape(subDiv + 1, subDiv + 1, subDiv + 1)

		# minimise ligament length through optimiser

		res = ligLengthOptMin(sigDistFieldArray, osPoints, ligSubdiv)
		
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

def sigDistField(origin, insertion, jPos, meshes, subdivision):

	# Input variables:
	#	origin = name of ligament origin (represented by object, e.g. locator, in Maya scene)
	#	insertion = name of ligament insertion (represented by object, e.g. locator, in Maya scene)
	#	jPos = joint centre position
	#	meshes = name(s) of the mesh(es) for which the signed distance field is calculated
	#	subdivision = number of elements per axis, e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points
	# ======================================== #

	# get positions of points of interest

	oPos = cmds.xform(origin, query = True, worldSpace = True, translation = True)
	iPos = cmds.xform(insertion, query = True, worldSpace = True, translation = True)

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
	wDir = (JointDir.normal() ^ uDir).normal() * Offset # cross product to get z axis
	vDir = (wDir ^ uDir).normal() * Offset # cross product to get y axis 

	# get rotation matrix from liagament plane directions

	rotMat = om.MMatrix([uDir.x, uDir.y, uDir.z, 0,
			     vDir.x, vDir.y, vDir.z, 0,
			     wDir.x, wDir.y, wDir.z, 0,
			     oPos[0],oPos[1],oPos[2],1]) # centre position around origin

	# create variables

	sigDistList = []
	gridPoints = []
	sigDistances = []

	# cycle through all meshes
	
	for mesh in meshes:
		meshSigDist, osPoints, wsPoints = sigDistMesh(mesh, Offset, rotMat, subdivision) # get signed distance field for each mesh
		sigDistList.append(meshSigDist)

	# compare distances for grid points relative to individual meshes and get the smallest one

	if len(meshes) == 1:
		sigDistances = np.array(sigDistList[0]) # for a single mesh
	elif len(meshes) == 2:
		sigDistances = np.minimum(sigDistList[0],sigDistList[1]) # for two meshes (faster than np.amin() for element-wise comparison of two arrays)
	else:
		sigDistances = np.amin(sigDistList, axis = 0) # for more than two meshes
	
	return sigDistances, osPoints, wsPoints, Offset, rotMat


# ========== signed distance field per mesh function ==========

def sigDistMesh(mesh, Offset, rotMat, subdivision):

	# Input variables:
	#	mesh = name of the mesh for which the signed distance field is calculated
	# 	Offset = distance from origin to insertion
	#	rotMat = rotation matrix for points in the ligament coordinate system relative to mid point between origin and insertion 
	#	subdivision = number of elements per axis, e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points
	# ======================================== #

	# get dag paths

	obj, dag = dagObjFromName(mesh)

	# create MObject

	shape = dag.extendToShape()
	mObj = shape.node()
	meshFn = om.MFnMesh(dag)
	
	# get the meshs transformation matrix

	mat = om.MMatrix(dag.inclusiveMatrix())

	# create the intersector

	polyIntersect = om.MMeshIntersector()
	polyIntersect.create(mObj,mat)
	ptON = om.MPointOnMesh()

	# create 3D grid where all axes are 1.5 times the distance between origin and insertion

	xElements = np.linspace(-0.25, 1.25, num = subdivision + 1, endpoint=True, dtype=float)
	elements = np.linspace(-1, 1, num = subdivision + 1, endpoint=True)
	
	points = [[i, j, k] for i in xElements # length along x
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


# ========== get point in world space function ==========	

def getPosInWS(rotMat,point):

	# Input variables:
	#	rotMat = rotation matrix
	#	point = 3D coordinates (translation) of point in object space (relative to parent)
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

def ligLengthOptMin(sigDistFieldArray, relGridPoints, ligSubdiv):

	# Input variables:
	#	sigDistFieldArray = 3D numpy array of signed distance field
	#	relGridPoints = 3D coordinates of grid points for signed distance field relative to ligament orientation
	#	ligSubdiv = number of ligament points, e.g., 20, see Marai et al., 2004 for details
	# ======================================== #

	# define initual guess condition

	xCoords = np.linspace(0.0, 1.0, num = ligSubdiv + 1, endpoint = True)
	initial_guess = np.zeros(2 * (ligSubdiv + 1))
												  
	# get dimensions of cubic grid

	dims = sigDistFieldArray.shape
	sigDistList = sigDistFieldArray.tolist()

	# get corner points of cubic grid relative to ligament

	origPos = om.MVector(relGridPoints[0])
	zVecPos = om.MVector(relGridPoints[dims[1] - 1])
	yVecPos = om.MVector(relGridPoints[dims[1] * (dims[2] - 1)])
	xVecPos = om.MVector(relGridPoints[dims[1] * dims[2] * (dims[0] - 1)])
	
	# get direction vectors to cubic grid corners and normalize by number of grid subdivisions

	xDir = (xVecPos - origPos) / (dims[0] - 1)
	yDir = (yVecPos - origPos) / (dims[1] - 1)
	zDir = (zVecPos - origPos) / (dims[2] - 1)

	# get rotation matrix from ligament coordinate system to default cubic grid coordinate system

	rotMat = om.MMatrix([	xDir.x, 	xDir.y,		xDir.z, 	0,
			     	yDir.x, 	yDir.y, 	yDir.z, 	0,
			     	zDir.x, 	zDir.y, 	zDir.z, 	0,
			     	origPos.x,	origPos.y,	origPos.z,	1]) 

	# initialise tricubic interpolator with signed distance data on default cubic grid

	ip = tricubic(sigDistList, [dims[0], dims[1], dims[2]]) # grid will be initialised in its relative coordinate system from [0,0,0] to [gridSubdiv+1, gridSubdiv+1, gridSubdiv+1].

	# create tuple for arguments passsed to both constraints and cost functions

	arguments = (xCoords, ip, rotMat)

	# set constraints function

	cons = ({'type': 'ineq',		# set type to inequality, which means that it is to be non-negative
		 'fun': sigDist_cons_fun,	# set constraint function
		 'args': arguments},
		{'type': 'ineq',		# set type to inequality, which means that it is to be non-negative
		 'fun': path_cons_fun,		# set constraint function
		 'args': arguments})		# pass arguments to constrain function

	# set bounds

	boundsList = [(-0.75,0.75)] * (len(initial_guess)) # keep y and z coordinates of points within boundary of cubic grid
	boundsList[0] = boundsList[1] = boundsList[len(initial_guess) - 2] = boundsList[len(initial_guess) - 1] = (0,0) # set y and z bounds of origin and insertion to zero
	bnds = tuple(boundsList)

	# set options

	options = {"maxiter": 500} # if it doesn't solve within 500 iterations it usually won't solve

	# optimization using SLSQP

	res = sp.optimize.minimize(cost_fun, initial_guess, args = arguments, bounds = bnds, method = 'SLSQP', constraints = cons, options = options)

	# get results: res.X = ligament point coordinates, res.fun = relative ligament length

	return res


# ========== signed distance field constraint function ==========

def sigDist_cons_fun(params, x, ip, rotMat):

	# Input variables:
	#	params = array of Y and Z coordinates of ligament points, i.e., params = [(y_0, z_0),(y_1, z_1), ... ,(y_n-1, z_n-1)]. They are, however, flattened into a single array, i.e., [y0, z0, y1, z1, y2, z2, ... , y_n-1, z_n-1], and therefore need to be extracted.
	#	x = constant X coordinates for ligament points
	#	ip = tricubic interpolation function from tricubic.tricubic() for the signed distance data on the cubic grid
	#	rotMat = rotation matrix from ligament coordinate system to default cubic grid coordinate system
	# ======================================== #

	# extract coordinates from params

	y = params[0::2]
	z = params[1::2]

	signDist = []

	# go through each ligament point (except origin and insertion) and check if any of them intersect with a mesh, i.e., sigDist < 0

	n = len(x)-2

	for i in range(n):

		# transform relative grid position into default cubic grid space for tricubic interpolation using rotation matrix

		relPos = getPosInOS(rotMat,[x[i+1], y[i+1], z[i+1]])
		
		# tricubic interpolation of signed distance at postion relative to default cubic grid

		tempSignDist = ip.ip([relPos[0], relPos[1], relPos[2]])
		signDist.append(tempSignDist)

		# check if current point is negative, i.e., inside a mesh, and break loop if so

		if 1e-05 <= tempSignDist < 0: # added minimum value for break condition because below threshold optimizer will end without considering other ligament points
			break
	
	return min(signDist) # return minimal value, if any of the points is inside a mesh it will be negative


# ========== path constraint function ==========

def path_cons_fun(params, x, ip, rotMat):

	# Input variables:
	#	params = array of Y and Z coordinates of ligament points, i.e., params = [(y_0, z_0),(y_1, z_1), ... ,(y_n-1, z_n-1)]. They are, however, flattened into a single array, i.e., [y0, z0, y1, z1, y2, z2, ... , y_n-1, z_n-1], and therefore need to be extracted.
	#	x = constant X coordinates for ligament points
	#	ip = tricubic interpolation function from tricubic.tricubic() for the signed distance data on the cubic grid 
	#	rotMat = rotation matrix from ligament coordinate system to default cubic grid coordinate system
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

def cost_fun(params, x, ip, rotMat):

	# Input variables:
	#	params = array of Y and Z coordinates of ligament points, i.e., params = [[y_0, z_0],[y_1, z_1], ... ,[y_n-1, z_n-1]]. They are, however, flattened into a single array, i.e., [y0, z0, y1, z1, y2, z2, ... , y_n-1, z_n-1], and therefore need to be extracted.
	#	x = constant X coordinates for ligament points.
	#	ip = tricubic interpolation function from tricubic.tricubic() for the signed distance data on the cubic grid. Passed through args, not part of cost function but constraints function
	#	rotMat = rotation matrix from ligament coordinate system to default cubic grid coordinate system. Passed through args, not part of cost function but constraints function
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


