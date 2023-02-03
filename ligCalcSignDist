#	ligCalcSignDist.py
#
#	This script calculates the shortest distance of a ligament from origin to insertion 
#	wrapping around the bone meshes. It is an implementation of the Marai et al., 2004.
#	approach for Autodesk Maya. It creates a signed distance field for the bone meshes 
#	around the joint, which is then used to calculate the 3D ligament lengths. 
#
#	Written by Oliver Demuth and Vittorio la Barbera
#	Last updated 03.02.2023 - Oliver Demuth
#
#	SYNOPSIS:
#
#		INPUT params:
#			string  jointName:		Name of the joint centre, i.e. the name of a locator or joint, e.g. "myJoint" if following the ROM mapping protocol of Manafzadeh & Padian 2018
#			string  meshes:			Name(s) of the bone meshes, e.g., the boolean object if following the ROM mapping protocol of Manafzadeh & Padian 2018, or several individual meshes
#			float	gridSize:		Float value for the dimension of a cube representing the bounding box of the grid points, e.g., 10 will results in a cube with a width, height and depth of 10 centered around the joint centre
#			int		gridSubdiv:		Integer value for the subdivision of the cube, i.e., number of grid points for any given axis, e.g., 20 will result in a cube grid with 21 x 21 x21 grid points
#			int 	ligSubdiv:		Integer value for the number of ligament points, e.g., 40 is a good starting point according to Marai et al., 2004. It will divide the ligament into 40 equidistant segments. 
#
#		RETURN params:
#			list	pathLengths:	Return value is a list with the path lengths for all ligaments designated as custom attributes in the 'jointName', see below.
#
#
#	IMPORTANT notes:
#		
#	(1) Meshes need realtively uniform face areas, otherwise large faces might skew 
#		vertex normals in their direction! It is, therefore, important to extrude 
#		edges around large faces prior to closing the hole at edges with otherwise 
#		acute angles to circumvent this issue, e.g., if meshes have been cut to reduce
#		polycount, prior to executing the Python scripts.
#
#	(2) For each ligament create a float attribute at 'jointName' and name it 
#		accordingly. Make sure that the naming convention for the ligament origins 
#		and insertions is correct, i.e. the locators should be named 'ligament*_orig' 
#		and 'ligament*_ins' for an attribute in 'jointName' called 'ligament*'.
#
#	(3)	This script requires several modules for Python. Make sure to have the 
#		following external plugins installed:
#
#			- 'numpy' 		NumPy:					https://numpy.org/about/
#			- 'scipy'		SciPy:					https://scipy.org/about/
#			- 'tricubic' 	Daniel Guterding: 		https://github.com/danielguterding/pytricubic
#				
#		For further information regarding them, please check the websites referenced 
#		above.


# ========== load plugins ==========

import maya.api.OpenMaya as om
import maya.cmds as cmds
import numpy as np
import scipy as sp
import time
from tricubic import tricubic


# ========== TESTTING ==========


meshes = ['prox_mesh','dist_mesh']
#meshes = ['bones_boo']


#array,points = ligCalc('myJoint', meshes, 20, 16 ,20) # ligCalc attributes in the follwoing order (jointName, meshes, gridSize, gridSubdiv ,ligSubdiv):

#rotM, points = ligCalc('myJoint', meshes, 20, 16 ,20) # ligCalc attributes in the follwoing order (jointName, meshes, gridSize, gridSubdiv ,ligSubdiv):

res = ligCalc('myJoint', meshes, 20, 16 ,20) # ligCalc attributes in the follwoing order (jointName, meshes, gridSize, gridSubdiv ,ligSubdiv):



# ============================ #


################################################
# ========== MAYA specific functions ========= #
################################################


# ========== ligament length calculation function ==========

def ligCalc(jointName, meshes, gridSize, gridSubdiv ,ligSubdiv):

	# Input variables:
	#	jointName = name of joint centre (represented by object, e.g. joint or locator, in Maya scene)
	#	meshes = name(s) of the mesh(es) for which the signed distance field is calculated
	#	gridSize = edge length of a cube for the 3D grid, e.g., 10 will results in a cube with a width, height and depth of 10 centered around the joint centre
	#	gridSubdiv = number of elements for any given axis, e.g., 20 will result in a cube grid with 21 x 21 x21 grid points
	#	ligSubdiv = number of ligament points, e.g., 40 is a good starting point according to Marai et al., 2004. It will divide the ligament into 40 equidistant segments. 
	# ======================================== #

	# ==== testing ====
	start = time.time()
	cmds.currentTime(1) # force boolean to update
	# ================ #

	# get joint centre position

	jPos = cmds.xform(jointName, query = True, worldSpace = True, translation = True)

	# get signed distance field

	sigDistances, gridPoints = sigDistField(jointName, meshes, gridSize, gridSubdiv)

	npSigDist = np.array(sigDistances)
	sigDistFieldArray = list(npSigDist.reshape(gridSubdiv+1, gridSubdiv+1, gridSubdiv+1))

	# ==== testing ====
	check1 = time.time()
	print('# Signed Distance Field calculation done in {:0.3f} seconds!'.format(check1 - start)) # ~ 0.407 seconds for two meshes, ~ 0.438 seconds for boolean, including boolean update
	# ================ #

	# get number of ligaments

	LigAttributes = cmds.listAttr(jointName,ud=True) # get user defined attributes of 'jointName', i.e. the float attributes that will contain the ligament lengths
	
	ligNames = []
	ligLengths = []
	relRotMatArray = []
	relGridPointArray = []
	ligPoints = []

	#for i in range(len(LigAttributes)):
	for i in range(1):

		# get names of ligaments and their origins and insertions

		ligNames.append(LigAttributes[i]) 
		ligOrigin = LigAttributes[i]+'_orig'
		ligInsertion = LigAttributes[i]+'_ins'

		relGridPoints, Offset = ligRelCoord(ligOrigin, ligInsertion, jPos, gridPoints)

		#relLigPoints, relLigLength = relLigLength(ligSubdiv, sigDistances, relGridPoints)
		res = ligLengthOptMin(sigDistFieldArray, relGridPoints, gridSubdiv, ligSubdiv)


		reLiglength = 1

		#ligLengths.append(reLiglength * Offset)
		ligLengths.append(fun)
		ligPoints.append(x)

		relRotMatArray.append(relRotMat)
		relGridPointArray.append(relGridPoints)
		
	# ==== testing ====
	end = time.time()
	print('# Relative coordinate system calculation done in {:0.3f} seconds!'.format(end - check1)) # ~ 0.154 seconds for 5 ligaments
	print('# Total time: {:0.3f} seconds!'.format(end - start)) # ~ 0.561 seconds for two meshes, ~ 0.592 seconds for boolean, including boolean update
																
	# ================ #

	return res



# ========== ligament relative grid coordinates function ==========

def ligRelCoord(origin,insertion,jPos,gridPoints):

	# Input variables:
	#	origin = name of ligament origin (represented by object, e.g. locator, in Maya scene)
	#	insertion = name of ligament insertion (represented by object, e.g. locator, in Maya scene)
	#	jointName = name of joint centre (represented by object, e.g. joint or locator, in Maya scene)
	#	gridPoints = 3D coordinates of grid points for signed distance field
	# ======================================== #

	# get positions of points of interest

	oPos = cmds.xform(origin, query = True, worldSpace = True, translation = True)
	iPos = cmds.xform(insertion, query = True, worldSpace = True, translation = True)

	# calculate vectors from origin to insertion
				
	LigDir = om.MVector(iPos[0] - oPos[0],
						iPos[1] - oPos[1],
						iPos[2] - oPos[2])

	JointDir = om.MVector(oPos[0] - jPos[0],
						  oPos[1] - jPos[1],
						  oPos[2] - jPos[2])

	Offset = LigDir.length() # linear distance from origin to insertion, i.e., scale factor for grid point positions

	# calculate planes relative to ligaments

	uDir = LigDir.normal() * Offset
	vDir = (uDir ^ JointDir.normal()).normal() * Offset # cross product to get up axis
	wDir = (uDir ^ vDir).normal() * Offset # cross product to get axis 

	# get rotation matrix from liagament plane directions

	rotMat = om.MMatrix([uDir.x, uDir.y, uDir.z, 0,
						 vDir.x, vDir.y, vDir.z, 0,
						 wDir.x, wDir.y, wDir.z, 0,
						 oPos[0],oPos[1],oPos[2],1]) # centre position around origin
	

	# get grid point position on ligament coordinate system normalized to distance between origin and insertion

	relGridPoints = []
	
	for point in gridPoints:
		relGridPoints.append(getPosInOS(rotMat, point))
		
	return relGridPoints, Offset


# ========== signed distance field per joint function ==========

def sigDistField(jointName, meshes, size, subdivision):

	# Input variables:
	#	jointName = name of joint centre (represented by object, e.g. joint or locator, in Maya scene)
	#	meshes = name(s) of the mesh(es) for which the signed distance field is calculated
	#	size = edge length of a cube for the 3D grid, e.g., 10 will results in a cube with a width, height and depth of 10 centered around the joint centre
	#	subdivision = number of elements for any given axis, e.g., 20 will result in a cube grid with 21 x 21 x21 grid points
	# ======================================== #

	# create variables

	sigDistList = []
	gridPoints = []
	sigDistances = []

	# cycle through all meshes
	
	for mesh in meshes:
		meshSigDist, gridPoints = sigDistMesh(jointName, mesh, size, subdivision) # get signed distance field for each mesh
		sigDistList.append([meshSigDist])
	
	# compare distances for individual grid points and get the smallest one

	for items in zip(*sigDistList[:len(meshes)]):
		sigDistances.append(min(items))
	
	return sigDistances, gridPoints


# ========== signed distance field per mesh function ==========

def sigDistMesh(jointName, mesh, size, subdivision):

	# Input variables:
	#	jointName = name of joint centre (represented by object, e.g. joint or locator, in Maya scene)
	#	mesh = name of ligament insertion (represented by object, e.g. locator, in Maya scene)
	#	size = edge length of a cube for the 3D grid, e.g., 10 will results in a cube with a width, height and depth of 10 centered around the joint centre
	#	subdivision = number of elements for any given axis, e.g., 20 will result in a cube grid of 20 x 20 x20
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
	
	# create variables for intersector

	resultPoint = om.MPoint()
	ptON = om.MPointOnMesh()

	# get joint centre position

	jPos = cmds.xform(jointName, query = True, worldSpace = True, translation = True)

	# create 3D grid

	elements = np.linspace(size / -2, size / 2, num = subdivision  + 1, endpoint=True)
	points = [[i, j, k] for i in elements
						for j in elements
						for k in elements]

	# go through grid points and calculate signed distance for each of them

	signedDist = []
	gridPoints = []

	meshMat = om.MMatrix(cmds.xform(mesh, q=True, matrix=True, ws=True)) # get world matrix of mesh

	for point in points:

		# get 3D position of grid points 

		gridPoint = om.MPoint(jPos[0] - point[0],
							  jPos[1] - point[1],
							  jPos[2] - point[2]) 
		
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

		if dot >= 0: # point is inside mesh
			signedDist.append(dist * -1)
		else: # point is outside of mesh
			signedDist.append(dist)
			
	return signedDist, gridPoints


# ========== get point in mesh object space function ==========	
	
def getPosInOS(rotMat, point):

	# Input variables:
	#	rotMat = rotation matrix
	#	point = 3D coordinates (translation) of point in world space
	# ======================================== #

	inverseMat = rotMat.inverse()

	ptTrans = (point[0], point[1], point[2], 1)

	ptWS = om.MMatrix(((1, 0, 0, 0),
					   (0, 1, 0, 0),
					   (0, 0, 1, 0),
					   ptTrans))

	ptPosOS = (ptWS * inverseMat)

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
# ============ optimizer functions =========== #
################################################

def ligLengthOptMin(sigDistFieldArray, relGridPoints, gridSubdiv, ligSubdiv):

	# Input variables:
	#	sigDistFieldArray = 3D array of signed distance field
	#	relGridPoints = 3D coordinates of grid points for signed distance field relative to ligament orientation
	#	gridSubdiv = number of elements for any given axis, e.g., 20 will result in a cube grid with 21 x 21 x21 grid points
	#	ligSubdiv = number of ligament points, 40 is a good starting point according to Marai et al., 2004
	# ======================================== #

	# define initual guess condition

	xCoords = np.linspace(0.0, 1.0, num = ligSubdiv+1, endpoint=True)
	yCoords = zCoords =[0]

	initial_guess = np.array([[round(i,10), j, k] for i in xCoords # round to 10 decimal points to avoid floating point errors
										 for j in yCoords
										 for k in zCoords]).ravel()
												  
	# get dimensions of cubic grid

	dim = gridSubdiv + 1

	# get corner points of cubic grid relative to ligament

	origPos = om.MVector(relGridPoints[0])
	xVecPos = om.MVector(relGridPoints[dim - 1])
	yVecPos = om.MVector(relGridPoints[dim * (dim - 1)])
	zVecPos = om.MVector(relGridPoints[dim * dim * (dim - 1)])

	# get direction vectors to cubic grid corners and normalize by number of grid subdivisions

	xDir = (xVecPos - origPos) / gridSubdiv
	yDir = (yVecPos - origPos) / gridSubdiv
	zDir = (zVecPos - origPos) / gridSubdiv

	# get rotation matrix from ligament coordinate system to default cubic grid coordinate system

	rotMat = om.MMatrix([xDir.x, 	xDir.y,		xDir.z, 	0,
						 yDir.x, 	yDir.y, 	yDir.z, 	0,
						 zDir.x, 	zDir.y, 	zDir.z, 	0,
						 origPos.x,	origPos.y,	origPos.z,	1]) 

	# initialise tricubic interpolator with signed distance data on default cubic grid

	ip = tricubic(sigDistFieldArray, [gridSubdiv+1, gridSubdiv+1, gridSubdiv+1]) # grid will be initialised in its relative coordinate system [0,0,0] to [gridSubdiv+1, gridSubdiv+1, gridSubdiv+1].

	arguments = (ip, rotMat) # create tuple for arguments passsed to constraints function

	# set constraints function

	cons = ({'type': 'ineq',	# set type to inequality, which means that it is to be non-negative
			 'fun': cons_fun,	# set constraint function
			 'args': arguments	# pass arguments to constrain function
			 })

	# optimization using SLSQP

	res = sp.optimize.minimize(cost_fun, initial_guess, args = arguments, method = 'SLSQP', constraints = cons)

	# get results: res.X = ligament point coordinates, res.fun = relative ligament length

	return res


# ========== constraints function for optimization ==========

def cons_fun(params, ip, rotMat):

	# Input variables:
	#	params = 3D points from cost function
	#	ip = tricubic interpolation function from tricubic.tricubic() for the signed distance data on the cubic grid
	#	rotMat = rotation matrix from ligament coordinate system to default cubic grid coordinate system
	# ======================================== #

	# extract coordinates from params
	
	print(params)

	x = params[0::3]
	y = params[1::3]
	z = params[2::3]

	signDist = []
	pos = []

	# go through each ligament point and check if any intersect with a mesh, i.e., sigDist < 0

	n = len(x)

	for i in range(n-2):

		pos.append([x[i+1], y[i+1], z[i+1]])

		# transform relative grid position into default cubic grid space for tricubic interpolation using rotation matrix

		relPos = getPosInOS(rotMat,[x[i], y[i], z[i]])
		relPos = np.absolute(relPos)
		
		print(relPos) # !! ==== Double check if relative position is accurate, the signDist below is bit odd. Might be accurate if order from orig to ins is reversed ?? ==== !!
		
		# tricubic interpolation of signed distance at postion relative to default cubic grid

		signDist.append(ip.ip([relPos[0], relPos[0], relPos[0]]))
	
	print(pos)
	print(signDist)
	print(min(signDist))
	
	return min(signDist) # return minimal value, if any of the points is inside a mesh it will be negative


# ========== cost function for optimization ==========

def cost_fun(params):

	# Input variables:
	#	params = array of points, i.e., x = [(x_0, y_0, z_0),(x_1, y_1, z_1), ... ,(x_n-1, y_n-1, z_n-1)]. They are, however, flattened into a single array, i.e., [x0, y0, z0, x1, y1, z1, x2, y2, z2, ... , x_n-1, y_n-1, z_n-1], and therefore need to be extracted.
	# ======================================== #
	
	# extract coordinates from params
	
	print(params)

	x = params[0::3]
	y = params[1::3]
	z = params[2::3]
	
	# get number of ligament segments

	n = len(x)-1 
	
	# set value of constant, i.e., the fraction of the ligament 
	
	const = 1/n 
	
	# cost, i.e., ligament length to be minimized
	
	cost = 0
	
	for i in range(n):
		
		# sum up the distances between the idnividiual ligament points, i.e., the length of the individual segments
		
		cost += sqrt(const + (y[i+1] - y[i])**2 + (z[i+1] - z[i])**2)
		
	return cost
    
    
    
    
