#	ligamentCalculationBatch.py
#
#	This script calculates the shortest distance of a ligament from origin to insertion 
#	wrapping around the bone meshes. It is an implementation of the Marai et al., 2004.
#	approach for Autodesk Maya. It creates signed distance fields for the proximal and
#	distal bone meshes, which are then used to approximate the 3D path of each ligament
#	accross them to calculate their lengths.
#
#	Written by Oliver Demuth and Vittorio la Barbera
#	Last updated 15.05.2025 - Oliver Demuth
#
#	SYNOPSIS:
#
#		INPUT params:
#			string  jointName:	Name of the joint centre, i.e. the name of a locator or joint (e.g., 'myJoint' if following the ROM mapping protocol of Manafzadeh & Padian 2018)
#			string  meshes:		Name(s) of the bone meshes (e.g., several individual meshes in the form of ['prox_mesh','dist_mesh'])
#			int	gridSubdiv:	Integer value for the subdivision of the cube, i.e., number of grid points per axis (e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points)
#			float	gridScale:	Float value for the scale factor of the cubic grid (i.e., 1.5 initialises the grid from -1.5 to 1.5)
#			int 	ligSubdiv:	Integer value for the number of ligament points (e.g., 20 will divide the ligament into 20 equidistant segments, see Marai et al., 2004 for details)
#
#		RETURN params:
#			list	pathLengths:	Return value is a list with the path lengths for all ligaments designated as custom attributes in the 'jointName'
#			list 	pathPoints:	Return value is a list of lists with the 3D coordinates of the path points in world space for all ligaments designated as custom attributes in the 'jointName'
#			list 	results: 	Return value is a list of objects of the scipy.optimize.OptimizeResult class. They represent the outputs of the scipy.optimize.minimize() function
#
#
#	IMPORTANT notes:
#		
#	(1) 	Meshes need realtively uniform face areas, otherwise large faces might skew 
#		vertex normals in their direction. It is, therefore, important to extrude 
#		edges around large faces prior to closing the hole at edges with otherwise 
#		acute angles to circumvent this issue (e.g., if meshes have been cut to reduce
#		polycount) prior to executing the Python scripts.
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

import maya.standalone
import maya.api.OpenMaya as om
import maya.cmds as cmds
import numpy as np
import scipy as sp
import functools
import os
import time

from tricubic import tricubic
from maya.api.OpenMaya import MVector, MPoint, MTransformationMatrix
from math import sqrt, floor


################################################
# ========== Maya specific functions ========= #
################################################


# ========== ligament length calculation function ==========
def ligCalc(x, jPos, ipProx, ipDist, rotMat, LigAttributes):

	# Input variables:
	#	x = constant X coordinates for ligament points
	#	jPos = world coordinates of joint
	#	ipProx = proximal signed distance field in tricubic form
	#	ipDist = distal signed distance field in tricubic form
	#	rotMat = array with the transformation matrices of the joint and its parent
	#	LigAttributes = array consisting of names of ligaments
	# ======================================== #

	# create variables

	ligLengths = []

	numPoints = len(x)

	ligArr = np.stack([np.eye(4)] * (numPoints-2), axis = 0)

	# define initual guess condition for optimiser

	initial_guess = np.zeros(2 * numPoints)

	for ligament in LigAttributes:

		# get ligament transformation matrices

		ligRotMat, offset = getLigTransMat(ligament + '_orig', ligament + '_ins', jPos)

		# get ligament specific transformation matrix to cubic grid
		
		relRotMat = []
		relRotMat.append(np.dot(ligRotMat,rotMat[0]))
		relRotMat.append(np.dot(ligRotMat,rotMat[1]))

		# minimise ligament length through optimiser

		res = ligLengthOptMin(x, initial_guess, ipProx, ipDist, relRotMat, ligArr)

		# correct relative ligament length by linear distance between origin and insertion to get actual ligament length

		if res.success: # check if optimiser terminated successfully
			ligLengths.append(res.fun * offset)
		else:
			ligLengths.append(-offset) # optimiser was unsuccessful: mark as outlier (negative Euclidean distance between origin and insertion)

	return ligLengths


# ========== signed distance field per joint function ==========

def sigDistField(jointName, meshes, subdivision, gridScale):

	# Input variables:
	#	jointName = name of joint centre (represented by object in Maya scene; e.g. joint or locator)
	#	meshes = name(s) of the mesh(es) for which the signed distance field is calculated
	#	subdivision = number of elements per axis (e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points)
	#	gridScale = scale factor for the cubic grid dimensions
	# ======================================== #

	# get joint centre position

	jPos = getWSPos(jointName)
	jDag = dagObjFromName(jointName)[1]
	jInclTransMat = MTransformationMatrix(jDag.inclusiveMatrix()) # world transformation matrix of joint
	jExclTransMat = MTransformationMatrix(jDag.exclusiveMatrix()) # world transformation matrix of parent of joint

	# get number and names of ligaments

	LigAttributes = cmds.listAttr(jointName, ud = True) # get user defined attributes of 'jointName' (i.e. the float attributes that will contain the ligament lengths)

	# cycle through ligaments and extract their information

	distArr = []

	for ligament in LigAttributes:

		# get positions of points of interest

		oPos = getWSPos(ligament + '_orig')
		iPos = getWSPos(ligament + '_ins')

		# get distances from origin and insertion to joint centre
				
		distArr.append((jPos - oPos).length()) # Euclidean distance from origin to joint centre (i.e., scale factor for grid point positions)
		distArr.append((jPos - iPos).length()) # Euclidean distance from insertion to joint centre (i.e., scale factor for grid point positions)

	maxDist = max(distArr) # maximal distance from joint centre to ligament attachment

	# normalize matrices by maxDist

	jInclTransMat.setScale([maxDist,maxDist,maxDist],4) # set scale in world space (om.MSpace.kWorld = 4)
	jExclTransMat.setScale([maxDist,maxDist,maxDist],4) # set scale in world space (om.MSpace.kWorld = 4)

	# get rotation matrices

	rotMat = []
	rotMat.append(np.array(jExclTransMat.asMatrix()).reshape(4,4)) # parent rotMat (prox) as numpy 4x4 array
	rotMat.append(np.array(jInclTransMat.asMatrix()).reshape(4,4)) # child rotMat (dist) as numpy 4x4 array

	# cycle through all meshes

	if len(meshes) > 2:
		error('Too many meshes in array. Please specify only the distal and proximal bone mesh in the mesh array.')
	elif len(meshes) < 2:
		error('Too few meshes specified. Please specify TWO meshes in the mesh array.')
	
	SDFs = []
	for i, mesh in enumerate(meshes):
		meshSigDist = sigDistMesh(mesh, rotMat[i], subdivision, gridScale) # get signed distance field for each mesh

		# initialise tricubic interpolator with signed distance data on default cubic grid
		
		SDFs.append(tricubic(meshSigDist.tolist(), list(meshSigDist.shape))) # grid will be initialised in its relative coordinate system from [0,0,0] to [subdivision+1, subdivision+1, subdivision+1].
	
	return SDFs, LigAttributes, maxDist


# ========== signed distance field per mesh function ==========

def sigDistMesh(mesh, rotMat, subdivision, gridScale):

	# Input variables:
	#	mesh = name of the mesh for which the signed distance field is calculated
	#	rotMat = transformation matrix of parent (joint) of the mesh
	#	subdivision = number of elements per axis, e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points
	# 	gridScale = scale factor for the cubic grid dimensions
	# ======================================== #

	# get dag paths

	dag = dagObjFromName(mesh)[1]

	# create MObject

	shape = dag.extendToShape()
	mObj = shape.node()

	# get the meshs transformation matrix

	meshMat = dag.inclusiveMatrix()
	meshMatInv = np.linalg.inv(np.array(meshMat).reshape(4,4)) # get inverse of mesh matrix as numpy 4x4 array

	# create the intersector

	polyIntersect = om.MMeshIntersector()
	polyIntersect.create(mObj,meshMat)
	ptON = om.MPointOnMesh()

	# create 3D grid

	elements = np.linspace(-gridScale, gridScale, num = subdivision + 1, endpoint=True, dtype=float)

	points = np.array([[x, y, z] for x in elements
				     for y in elements
				     for z in elements])

	# get coordinates and transform them into matrices

	gridArr = np.stack([np.eye(4)] * points.shape[0], axis = 0)
	gridArr[:,3,0:3] = points # append coordinates to rotation matrix array

	# calculate position of vertices relative to cubic grid

	gridWSArr = np.dot(gridArr,rotMat)
	gridWSList = gridWSArr[:,3,0:3].tolist()

	# go through grid points and calculate signed distance for each of them

	P = np.zeros((points.shape[0],3))
	N = np.zeros((points.shape[0],3))

	for i, gridPoint in enumerate(gridWSList):
		ptON = polyIntersect.getClosestPoint(MPoint(gridPoint)) # get point on mesh
		P[i,:] = [ptON.point.x, ptON.point.y, ptON.point.z] # coordinates of point on mesh in mesh coordinate system
		N[i,:] = [ptON.normal.x, ptON.normal.y, ptON.normal.z] # surface normal at point on mesh

	# get vectors from gridPoints to points on mesh

	diff = np.dot(gridWSArr,meshMatInv)[:,3,0:3] - P

	# get distances from gridPoints to points on mesh

	dist = np.linalg.norm(diff, axis = 1)

	# normalise vector to get direction from gridPoints to points on mesh

	normDiff = diff/dist.reshape(-1,1)

	# calculate dot product between the normal at ptON and vector to check if point is inside or outside of mesh

	dot = np.sum(N * normDiff, axis = 1)

	# get sign from dot product for distance

	signDist = dist * np.sign(dot)

	return signDist.reshape(subdivision + 1, subdivision + 1, subdivision + 1) # convert signed distance array into cubic grid format


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
				
	LigDir = iPos - oPos
	JointDir = jPos - oPos

	offset = LigDir.length() # linear distance from origin to insertion, i.e., scale factor for grid point positions

	# calculate planes relative to ligaments

	uDir = LigDir.normal() * offset
	wDir = (JointDir.normal() ^ uDir).normal() * offset # cross product to get z axis
	vDir = (wDir ^ uDir).normal() * offset # cross product to get y axis 

	# get transformation matrix from liagament plane directions

	transMat = np.array([[uDir.x, uDir.y, uDir.z, 0],
			     [vDir.x, vDir.y, vDir.z, 0],
			     [wDir.x, wDir.y, wDir.z, 0],
			     [oPos[0],oPos[1],oPos[2],1]]) # centre position around origin
		
	return transMat, offset 


# ========== get ws pos ==========

def getWSPos(name):

	# Input variables:
	#	name = string representing object name
	# ======================================== #
	
	dag = dagObjFromName(name)[1]
	
	return MTransformationMatrix(dag.inclusiveMatrix()).translation(4) # extract translation in world space from transformation matrix (3x faster than xform; om.MSpace.kWorld = 4)


# ========== get dag path function ==========

def dagObjFromName(name):

	# Input variables:
	#	name = string representing object name
	# ======================================== #
	
	sel = om.MSelectionList()
	sel.add(name)

	return sel.getDependNode(0), sel.getDagPath(0)


################################################
# ============ optimiser functions =========== #
################################################


# ========== ligament optimiser function ==========

def ligLengthOptMin(x, initial_guess, ipProx, ipDist, rotMat, ligArr):

	# Input variables:
	#	x = constant X coordinates for ligament points
	#	initial_guess = initual guess condition for optimiser
	#	ipProx = proximal signed distance field in tricubic form
	#	ipDist = distal signed distance field in tricubic form
	#	rotMat = array with the transformation matrices of the joint and its parent
	#	ligArr = 3D array of ligament point transformation matrices for fast computation of relative coordinates
	# ======================================== #

	# create tuple for arguments passed to both constraints and cost functions

	arguments = (x, ipProx, ipDist, rotMat, ligArr)

	# set constraints functions

	cons = ({'type': 'ineq',		# set type to inequality, which means that it is to be non-negative
		 'fun': sigDist_cons_fun,	# set constraint function
		 'args': arguments},		# pass arguments to constrain function
		{'type': 'ineq',		# set type to inequality, which means that it is to be non-negative
		 'fun': path_cons_fun,		# set constraint function
		 'args': arguments})		# pass arguments to constrain function

	# set bounds

	length_init = len(initial_guess)

	boundsList = [(-1,1)] * length_init # set y and z coordinate boundaries to the euclidean distance between origin and insertion
	boundsList[0] = boundsList[1] = boundsList[length_init - 2] = boundsList[length_init - 1] = (0,0) # set y and z bounds of origin and insertion to zero
	bnds = tuple(boundsList)

	# set options

	options = {"maxiter": 1000} # if it doesn't solve within 1000 iterations it usually won't solve

	# optimization using SLSQP

	res = sp.optimize.minimize(cost_fun, initial_guess, args = arguments, bounds = bnds, method = 'SLSQP', constraints = cons, options = options)

	# get results: res.x = ligament point coordinates, res.fun = relative ligament length

	return res


# ========== signed distance field constraint function ==========

def sigDist_cons_fun(params, x, ipProx, ipDist, rotMat, ligArr):

	# Input variables:
	#	params = array of Y and Z coordinates of ligament points, i.e., params = [(y_0, z_0),(y_1, z_1), ... ,(y_n-1, z_n-1)]. They are, however, flattened into a single array, i.e., [y0, z0, y1, z1, y2, z2, ... , y_n-1, z_n-1], and therefore need to be extracted.
	#	x = constant X coordinates for ligament points
	#	ipProx = tricubic interpolation function from tricubic.tricubic() for the signed distance data on the proximal cubic grid
	#	ipDist = tricubic interpolation function from tricubic.tricubic() for the signed distance data on the distal cubic grid
	#	rotMat = array with the transformation matrices of the joint and its parent
	#	ligArr = 3D array of ligament point transformation matrices for fast computation of relative coordinates
	# ======================================== #

	# assign values to 3D array for matrix multiplications

	ligArr[:,3,0] = x[1:-1] # x coordinates, skip first and last ligament coords (i.e., origin and insertion)
	ligArr[:,3,1] = params[0::2][1:-1] # y coordinates, skip first and last ligament coords (i.e., origin and insertion)
	ligArr[:,3,2] = params[1::2][1:-1] # z coordinates, skip first and last ligament coords (i.e., origin and insertion)

	relPosProxArr = np.dot(ligArr,rotMat[0])[:,3,0:3].tolist() # get points in proximal cubic grid coordinates
	relPosDistArr = np.dot(ligArr,rotMat[1])[:,3,0:3].tolist() # get points in distal cubic grid coordinates

	signDist = [ipProx.ip(point) for point in relPosProxArr] # proximal signed distance
	signDist.extend([ipDist.ip(point) for point in relPosDistArr]) # distal signed distance
		
	return min(signDist)


# ========== path constraint function ==========

def path_cons_fun(params, x, ipProx, ipDist, rotMat, ligArr):

	# Input variables:
	#	params = array of Y and Z coordinates of ligament points, i.e., params = [(y_0, z_0),(y_1, z_1), ... ,(y_n-1, z_n-1)].
	#	x = constant X coordinates for ligament points. Passed through args, not part of this constraint function
	#	ipProx = tricubic interpolation function from tricubic.tricubic() for the signed distance data on the proximal cubic grid. Passed through args, not part of this constraint function
	#	ipDist = tricubic interpolation function from tricubic.tricubic() for the signed distance data on the distal cubic grid. Passed through args, not part of this constraint function
	#	rotMat = array with the transformation matrices of the joint and its parent. Passed through args, not part of this constraint function
	#	ligArr = 3D array of ligament point transformation matrices for fast computation of relative coordinates. Passed through args, not part of this constraint function
	# ======================================== #

	# calculate squared offset between subsequent points

	offset = sum(np.diff([params[0::2],params[1::2]])**2) # y = params[0::2], z = params[1::2]

	return(1.7 / len(offset)) - sqrt(max(offset)) # perpendicular offset can be maximally 1.7 times the distance between path segment along X-axis (i.e., arctan(offset/dist) <= ~60°)


# ========== cost function for optimisation ==========

def cost_fun(params, x, ipProx, ipDist, rotMat, ligArr):

	# Input variables:
	#	params = array of Y and Z coordinates of ligament points, i.e., params = [[y_0, z_0],[y_1, z_1], ... ,[y_n-1, z_n-1]]. They are, however, flattened into a single array, i.e., [y0, z0, y1, z1, y2, z2, ... , y_n-1, z_n-1], and therefore need to be extracted.
	#	x = constant X coordinates for ligament points.
	#	ipProx = tricubic interpolation function from tricubic.tricubic() for the signed distance data on the proximal cubic grid. Passed through args, not part of cost function
	#	ipDist = tricubic interpolation function from tricubic.tricubic() for the signed distance data on the distal cubic grid. Passed through args, not part of cost function
	#	rotMat = array with the transformation matrices of the joint and its parent. Passed through args, not part of cost function
	#	ligArr = 3D array of ligament point transformation matrices for fast computation of relative coordinates. Passed through args, not part of cost function
	# ======================================== #
		
	return sum(np.sqrt(sum(np.diff([x,params[0::2],params[1::2]])**2))) # length of ligament from origin to insertion; y = params[0::2], z = params[1::2]


################################################
# ========= multiprocessing functions ======== #
################################################


# ========== Maya instancing function ==========

def MayaInstance(function):

	# Input variables:
	#	function = The function to be wrapped inside a maya.standalone instance
	# ======================================== #

	@functools.wraps(function)
	def instance_wrapper(queue,args):

		# Input variables:
		#	queue = The queue of files to be processed
		#	args = arguments to be passed to internal functions
		# ======================================== #

		# initialise Maya

		maya.standalone.initialize(name='python')

		# get one of remaining elements of the queue

		while not queue.empty():
			function(queue.get(),args)

	return instance_wrapper # return wrapped function


# ========== processing Maya file function ==========

@MayaInstance
def processMayaFiles(filePath,args):

	# Input variables:
	#	filePath = The path to a file to be processed
	#	args = arguments to be passed to ligament calculation functions
	# ======================================== #

	# supress error messages

	cmds.scriptEditorInfo(sw=True,se=True)

	# get file name 

	fileName = filePath.split('/')[-1]

	print('Processing file: ', fileName)

	# open Maya scene and initialise calculations 

	cmds.file(filePath, open=True, force=True)

	# extract arguments

	[jointName, meshes, gridSubdiv, gridScale, ligSubdiv, FrameInterval, outDir] = args


	# ==== calculate signed distance fields ====


	start = time.time()

	print('Calculating signed distance fields for {}...'.format(fileName))
	
	cmds.currentTime(0)
	cmds.move(0,0,0, jointName, localSpace=True)
	cmds.rotate(0,0,0,jointName)
	
	SDF, LigAttributes, gridSize = sigDistField(jointName, meshes, gridSubdiv, gridScale))
	
	# get inverse of rotation matrix for default cubic grid coordinate system

	gridVec = 2 * gridScale / subdivision
	gridRotMat = np.linalg.inv(np.array([[gridVec, 0, 0, 0], # x direction
					     [0, gridVec, 0, 0], # y direction
					     [0, 0, gridVec, 0], # z direction
					     [-gridScale, -gridScale, -gridScale, 1]])) # origin

	mid = time.time()

	print('Signed distance fields calculated in {0:.3f} seconds for {1}!'.format(mid - start,fileName))


	# ==== calculate ligament lengths ====

	
	# get total number of keyed frames from 'jointName', i.e., max number of frames to be calculated

	if FrameInterval == None:
		frames = cmds.keyframe(jointName, attribute='rotateX', query=True, keyframeCount=True)
	else:
		frames = FrameInterval

	# initialise results array
	
	ligRes =[] 
	ligRes.append(LigAttributes)

	# initialise constant x coords

	x = np.linspace(0.0, 1.0, num = ligSubdiv + 1, endpoint=True)

	# set to starting frame 

	cmds.currentTime(1)

	# go through each frame and calculate ligament lengths 

	updateSwitch = True

	print('{0} ligament calculation progress: {1:.0f}%.'.format(fileName,0)) 

	for frame in range(frames):

		# get joint centre position

		jPos = getWSPos(jointName)
		jDag = dagObjFromName(jointName)[1]
		jInclTransMat = MTransformationMatrix(jDag.inclusiveMatrix()) # world transformation matrix of joint
		jExclTransMat = MTransformationMatrix(jDag.exclusiveMatrix()) # world transformation matrix of parent of joint

		# normalize matrices by gridSize

		jInclTransMat.setScale([gridSize,gridSize,gridSize],4) # set scale in world space (om.MSpace.kWorld = 4)
		jExclTransMat.setScale([gridSize,gridSize,gridSize],4) # set scale in world space (om.MSpace.kWorld = 4)

		# get rotation matrices

		jExclMatInv = np.linalg.inv(np.array(jExclTransMat.asMatrix()).reshape(4,4))
		jInclMatInv = np.linalg.inv(np.array(jInclTransMat.asMatrix()).reshape(4,4))

		rotMat = []
		rotMat.append(np.dot(jExclMatInv,gridRotMat)) # inverse of parent rotMat (prox)
		rotMat.append(np.dot(jInclMatInv,gridRotMat)) # inverse of child rotMat (dist)

		# calculate the length of the ligaments for the current frame and append it to results array

		ligRes.append(ligCalc(x, jPos, SDF[0], SDF[1], rotMat, LigAttributes)) # add row of ligament lengths to result array

		# update progress
		
		percent = (100 * (frame + 1)) // frames

		if updateSwitch:
			previous = percent
			updateSwitch = False

		if percent > previous:
			ETA = time.strftime("%H h %M min %S sec", time.gmtime((100 - percent) * (time.time() - mid) / percent))
			print('{0} ligament calculation progress: {1:.0f}%. Estimated completion in: {2}'.format(fileName,percent,ETA)) 
			updateSwitch = True

		# go to next frame

		cmds.currentTime(frame + 2)
		
	
	# ==== save results and print to file ====


	# define outpule file name and path

	namei = fileName.replace('.mb','')
	fname = outDir + '/' + namei + '.csv'
	
	# write to output file

	with open(fname, 'w') as filehandle:
		for listitem in ligRes:
			s = ",".join(map(str, listitem))
			filehandle.write('%s\n' % s)
	
	# shut down the Maya scene

	cmds.file(modified=0) 

	# print simulation time for each Maya scene

	end = time.time()
	convert = time.strftime("%H hours %M min %S seconds", time.gmtime(end - mid))
	print('Ligament calculation for {0} done in {1}!'.format(fileName,convert))


