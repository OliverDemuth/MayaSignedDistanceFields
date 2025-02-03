#	ROMmapperBatch.py
#
#	This script optimises contact-based positions across a set of rotational poses for
#	a set of bone meshes. It is an implementation of the Marai et al., 2006 and Lee
#	et al. 2023 approach for Autodesk Maya. It creates signed distance fields for the
#	proximal and distal bone meshes which are then used to caculate intersections
#	between the meshes and the distance between them. 
#
#	Written by Oliver Demuth 
#	Last updated 01.02.2025 - Oliver Demuth
#
#	SYNOPSIS:
#
#		INPUT params:
#			string	jointName:		Name of the joint centre, i.e. the name of a locator or joint (e.g., 'myJoint' if following the ROM mapping protocol of Manafzadeh & Padian 2018)
#			string	meshes:			Names of the two bone meshes, i.e., several individual meshes representing the bones (e.g., in the form of ['prox_mesh','dist_mesh'])
#			string	congruencyMeshes:	Names of the meshes to check articular congruency, i.e., several individual meshes (e.g., in the form of ['prox_art_surf','dist_art_surf'])
#			string	fittedShape:		Name of the shape fitted to the proximal articular surface (e.g., in the form 'prox_fitted_sphere') used for proximity estimation
#			int	gridSubdiv:		Integer value for the subdivision of the cube, i.e., number of grid points per axis (e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points)
#			float	gridSize:		Float value indicating the size of the cubic grid, i.e., the length of a side (e.g., 10 will result ing a cubic grid with the dimensions 10 x 10 x 10)
#			float	thickness:		Float value indicating the thickness value which correlates with the joint spacing
#
#		RETURN params:
#			array 	coords:			Return value is a an array of 3D coordinates of the distal bone relative to the proximal one
#			boolean	viable:			Return value is a boolean whether the rotational poses are viable or inviable (i.e, intersection or disarticulation of the bone meshes)
#
#
#	IMPORTANT notes:
#
#	(1) Meshes need realtively uniform face areas, otherwise large faces might skew 
#	    vertex normals in their direction. It is, therefore, important to extrude 
#	    edges around large faces prior to closing the hole at edges with otherwise 
#	    acute angles to circumvent this issue, e.g., if meshes have been cut to reduce
#	    polycount, prior to executing the Python scripts.
#
#	(2) For each bone two sets of meshes are required, the articular surface(s) and
#	    the bone mesh
#
#	(3) This script requires several modules for Python, see README file. Make sure to
#	    have the following external modules installed for the mayapy application:
#
#		- 'numpy' 	NumPy:			https://numpy.org/about/
#		- 'scipy'	SciPy:			https://scipy.org/about/
#		- 'tricubic' 	Daniel Guterding: 	https://github.com/danielguterding/pytricubic
#				
#	    For further information regarding them, please check the website(s) referenced 
#	    above.


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
from math import ceil


################################################
# ========== Maya specific functions ========= #
################################################


# ========== signed distance field per joint function ==========

def sigDistField(jointName, meshes, subdivision, size):

	# Input variables:
	#	jointName = name of joint centre (represented by object, e.g. joint or locator, in Maya scene)
	#	meshes = name(s) of the mesh(es) for which the signed distance field is calculated
	#	subdivision = number of elements per axis, e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points
	#	size = grid size of the cubic grid
	# ======================================== #

	# get joint centre position

	jDag = dagObjFromName(jointName)[1]
	jInclTransMat = MTransformationMatrix(jDag.inclusiveMatrix()) # world transformation matrix of joint
	jExclTransMat = MTransformationMatrix(jDag.exclusiveMatrix()) # world transformation matrix of parent of joint

	# normalize matrices by size

	jInclTransMat.setScale([size,size,size],4) # set scale in world space (om.MSpace.kWorld = 4)
	jExclTransMat.setScale([size,size,size],4) # set scale in world space (om.MSpace.kWorld = 4)

	# get rotation matrices

	rotMat = []
	rotMat.append(np.array(jExclTransMat.asMatrix()).reshape(4,4)) # parent rotMat (prox) as numpy 4x4 array
	rotMat.append(np.array(jInclTransMat.asMatrix()).reshape(4,4)) # child rotMat (dist) as numpy 4x4 array

	# cycle through all meshes

	if len(meshes) > 2:
		error('Too many meshes in array. Please specify only the distal and proximal bone mesh in the mesh array.')
	elif len(meshes) < 2:
		error('Too few meshes specified. Please specify TWO meshes in the mesh array.')
	
	sigDistances = []
	for i, mesh in enumerate(meshes):
		meshSigDist, osPoints = sigDistMesh(mesh, rotMat[i], subdivision) # get signed distance field for each mesh
		sigDistances.append(np.array(meshSigDist).reshape(subdivision + 1, subdivision + 1, subdivision + 1))
	
	return sigDistances, osPoints, rotMat


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

	meshMat = dag.inclusiveMatrix()
	meshMatInv = np.linalg.inv(np.array(meshMat).reshape(4,4)) # get inverse of mesh matrix as numpy 4x4 array

	# create the intersector

	polyIntersect = om.MMeshIntersector()
	polyIntersect.create(mObj,meshMat)
	ptON = om.MPointOnMesh()

	# create 3D grid

	elements = np.linspace(-1, 1, num = subdivision + 1, endpoint=True, dtype=float)
	
	points = np.array([[x, y, z] for x in elements
				     for y in elements 
				     for z in elements])

	# get coordinates and transform them into matrices

	gridArr = np.stack([np.eye(4)] * points.shape[0], axis = 0)
	gridArr[:,3,0:3] = points # append coordinates to rotation matrix array

	# calculate position of vertices relative to cubic grid

	gridWSArr = np.dot(gridArr,rotMat)
	gridWSList = gridWSArr[:,3,0:3].tolist()
	localList = np.dot(gridWSArr,meshMatInv)[:,3,0:3].tolist()

	# go through grid points and calculate signed distance for each of them

	signedDist = []

	for i in range(points.shape[0]):
		
		# find closest points on mesh and get their distance to the grid points
							  
		ptON = polyIntersect.getClosestPoint(MPoint(gridWSList[i])) # get point on mesh
		
		# get vector from localPoint to ptON

		diff = (MVector(ptON.point) -  MVector(localList[i]))

		# get distance from localPoint to ptON    
	
		dist = diff.length()

		# calculate dot product between the normal at ptON and vector to check if point is inside or outside of mesh
		
		dot = MVector(ptON.normal).normal() * diff.normal() 

		# get sign for distance

		if dot >= 0: # point is inside mesh
			signedDist.append(-dist)
		else: # point is outside of mesh
			signedDist.append(dist)

	return signedDist, points.tolist()


# ========== get vertices position in reference frame ==========

def relVtcPos(mesh, rotMat):

	# Input variables:
	#	mesh = name of the mesh for which the signed distance field is calculated
	#	rotMat = transformation matrix of parent (joint) of the mesh
	# ======================================== #

	# get dag paths

	dag = dagObjFromName(mesh)[1]

	# get vertices in global coordinate system

	vertices = np.array(om.MFnMesh(dag).getPoints(4)) # get world position of all vertices (om.MSpace.kWorld = 4)

	# get coordinates and transform them into matrices

	vtxArr = np.stack([np.eye(4)] * vertices.shape[0], axis = 0)
	vtxArr[:,3,:] = vertices # append coordinates to rotation matrix array

	return np.dot(vtxArr,np.linalg.inv(rotMat))[:,3,:] # calculate new coordinates and extract them


# ========== get dag path function ==========

def dagObjFromName(name):

	# Input variables:
	#	name = string representing object name
	# ======================================== #
	
	sel = om.MSelectionList()
	sel.add(name)

	return sel.getDependNode(0), sel.getDagPath(0)


# ========== get mean distance from center ==========

def meanRad(mesh):

	# Input variables:
	#	mesh = name of the mesh in scene
	# ======================================== #

	# get dag paths

	dag = dagObjFromName(mesh)[1]
	shape = dag.extendToShape()

	transMat = np.array(dag.inclusiveMatrix()).reshape(4,4)

	# get mesh object

	fnMesh = om.MFnMesh(shape)

	fnMeshMat = np.eye(4)
	fnMeshMat[3,:] = np.array(fnMesh.boundingBox.center)

	centerPos = np.dot(fnMeshMat,transMat)[3,:]

	# get world position of all vertices (om.MSpace.kWorld = 4)

	vertices = np.array(fnMesh.getPoints(4))

	# substract center position from vertices and return mean distance

	return np.linalg.norm((vertices - centerPos), axis = 1).mean() 


################################################
# ============ optimiser functions =========== #
################################################


# ========== check viability function ==========

def optimisePosition(proxArr, distArr, ipProx, ipDist, gridRotMat, rotMat, thickness, initial_guess, paramCoords):

	# Input variables:
	#	proxArr = 3D array of proximal articular surface vertex transformation matrices for fast computation of relative coordinates 
	#	distArr = 3D array of distal articular surface vertex transformation matrices for fast computation of relative coordinates 
	#	ipProx = proximal signed distance field in tricubic form
	#	ipDist = distal signed distance field in tricubic form
	#	gridRotMat = rotation matrix of default cubic grid
	#	rotMat = array with the transformation matrices of the joint and its parent
	#	thickness = thickness measure correlated with joint spacing
	#	initial_guess = initual guess condition for optimiser
	#	paramCoords = 4x4 identity matrix for matrix multiplications
	# ======================================== #

	# optimise translation for specific rotational pose

	results = posOptMin(proxArr, distArr, ipProx, ipDist, gridRotMat, rotMat, thickness, initial_guess, paramCoords)

	# check if optimisation was successful

	if results.success: 

		diff = MVector(results.x).length() # get offset from glenoid centre

		# check for disarticulation 
						  
		if diff < (1.05 * thickness * 2): # first crudely (if distal ACS is more than 5% beyond radius of fitted proximal shape)

			# get coordinates of results and transform them into transformation matrix coords

			paramCoords[3,0:3] = results.x

			# get transformation matrix

			transMat = rotMat[1] # transformation matrix
			transMat[3,0:3] = np.dot(paramCoords,rotMat[2])[3,0:3] # append result world space coordinates to transformation matrix
			transMatInv = np.linalg.inv(transMat) # get inverse of transformation matrix

			# calculate position of vertices relative to cubic grid

			vtcRelArr = np.dot(proxArr,np.dot(np.dot(rotMat[0],transMatInv),gridRotMat))[:,3,0:3].tolist()
			signDist = [ipDist.ip(vtx) for vtx in vtcRelArr]
			avgdist = sum(signDist) / len(vtcRelArr)

			# if disarticulated or any points penetrate meshes the position becomes inviable

			if avgdist < (1.05 * thickness) and all(dist > 0 for dist in signDist):
				viable = True
			else:
				viable = False
		else:
			viable = False		
	else:
		viable = False

	# gather results

	return results.x, viable


# ========== translation optimisation ==========

def posOptMin(proxArr, distArr, ipProx, ipDist, gridRotMat, rotMat, thickness, initial_guess, paramCoords):

	# Input variables:
	#	proxArr = 3D array of proximal articular surface vertex transformation matrices for fast computation of relative coordinates 
	#	distArr = 3D array of distal articular surface vertex transformation matrices for fast computation of relative coordinates 
	#	ipProx = proximal signed distance field in tricubic form
	#	ipDist = distal signed distance field in tricubic form
	#	gridRotMat = rotation matrix of default cubic grid
	#	rotMat = array with the transformation matrices of the joint and its parent
	#	thickness = thickness measure correlated with joint spacing
	#	initial_guess = initual guess condition for optimiser
	#	paramCoords = 4x4 identity matrix for matrix multiplications
	# ======================================== #

	# create tuple for arguments passed to both constraints and cost functions

	arguments = (proxArr, distArr, ipProx, ipDist, rotMat, gridRotMat, thickness, paramCoords)

	# set constraints functions

	cons = ({'type': 'ineq',	# set type to inequality, which means that it is to be non-negative
		 'fun': cons_fun,	# set constraint function
		 'args': arguments})	# pass arguments to constrain function

	# set bounds

	boundsList = [(-10,10)] * (len(initial_guess)) # set x, y and z coordinate boundaries
	bnds = tuple(boundsList)

	# set options

	options = {"maxiter": 50} # if it doesn't solve within 25 iterations it usually won't solve. Any more than 50 will just increase computational time without much benefit

	# optimization using SLSQP

	res = sp.optimize.minimize(cost_fun, initial_guess, args = arguments, bounds = bnds, method = 'SLSQP', constraints = cons, options = options)

	# get results: res.x = joint centre point coordinates, res.fun = mean articular distance - thickness squared + variance

	return res


# ========== signed distance field constraint function ==========

def cons_fun(params, proxArr, distArr, ipProx, ipDist, rotMat, gridRotMat, thickness, paramCoords):

	# Input variables:
	#	params = array of X, Y and Z coordinates of distal element position
	#	proxArr = 3D array of proximal articular surface vertex transformation matrices for fast computation of relative coordinates 
	#	distArr = 3D array of distal articular surface vertex transformation matrices for fast computation of relative coordinates 
	#	ipProx = tricubic interpolation function from tricubic.tricubic() for the signed distance data on the proximal cubic grid
	#	ipDist = tricubic interpolation function from tricubic.tricubic() for the signed distance data on the distal cubic grid
	#	rotMat = array with the transformation matrices of the joint and its parent
	#	gridRotMat = rotation matrix of default cubic grid coordinate system
	#	thickness = thickness measure correlated with joint spacing. Passed through args, not part of this constraint function
	#	paramCoords = 4x4 identity matrix for matrix multiplications
	# ======================================== #

	# extract coordinates from paramsand transform them into transformation matrix coords

	paramCoords[3,0:3] = params

	# get transformation matrix

	transMat = rotMat[1] # transformation matrix
	transMat[3,0:3] = np.dot(paramCoords,rotMat[2])[3,0:3] # append result world space coordinates to transformation matrix
	transMatInv = np.linalg.inv(transMat) # get inverse of transformation matrix

	# calculate position of vertices relative to cubic grid

	proxVtcRelArr = np.dot(proxArr,np.dot(np.dot(rotMat[0],transMatInv),gridRotMat))[:,3,0:3].tolist()
	distVtcRelArr = np.dot(distArr,np.dot(np.dot(transMat,rotMat[3]),gridRotMat))[:,3,0:3].tolist()

	# go through each proximal and distal articular surface point and check if any of them intersect with a mesh (i.e., signDist < 0)

	signDist = [min([ipDist.ip(vtx) for vtx in proxVtcRelArr]), # minimal proximal signed distance
		    min([ipProx.ip(vtx) for vtx in distVtcRelArr])] # minimal distal signed distance

	return min(signDist) # return minimal value, if any of the points are inside a mesh it will be negative


# ========== cost function for optimisation ==========

def cost_fun(params, proxArr, distArr, ipProx, ipDist, rotMat, gridRotMat, thickness, paramCoords):

	# Input variables:
	#	params = array of X, Y and Z coordinates of distal element position
	#	proxArr = 3D array of proximal articular surface vertex transformation matrices for fast computation of relative coordinates 
	#	distArr = 3D array of distal articular surface vertex transformation matrices for fast computation of relative coordinates. Passed through args, not part of this constraint function
	#	ipProx = tricubic interpolation function from tricubic.tricubic() for the signed distance data on the proximal cubic grid. Passed through args, not part of cost function
	#	ipDist = tricubic interpolation function from tricubic.tricubic() for the signed distance data on the distal cubic grid
	#	rotMat = array with the transformation matrices of the joint and its parent
	#	gridRotMat = rotation matrix of default cubic grid coordinate system
	#	thickness = thickness measure correlated with joint spacing
	#	paramCoords = 4x4 identity matrix for matrix multiplications
	# ======================================== #
	
	# extract coordinates from params and transform them into transformation matrix coords

	paramCoords[3,0:3] = params

	# get transformation matrix

	transMat = rotMat[1] # transformation matrix
	transMat[3,0:3] = np.dot(paramCoords,rotMat[2])[3,0:3] # append result world space coordinates to transformation matrix
	transMatInv = np.linalg.inv(transMat) # get inverse of transformation matrix

	# calculate position of vertices relative to cubic grid

	vtcRelArr = np.dot(proxArr,np.dot(np.dot(rotMat[0],transMatInv),gridRotMat))[:,3,0:3].tolist()

	# calculate mean distance 

	signDist = [ipDist.ip(vtx) for vtx in vtcRelArr]
	meanDist = sum(signDist)/len(signDist)

	# calculate variance

	var = sum([(dist - meanDist) ** 2 for dist in signDist]) / len(signDist)

	return (meanDist - thickness) ** 2 + var # minimising variance enforces largest possible joint concruency


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

	[jointName, meshes, congruencyMeshes, fittedShape, gridSubdiv, interval, outDir] = args


	# ==== calculate signed distance fields ====


	start = time.time()

	print('Calculating signed distance fields for {}...'.format(fileName))

	# reset joint

	cmds.currentTime(0)
	cmds.move(0,0,0, jointName, localSpace=True)
	cmds.rotate(0,0,0,jointName)

	# get gridsize from glenoid sphere radius

	sphereRad = meanRad(fittedShape)
	thickness = sphereRad/2
	gridSize = 8 * sphereRad

	sigDistFieldArray, localPoints, initialRotMat = sigDistField(jointName, meshes, gridSubdiv, gridSize)

	# calculate relative position of articular surfaces

	proxCoords = relVtcPos(congruencyMeshes[0], initialRotMat[0])
	distCoords = relVtcPos(congruencyMeshes[1], initialRotMat[1])

	# get coordinates and transform them into 3D matrix arrays

	identityMat = np.eye(4)

	proxArr = np.stack([identityMat] * proxCoords.shape[0], axis = 0)
	proxArr[:,3,:] = proxCoords # append coordinates to 3D rotation matrix array

	distArr = np.stack([identityMat] * distCoords.shape[0], axis = 0)
	distArr[:,3,:] = distCoords # append coordinates to 3D rotation matrix array

	# get dimensions of cubic grids

	dims = sigDistFieldArray[0].shape
	proxSigDistList = sigDistFieldArray[0].tolist()
	distSigDistList = sigDistFieldArray[1].tolist()

	# initialise tricubic interpolator with signed distance data on default cubic grid

	ipProx = tricubic(proxSigDistList, [dims[0], dims[1], dims[2]]) # grid will be initialised in its relative coordinate system from [0,0,0] to [gridSubdiv+1, gridSubdiv+1, gridSubdiv+1].
	ipDist = tricubic(distSigDistList, [dims[0], dims[1], dims[2]]) # grid will be initialised in its relative coordinate system from [0,0,0] to [gridSubdiv+1, gridSubdiv+1, gridSubdiv+1].

	# get corner points of cubic grids (both grids are set up identically)

	origPos = MVector(localPoints[0])
	zVecPos = MVector(localPoints[dims[1] - 1])
	yVecPos = MVector(localPoints[dims[1] * (dims[2] - 1)])
	xVecPos = MVector(localPoints[dims[1] * dims[2] * (dims[0] - 1)])

	# get direction vectors to cubic grid corners and normalize by number of grid subdivisions

	xDir = (xVecPos - origPos) / (dims[0] - 1)
	yDir = (yVecPos - origPos) / (dims[1] - 1)
	zDir = (zVecPos - origPos) / (dims[2] - 1)

	# get rotation matrix of default cubic grid coordinate system

	gridRotMat = np.linalg.inv(np.array([[xDir.x, xDir.y, xDir.z, 0],
					     [yDir.x, yDir.y, yDir.z, 0],
					     [zDir.x, zDir.y, zDir.z, 0],
					     [origPos.x, origPos.y, origPos.z, 1]])) 

	mid = time.time()

	print('Signed distance fields calculated in {0:.3f} seconds for {1}!'.format(mid - start,fileName))


	# ==== initialise variables and precalculations ====


	# create 3D grid for rotations

	xRots = zRots = np.arange(-180, ceil(180+interval/2), interval, dtype = int)
	yRots = np.arange(-90, ceil(90+interval/2), interval, dtype = int)

	rotations = [[x, y, z] for x in xRots  # length along x
			       for y in yRots  # length along y
			       for z in zRots] # length along z	       

	frames = len(rotations)

	# initialise results array

	transRes =[] 

	# get dag path for joint

	jDag = dagObjFromName(jointName)[1]

	# get joint exclusive transformation matrix (parent)

	jExclMat = jDag.exclusiveMatrix() # world rotation matrix of parent joint
	jExclNPMat = np.array(jExclMat).reshape(4,4) # convert into numpy 4x4 array

	jExclTransMat = MTransformationMatrix(jExclMat) # world transformation matrix of parent joint
	jExclTransMat.setScale([gridSize,gridSize,gridSize],4) # set scale in world space (om.MSpace.kWorld = 4)
	jExclTransNPMat = np.array(jExclTransMat.asMatrix()).reshape(4,4) # convert into numpy 4x4 array

	# define initial guess condition

	initial_guess = np.zeros(3)

	# create 4x4 identity matrix for coordinate matrix multiplication

	paramCoords = np.eye(4)

	# set time to 1

	cmds.currentTime(1)


	# ==== optimise translations ====


	# go through all possible combinations

	updateSwitch = True

	print('{0} Translation optimisation progress: {1:.1f}%.'.format(fileName,0)) 

	for frame in range(frames):

		# extract rotation

		rotation = [float(rotations[frame][0]),float(rotations[frame][1]),float(rotations[frame][2])]

		# get joint inclusive transformation matrix (child)

		localTransMat = MTransformationMatrix()
		localTransMat.setRotation(om.MEulerRotation(np.deg2rad(rotation), order = 0)) # set rotation (om.MEulerRotation.kXYZ = 0)
		localTransMat.setTranslation(MVector([0,0,0]),2) # reset translation (om.MSpace.kObject = 2)

		jInclTransMat = localTransMat.asMatrix() * gridSize * jExclMat
		jInclTransMat[-1] = 1 # reset last element to 1

		# get rotation matrices

		rotMat = []
		rotMat.append(jExclTransNPMat) # append parent rotMat (prox) as numpy 4x4 array
		rotMat.append(np.array(jInclTransMat).reshape(4,4)) # append child rotMat (dist) as numpy 4x4 array
		rotMat.append(jExclNPMat) # append parent rotMat (prox) without scale as numpy 4x4 array
		rotMat.append(np.linalg.inv(jExclTransNPMat)) # append inverse of parent rotMat (prox) as numpy 4x4 array

		# optimise the joint translations

		coords, viable = optimisePosition(proxArr, distArr, ipProx, ipDist, gridRotMat, rotMat, thickness, initial_guess, paramCoords)

		# check if pose was viable

		if viable:
			transRes.append(coords.tolist() + rotation)

		# update progress
		
		percent = ((1000 * (frame + 1)) // frames) / 10

		if updateSwitch:
			previous = percent
			updateSwitch = False

		if percent > previous:
			ETA = time.strftime("%H h %M min %S sec", time.gmtime((100 - percent) * (time.time() - mid) / percent))
			print('{0} Translation optimisation progress: {1:.1f}%. Estimated completion in: {2}'.format(fileName,percent,ETA)) 
			updateSwitch = True

		# go to next frame

		cmds.currentTime(frame + 2)


	# ==== save results and print to file ====


	# define outpule file name and path

	namei = fileName.replace('.mb','')
	fname = outDir + '/' + namei + '.csv'

	# write to output file

	with open(fname, 'w') as filehandle:
		for listitem in transRes:
			s = ",".join(map(str, listitem))
			filehandle.write('%s\n' % s)

	# shut down the Maya scene

	cmds.file(modified = 0) 

	# print simulation time for each Maya scene

	end = time.time()
	convert = time.strftime("%H hours %M min %S seconds", time.gmtime(end - mid))
	print('Translation optimisation for {0} done in {1}! Successfully tested {2} frames and exported {3} viable joint transformations'.format(fileName,convert,frames,len(transRes)))





