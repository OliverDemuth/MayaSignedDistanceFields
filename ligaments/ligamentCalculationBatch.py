#	ligamentCalculationBatch.py
#
#	This script calculates the shortest distance of a ligament from origin to insertion 
#	wrapping around the bone meshes. It is an implementation of the Marai et al., 2004.
#	approach for Autodesk Maya. It creates signed distance fields for the proximal and
#	distal bone meshes, which are then used to approximate the 3D path of each ligament
#	accross them to calculate their lengths.
#
#	Written by Oliver Demuth and Vittorio la Barbera
#	Last updated 29.05.2026 - Oliver Demuth
#
#	SYNOPSIS:
#
#		INPUT params:
#			string  jointName:		Name of the joint centre, i.e. the name of a locator or joint (e.g., 'myJoint' if following the ROM mapping protocol of Manafzadeh & Padian 2018)
#			string  meshes:			Name(s) of the bone meshes (e.g., several individual meshes in the form of ['prox_mesh','dist_mesh'])
#			int		gridSubdiv:		Integer value for the subdivision of the cube, i.e., number of grid points per axis (e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points)
#			float	gridScale:		Float value for the scale factor of the cubic grid (i.e., 1.5 initialises the grid from -1.5 to 1.5)
#			int 	ligSubdiv:		Integer value for the number of ligament points (e.g., 20 will divide the ligament into 20 equidistant segments, see Marai et al., 2004 for details)
#			int 	frameInterval:	Integer value to specify number of frames to be tested. If all frames are to be tested set to standard value: None
#			int 	maxIter:		Integer value specifying the maximum number of iterations for the SLSQP optimiser
#			string 	outDir:			Output directory
#
#		RETURN params:
#			list	pathLengths:	Return value is a list with the path lengths for all ligaments designated as custom attributes in the 'jointName'
#			list 	pathPoints:		Return value is a list of lists with the 3D coordinates of the path points in world space for all ligaments designated as custom attributes in the 'jointName'
#			list 	results: 		Return value is a list of objects of the scipy.optimize.OptimizeResult class. They represent the outputs of the scipy.optimize.minimize() function
#
#
#	IMPORTANT notes:
#		
#	(1) Meshes need realtively uniform face areas, otherwise large faces might skew 
#		vertex normals in their direction. It is, therefore, important to extrude 
#		edges around large faces prior to closing the hole at edges with otherwise 
#		acute angles to circumvent this issue (e.g., if meshes have been cut to reduce
#		polycount) prior to executing the Python scripts.
#
#	(2) For each ligament create a float attribute at 'jointName' and name it 
#		accordingly. Make sure that the naming convention for the ligament origins 
#		and insertions is correct, i.e. the locators should be named 'ligament*_orig' 
#		and 'ligament*_ins' for an attribute in 'jointName' called 'ligament*'.
#
#	(3)	This script requires several modules for Python, see README file. Make sure to
#		have the following external modules installed for the mayapy application:
#
#			- 'numpy' 	NumPy:			https://numpy.org/about/
#			- 'scipy'	SciPy:			https://scipy.org/about/
#				
#		For further information regarding them, please check the website(s) referenced 
#		above.


# ========== load modules ==========

import functools
import time
import os

from math import floor, ceil
from datetime import timedelta
from ligamentCalculation import * # source the ligament functions


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


		# ==== frist set environment to single core ==== 


		# IMPORTANT:
		# 	Force single core execution of underlaying C libraries to prevent severe 
		#	thread over-subscription and hardware starvation! Otherwise they parallelise
		#	across all available CPU cores for each mayapy instance (requiring N times
		#	more CPU cores than are actually available, regardless of how many cores were
		#	assigned in the ligamentCalculationWrapper.py) and thus leading to CPU 
		#	thrashing.

		import os

		os.environ["OMP_NUM_THREADS"] = "1"
		os.environ["MKL_NUM_THREADS"] = "1"
		os.environ["OPENBLAS_NUM_THREADS"] = "1"
		os.environ["VECLIB_MAXIMUM_THREADS"] = "1"
		os.environ["NUMEXPR_NUM_THREADS"] = "1"


		# ==== now import modules into single core enivornment ==== 


		import maya.standalone
		import maya.cmds as cmds
		import maya.api.OpenMaya as om
		import maya.api.OpenMayaAnim as oma
		import numpy as np
		import scipy as sp


		# ==== initialise Maya ====


		maya.standalone.initialize(name = 'python')

		# force single core useage in Maya as well

		cmds.threadCount(n = 1)

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

	cmds.scriptEditorInfo(sw = True ,se = True)

	# get file name 

	fileName = os.path.basename(filePath)

	print('Processing file: ', fileName)

	# open Maya scene and initialise calculations 

	cmds.file(filePath, open = True, force = True)

	# extract arguments

	[jointName, meshes, gridSubdiv, gridScale, ligSubdiv, frameInterval, maxIter, outDir] = args


	# ==== calculate signed distance fields ====


	start = time.time()

	print('Calculating signed distance fields for {}...'.format(fileName))
	
	# get dag path for joint

	jNode, jDag = dagObjFromName(jointName)

	# reset joint

	eyeMat = om.MTransformationMatrix(om.MMatrix(np.eye(4)))
	om.MFnTransform(jDag).setTransformation(eyeMat)

	# calculate signed distance fields
	
	SDFs, ligAttributes, oDags, iDags, maxDist = sigDistField(jointName, meshes, gridSubdiv, gridScale)

	part1 = time.time()

	print('Signed distance fields calculated in {0:.3f} seconds for {1}!'.format(part1 - start,fileName))


	# ==== extract joint transformations from keyframes ====


	# get reference transformation matrices

	jExclInv = jDag.exclusiveMatrix().inverse()
	jInclInv = jDag.inclusiveMatrix().inverse()

	# get local offsets of origins and insertions

	oRelPos = np.array([np.array(orig.inclusiveMatrix() * jExclInv) for orig in oDags]).reshape(len(oDags),4,4)[:,3,:] # N x 4 array
	iRelPos = np.array([np.array(ins.inclusiveMatrix()  * jInclInv) for ins  in iDags]).reshape(len(oDags),4,4)[:,3,:] # N x 4 array

	# get total number of keyed frames from 'jointName', i.e., max number of frames to be calculated

	attributes = ["translateX","translateY","translateZ","rotateX","rotateY","rotateZ"]
	maxFrames = 0

	for attr in attributes:
		attr_node = om.MSelectionList().add(f"{jointName}_{attr}").getDependNode(0)
		attr_curve = oma.MFnAnimCurve(attr_node)
		maxFrames = max(maxFrames,attr_curve.numKeys)

	if frameInterval and frameInterval < maxFrames:
		frames = frameInterval
	else:
		frames = maxFrames

	inclMat = np.empty((frames, 16), dtype = np.float64)
	exclMat = np.empty((frames, 16), dtype = np.float64)

	# get matrix plugs

	j_incl_plug = om.MFnDependencyNode(jNode).findPlug("worldMatrix", False).elementByLogicalIndex(0)
	j_excl_plug = om.MFnDependencyNode(jNode).findPlug("parentMatrix", False).elementByLogicalIndex(0)

	# cycle through frames and calculate matrix transformations

	for frame in range(frames):

		# get frame

		context = om.MDGContext(om.MTime(frame + 1, 6)) # om.MTime.uiUnit() = 6

		# get joint transformation matrices

		inclMat[frame,:] = om.MFnMatrixData(j_incl_plug.asMObject(context)).matrix() # extract matrix without updating viewport
		
		# check if parent has incoming connections, otherwise use static

		jExclPlug = j_excl_plug.asMObject(context)

		# get parent transformation matrices

		if jExclPlug.isNull(): # check whether parent is stationary or dynamic (has incoming connections)
			exclMat[frame,:] = om.MFnMatrixData(j_excl_plug.asMObject()).matrix()
		else:
			exclMat[frame,:] = om.MFnMatrixData(jExclPlug).matrix()

	# reshape arrays into Nx4x4 transformation matrix arrays

	transMat = np.empty((frames,2,4,4))
	transMat[:,0,:,:] = exclMat.reshape(frames,4,4)
	transMat[:,1,:,:] = inclMat.reshape(frames,4,4)

	# calculate world coordinates of joint and ligament attachments

	jPos = transMat[:,1,3,0:3]
	oPos = oRelPos @ transMat[:,0,:,:]
	iPos = iRelPos @ transMat[:,1,:,:]

	# get inverse of both parent and child rotation matrices

	invTransMat = np.linalg.inv(transMat)

	part2 = time.time()

	print('# Calculated {0} joint transformations in {1:.3f} seconds.'.format(frames, part2 - part1))


	# ==== calculate ligament lengths ====


	numPoints = ligSubdiv + 1

	# define constant x coords

	ligArr = np.stack([np.array([0.0,0.0,0.0,1.0])] * numPoints, axis = 0)
	ligArr[:,0] = np.linspace(0.0, 1.0, num = numPoints, endpoint = True) # constant X coordinates

	# maximal offset for path constraint

	maxOffset = 3 / (ligSubdiv ** 2) # max squared mediolateral offset (i.e., arctan(offset/dist) ≤ 60° as tan(60°) = sqrt(3))

	# define initual guess condition for optimiser

	initial_guess = np.zeros(2 * numPoints)

	# set bounds

	bounds = [(-maxDist, maxDist) for _ in range(2 * numPoints)]
	bounds[0] = bounds[1] = bounds[-2] = bounds[-1] = (0,0)
	bounds = tuple(bounds)

	# go through each frame and calculate ligament lengths 

	updateSwitch = True

	# initialise results array

	ligRes = np.empty((frames,len(ligAttributes)))

	print('{0} ligament calculation progress: {1:.0f}%.'.format(fileName,0)) 

	for frame in range(frames):

		# calculate 4x4 transformation matrices for all ligaments

		ligRotMats, offsets = getLigTransMat(oPos[frame,:], iPos[frame,:], jPos[frame,:])

		# calculate the length of each ligament 

		ligRes[frame,:], *_ = ligCalc(initial_guess, ligArr, SDFs[0], SDFs[1], invTransMat[frame,:,:,:], ligRotMats, offsets, False, maxOffset, numPoints, bounds, maxIter)

		# update progress
		
		percent = ((1000 * (frame + 1)) // frames) / 10

		if updateSwitch:
			previous = percent
			updateSwitch = False

		if percent > previous:
			ETA = '{0} hours {1} min {2} seconds'.format(*str(timedelta(seconds=ceil((100 - percent) * (time.time() - part2) / percent))).split(':'))
			print('{0} Ligament calculation progress: {1:.1f}%. Estimated completion in: {2}'.format(fileName,percent,ETA)) 
			updateSwitch = True


	# ==== save results and print to file ====


	# collect output

	exportData = ligRes.tolist()
	exportData.insert(0,ligAttributes)

	# define outpule file name and path

	namei = fileName.replace('.mb','.csv')
	fname = os.path.join(outDir,namei)

	# write to output file

	with open(fname, 'w') as filehandle:
		for listitem in exportData:
			s = ",".join(map(str, listitem))
			filehandle.write('%s\n' % s)

	# shut down the Maya scene

	cmds.file(modified = 0) 

	# print simulation time for each Maya scene

	end = time.time()
	convert = '{0} hours {1} min {2} seconds'.format(*str(timedelta(seconds = ceil(end - part2))).split(':'))
	print('Ligament calculation for {0} done in {1}!'.format(fileName,convert))
	
