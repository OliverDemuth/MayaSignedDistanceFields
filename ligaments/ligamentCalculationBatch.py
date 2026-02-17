#	ligamentCalculationBatch.py
#
#	This script calculates the shortest distance of a ligament from origin to insertion 
#	wrapping around the bone meshes. It is an implementation of the Marai et al., 2004.
#	approach for Autodesk Maya. It creates signed distance fields for the proximal and
#	distal bone meshes, which are then used to approximate the 3D path of each ligament
#	accross them to calculate their lengths.
#
#	Written by Oliver Demuth and Vittorio la Barbera
#	Last updated 17.02.2026 - Oliver Demuth
#
#	SYNOPSIS:
#
#		INPUT params:
#			string  jointName:		Name of the joint centre, i.e. the name of a locator or joint (e.g., 'myJoint' if following the ROM mapping protocol of Manafzadeh & Padian 2018)
#			string  meshes:			Name(s) of the bone meshes (e.g., several individual meshes in the form of ['prox_mesh','dist_mesh'])
#			int	gridSubdiv:			Integer value for the subdivision of the cube, i.e., number of grid points per axis (e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points)
#			float	gridScale:		Float value for the scale factor of the cubic grid (i.e., 1.5 initialises the grid from -1.5 to 1.5)
#			int 	ligSubdiv:		Integer value for the number of ligament points (e.g., 20 will divide the ligament into 20 equidistant segments, see Marai et al., 2004 for details)
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

import maya.standalone
import maya.api.OpenMaya as om
import maya.cmds as cmds
import numpy as np
import scipy as sp
import functools
import os
import time

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

	cmds.scriptEditorInfo(sw = True ,se = True)

	# get file name 

	fileName = os.path.basename(filePath)

	print('Processing file: ', fileName)

	# open Maya scene and initialise calculations 

	cmds.file(filePath, open = True, force = True)

	# extract arguments

	[jointName, meshes, gridSubdiv, gridScale, ligSubdiv, FrameInterval, outDir] = args


	# ==== calculate signed distance fields ====


	start = time.time()

	print('Calculating signed distance fields for {}...'.format(fileName))
	
	cmds.currentTime(0)
	cmds.move(0,0,0, jointName, localSpace = True)
	cmds.rotate(0,0,0,jointName)
	
	SDFs, LigAttributes, oDags, iDags, jDag, maxDist = sigDistField(jointName, meshes, gridSubdiv, gridScale)
	
	# get inverse of rotation matrix for default cubic grid coordinate system

	mid = time.time()

	print('Signed distance fields calculated in {0:.3f} seconds for {1}!'.format(mid - start,fileName))


	# ==== calculate ligament lengths ====


	# set to starting frame 

	cmds.currentTime(1)

	# get total number of keyed frames from 'jointName', i.e., max number of frames to be calculated

	if FrameInterval:
		frames = FrameInterval
	else:
		frames = cmds.keyframe(jointName, attribute = 'rotateX', query = True, keyframeCount = True)

	numPoints = ligSubdiv + 1

	# initialise results array

	ligRes =[] 
	ligRes.append(LigAttributes)

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

	print('{0} ligament calculation progress: {1:.0f}%.'.format(fileName,0)) 

	for frame in range(frames):

		# get joint centre position

		jInclMat = jDag.inclusiveMatrix()
		jPos = np.array(om.MTransformationMatrix(jInclMat).translation(4)) # om.MSpace.kWorld = 4

		# get rotation matrices

		rotMat = []
		rotMat.append(np.array(jDag.exclusiveMatrix().inverse()).reshape(4,4)) # inverse of parent rotMat (prox)
		rotMat.append(np.array(jInclMat.inverse()).reshape(4,4)) # inverse of child rotMat (dist)

		# get world coordinates of ligament origins and insertions

		oPos = np.array([om.MTransformationMatrix(orig.inclusiveMatrix()).translation(4) for orig in oDags]) # get world coordinates from dagPaths; om.MSpace.kWorld = 4
		iPos = np.array([om.MTransformationMatrix(ins.inclusiveMatrix()).translation(4) for ins in iDags]) # get world coordinates from dagPaths; om.MSpace.kWorld = 4

		# calculate 4x4 transformation matrices for all ligaments

		ligRotMats, offsets = getLigTransMat(oPos, iPos, jPos)

		# calculate the length of each ligament 

		pathLengths, *_ = ligCalc(initial_guess, ligArr, SDFs[0], SDFs[1], rotMat, ligRotMats, offsets, False, maxOffset, numPoints, bounds)

		ligRes.append(pathLengths) # add row of ligament lengths to result array

		# update progress
		
		percent = ((1000 * (frame + 1)) // frames) / 10

		if updateSwitch:
			previous = percent
			updateSwitch = False

		if percent > previous:
			ETA = '{0} hours {1} min {2} seconds'.format(*str(timedelta(seconds=ceil((100 - percent) * (time.time() - mid) / percent))).split(':'))
			print('{0} Ligament calculation progress: {1:.1f}%. Estimated completion in: {2}'.format(fileName,percent,ETA)) 
			updateSwitch = True

		# go to next frame

		cmds.currentTime(frame + 2) # frames start from 1 and not from 0 (current frame is frame + 1)


	# ==== save results and print to file ====


	# define outpule file name and path

	namei = fileName.replace('.mb','.csv')
	fname = os.path.join(outDir,namei)

	# write to output file

	with open(fname, 'w') as filehandle:
		for listitem in ligRes:
			s = ",".join(map(str, listitem))
			filehandle.write('%s\n' % s)

	# shut down the Maya scene

	cmds.file(modified = 0) 

	# print simulation time for each Maya scene

	end = time.time()
	convert = '{0} hours {1} min {2} seconds'.format(*str(timedelta(seconds = ceil(end - mid))).split(':'))
	print('Ligament calculation for {0} done in {1}!'.format(fileName,convert))
	
