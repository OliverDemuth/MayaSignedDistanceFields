#	ROMmapperBatch.py
#
#	This script optimises contact-based positions across a set of rotational poses for
#	a set of bone meshes. It is an implementation of the Marai et al., 2006 and Lee
#	et al. 2023 approach for Autodesk Maya. It creates signed distance fields for the
#	proximal and distal bone meshes which are then used to caculate intersections
#	between the meshes and the distance between them. 
#
#	Written by Oliver Demuth 
#	Last updated 02.12.2025 - Oliver Demuth
#
#	SYNOPSIS:
#
#		INPUT params:
#			string	jointName:			Name of the joint centre (i.e. the name of a locator or joint; e.g., 'myJoint' if following the ROM mapping protocol of Manafzadeh & Padian 2018)
#			string	meshes:				Names of the two bone meshes and optional convex hull (i.e., several individual meshes representing the bones and rib cage; e.g., in the form of ['prox_mesh','dist_mesh', 'conv_hull'])
#			string	congruencyMeshes:	Names of the meshes to check articular congruency (i.e., several individual meshes; e.g., in the form of ['prox_art_surf','dist_art_surf'])
#			string	fittedShape:		Name of the shape fitted to the proximal articular surface (e.g., in the form 'prox_fitted_sphere') used for proximity estimation
#			float	xBounds:			Float array of bounds for X-axis rotation in the form of [min, max] (i.e., LAR, e.g., [-180,180] for spherical joints or [-90,90] for hinge joints)
#			float	yBounds:			Float array of bounds for Y-axis rotation in the form of [min, max] (i.e., ABAD, e.g.,  [-90,90] for spherical joints or [-90,90] for hinge joints)
#			float	zBounds:			Float array of bounds for Z-axis rotation in the form of [min, max] (i.e., FE, e.g.,  [-180,180] for spherical joints or [0,180] for hinge joints)
#			int		interval:			Integer value for the sampling interval between bounds (e.g., for FE and LAR -180:interval:180, and for ABAD -90:interval:90; see Manafzadeh & Padian, 2018)
#			int		gridSubdiv:			Integer value for the subdivision of the cube (i.e., number of grid points per axis; e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points)
#			float	gridSize:			Float value indicating the size of the cubic grid (i.e., the length of a side; e.g., 10 will result ing a cubic grid with the dimensions 10 x 10 x 10)
#			float	gridScale:			Float value for the scale factor of the cubic grid (i.e., 1.5 initialises the grid from -1.5 to 1.5)
#			float	thickness:			Float value indicating the thickness value which correlates with the joint spacing
#			float	weights:			Float array of weights for the cost function in the form of [w1, w2, w3, w4, w5]
#			float	tolerance: 			Float value for the joint proximity tolerance
#			float	scaleFactor:		Float value for the joint offset to determine joint disarticulation
#			float	cutOff:				Float value for the final SDF interpolation (default is 0.0)
#
#		RETURN params:
#			array 	coords:				Return value is a an array of 3D coordinates of the distal bone relative to the proximal one
#			boolean	viable:				Return value is a boolean whether the rotational poses are viable or inviable (i.e, intersection or disarticulation of the bone meshes)
#
#
#	IMPORTANT notes:
#
#	(1) Meshes need realtively uniform face areas, otherwise large faces might skew 
#	    vertex normals in their direction. It is, therefore, important to extrude 
#	    edges around large faces prior to closing the hole at edges with otherwise 
#	    acute angles to circumvent this issue (e.g., if meshes have been cut to reduce
#	    polycount, prior to executing the Python scripts).
#
#	(2) For each bone two sets of meshes are required, the articular surface(s) and
#	    the bone mesh
#
#	(3) This script requires several modules for Python, see README file. Make sure to
#	    have the following external modules installed for the mayapy application:
#
#		- 'numpy' 	NumPy:		https://numpy.org/about/
#		- 'scipy'	SciPy:		https://scipy.org/about/
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

from math import ceil
from datetime import timedelta
from ROMmapper import * # source the ROM functions


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

		maya.standalone.initialize(name = 'python')

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

	cmds.scriptEditorInfo(sw = True, se = True)

	# get file name 

	fileName = os.path.basename(filePath)
	print('Processing file: ', fileName)

	# open Maya scene and initialise calculations 

	cmds.file(filePath, open = True, force = True)

	# extract arguments	

	[jointName, meshes, congruencyMeshes, fittedShape, gridSubdiv, gridScale, simBounds, interval, weights, tolerance, scaleFactor, cutOff, thickness, outDir] = args


	# ==== calculate signed distance fields ====


	start = time.time()

	print('Calculating signed distance fields for {}...'.format(fileName))

	# reset joint

	cmds.currentTime(0)
	cmds.move(0,0,0, jointName, localSpace=True)
	cmds.rotate(0,0,0,jointName)

	# get gridsize from glenoid sphere radius
	meanRad, dims = meanRadMaya(fittedShape)

	if thickness is None:
		thickness = meanRad / 2

	gridSize = 16 * thickness

	# get dag path for joint

	jDag = dagObjFromName(jointName)[1]

	SDF, initialRotMat = sigDistField(jDag, meshes, gridSubdiv, gridSize, gridScale)

	mid = time.time()

	print('Signed distance fields calculated in {0:.3f} seconds for {1}!'.format(mid - start,fileName))


	# ==== initialise variables and precalculations ====


	# calculate relative position of articular surfaces

	proxArr = relVtcPos(congruencyMeshes[0], initialRotMat[0])
	distArr = relVtcPos(congruencyMeshes[1], initialRotMat[1])
	distMeshArr = relVtcPos(meshes[1], initialRotMat[1])

	# create 3D grid for rotations

	xRots = np.arange(xBounds[0], ceil(xBounds[1] + interval / 2), interval, dtype = float)
	yRots = np.arange(yBounds[0], ceil(yBounds[1] + interval / 2), interval, dtype = float)
	zRots = np.arange(zBounds[0], ceil(zBounds[1] + interval / 2), interval, dtype = float)

	rotX, rotY, rotZ = np.meshgrid(xRots, yRots, zRots, indexing='ij')
	rotations = np.vstack((rotX.ravel(), rotY.ravel(), rotZ.ravel())).T 

	frames = len(rotations)

	# initialise results array

	transRes =[] 

	# get joint exclusive transformation matrix (parent)

	jExclNPMat = np.array(jDag.exclusiveMatrix()).reshape(4,4) # world rotation matrix of parent joint as numpy 4x4 array
	jExclNPMatInv = np.linalg.solve(jExclNPMat, np.eye(4)) # inverse of parent rotMat (prox) as numpy 4x4 array

	# define initial guess condition

	if shapeCheck: # sphere

		initial_guess = np.zeros(3)

	else: # cylinder or ellipsoid
			
		initial_guess = (np.array((1.1 * meanRad, 0.0, 0.0, 1.0)) @ transMat)[0:3] # set initial guess as 1.1 times the radius in X-axis direction (joint distraction)

		# clip initial guess to cylinder bounds

		bnds = np.array(bounds)
		initial_guess = np.clip(initial_guess, bnds[:,0], bnds[:,1])


	# ==== optimise translations ====


	# go through all possible combinations

	updateSwitch = True

	print('{0} Translation optimisation progress: {1:.1f}%.'.format(fileName,0)) 

	for frame in range(frames):

		# extract rotation

		rotation = rotations[j,:]

		# get joint inclusive transformation matrix (child)

		transMat = np.eye(4)
		transMat[0:3,0:3] = sp.spatial.transform.Rotation.from_euler('ZYX', rotation, degrees = True).as_matrix()[::-1,::-1] # inverse matrix directions to be consistent with previous approach (i.e., converting SciPy’s (x,y,z) basis into Maya’s (z,y,x) basis)

		# get rotation matrices

		rotMat = []
		rotMat.append(jExclNPMat) # append parent rotMat (prox) as numpy 4x4 array
		rotMat.append(transMat @ jExclNPMat) # append child rotMat (dist) as numpy 4x4 array
		rotMat.append(jExclNPMatInv) # append inverse of parent rotMat (prox) as numpy 4x4 array

		# optimise the joint translations

		coords, viable = optimisePosition(proxArr, 		 # 2D array of proximal articular surface vertex coordinates for fast computation of relative coordinates 
										  distArr, 		 # 2D array of distal articular surface vertex coordinates for fast computation of relative coordinates 
										  distMeshArr,   # 2D array of distal mesh vertex coordinates for fast computation of relative coordinates
										  SDF, 			 # list containing multiple signed distance fields in tricubic form (e.g., [ipProx, ipDist])
										  rotMat, 		 # array with the transformation matrices of the joint and its parent
										  thickness, 	 # thickness measure correlated with joint spacing
										  weights, 		 # weights for the individual cost function terms
										  initial_guess, # initual guess condition for optimiser 
										  bounds, 		 # bounds for optimisation
										  scaleFactor, 	 # scale factor to roughly check if joint is disarticulated
										  maxIter, 		 # maximum number of iterations
										  tol, 			 # tolerance for joint proximity
										  cutOff)		 # cutoff value for signed distance fields

		# check if pose was viable

		if viable:
			transRes.append(coords.tolist() + rotation.tolist()) # combine both lists and append to results array [Tx, Ty, Tz, Rx, Ry, Rz]

		# update progress

		percent = ((1000 * (frame + 1)) // frames) / 10

		if updateSwitch:
			previous = percent
			updateSwitch = False

		if percent > previous:
			ETA = '{0} hours {1} min {2} seconds'.format(*str(timedelta(seconds = ceil((100 - percent) * (time.time() - mid) / percent))).split(':'))
			print('{0} Translation optimisation progress: {1:.1f}%. Estimated completion in: {2}'.format(fileName,percent,ETA)) 
			updateSwitch = True


	# ==== save results and print to file ====


	# define outpule file name and path

	namei = fileName.replace('.mb','.csv')
	fname = os.path.join(outDir,namei)

	# write to output file

	with open(fname, 'w') as filehandle:
		for listitem in transRes:
			s = ",".join(map(str, listitem))
			filehandle.write('%s\n' % s)

	# shut down the Maya scene

	cmds.file(modified = 0) 

	# print simulation time for each Maya scene

	end = time.time()
	convert = '{0} hours {1} min {2} seconds'.format(*str(timedelta(seconds=ceil(end - mid))).split(':'))
	print('Translation optimisation for {0} done in {1}! Successfully tested {2} frames and exported {3} viable joint transformations'.format(fileName,convert,frames,len(transRes)))
	print('Wrote joint transformations to {0} file at {1}.'.format(namei, outDir))
	
