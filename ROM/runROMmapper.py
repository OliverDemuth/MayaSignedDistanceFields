#	runROMmapper.py
#
#	This script estimates the (mobile) joint centre position and moves the distal bone
#	mesh into position for a set of presribed joint orientations. Only feasible joint 
#	positions will be keyed. The resulting set of keyed frames represent vialbe joint 
#	positions and orientations. It is an implementation of the Marai et al., 2006 and 
#	Lee et al., 2023 approach for Autodesk Maya.
#
#	Written by Oliver Demuth
#	Last updated 16.02.2026 - Oliver Demuth
#
#
#	Rename the strings in the user defined variables below according to the objects in
#	your Maya scene and define the other attributes accordingly.
#
#
#	This script relies on the following other (Python) script(s) which need to be run
#	in the Maya script editor before executing this script:
#
#		- 'ROMmapper.py'
#
#	For further information please check the Python script(s) referenced above


#################################################
# ========== user defined variables  ========== #
#################################################


jointName = 'myJoint' 					# specify according to the joint centre in the Maya scene, i.e. the name of a locator or joint (e.g. 'myJoint' if following the ROM mapping protocol of Manafzadeh & Padian 2018)
meshes = ['prox_mesh', 					# specify according to meshes in the Maya scene
		  'dist_mesh']
congruencyMeshes = ['prox_art_surf', 	# specify according to meshes in the Maya scene
					'dist_art_surf']
fittedShape = 'prox_sphere'				# specify according to meshes in the Maya scene		
xBounds = [-180,180]					# bounds for X-axis rotation in the form of [min, max] (i.e., LAR, e.g., [-180,180] for spherical joints or [-90,90] for hinge joints)
yBounds = [-90,90]						# bounds for Y-axis rotation in the form of [min, max] (i.e., ABAD, e.g.,  [-90,90] for spherical joints or [-90,90] for hinge joints)
zBounds = [-180,180]					# bounds for Z-axis rotation in the form of [min, max] (i.e., FE, e.g.,  [-180,180] for spherical joints or [0,180] for hinge joints)
gridSubdiv = 100						# integer value for the subdivision of the cube, i.e., number of grid points per axis (e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points)
gridScale = 1							# float value for the scale factor of the cubic grid (i.e., 1.5 initialises the grid from -1.5 to 1.5)
interval = 5							# sampling interval, see Manafzadeh & Padian, 2018, (e.g., for FE and LAR -180:interval:180, and for ABAD -90:interval:90)
weights = [1.0,							# weight for first cost term (proximal joint spacing; prox_art_surf in ipDist)
		   0.0,							# weight for second cost term (distal joint spacing; dist_art_surf in ipProx)
		   1.0,							# weight for third cost term (proximal joint congruency)
		   0.0,							# weight for fourth cost term (distal joint congruency)
		   0.0]							# weight for fifth cost term (joint offset)
tolerance = 0.07						# tolerance for joint proximity (i.e, set target thickness tolerance; e.g., 0.07 based on experimental data)
scaleFactor = 2.2 						# scale factor to roughly check if joint is disarticulated (i.e, if distal ACS is more than 10% beyond radius of fitted proximal shape; default value is 2.2: thickness = 0.5 * radius)
cutOff = 0 								# cut off value for final SDF interpolation (default is 0, but sometimes differences in mesh resolution between articular surfaces and mesh might result in slightly negative values. In that case -0.005 might be a better choice)
thickness = None						# Float value indicating the thickness value which correlates with the joint spacing. If set to None it will automatically be determined based on the fitted shape radius.
StartFrame = None 						# Integer value to specify the start frame. If all frames are to be keyed from the beginning (Frame 1) set to standard value: None or 1.
FrameInterval = None					# Integer value to specify number of frames to be keyed. If all frames are to be keyed set to standard value: None
ContinueKeys = False					# Boolean value (True or False) to specify whether a previous simulation should be continued (under the assumption that the interval has not changed)
debug = 0 								# Debug mode to check if signed distance fields have already been calculated


#################################################
# ==========    main script below    ========== #
#################################################


# ========== load modules ==========

import maya.api.OpenMaya as om
import maya.cmds as cmds
import numpy as np
import scipy as sp
import time

from math import ceil

# ========================================

# reset joint

cmds.rotate(0, 0, 0, jointName)
cmds.move(0, 0, 0, jointName)

# get dag path for joint

jDag = dagObjFromName(jointName)[1]

# get gridsize from glenoid sphere radius

meanR, dims = meanRad(fittedShape)

if thickness is None:
	thickness = meanR / 2

gridSize = 16 * thickness

# initialise signed distance fields

start = time.time()

if debug == 1 or ContinueKeys:

	# check if distance fields have already been calculated

	try:
		SDF
	except NameError:
		var_exists = False
	else:
		var_exists = True # signed distance field already calculated, no need to do it again

else:
	var_exists = False

if not var_exists:

	# calculate signed distance fields

	print('Calculating signed distance fields...')

	SDF, initialRotMat = sigDistField(jDag, meshes, gridSubdiv, gridSize, gridScale)

	# calculate relative position of articular surfaces

	proxArr = relVtcPos(congruencyMeshes[0], initialRotMat[0])
	distArr = relVtcPos(congruencyMeshes[1], initialRotMat[1])
	distMeshArr = relVtcPos(meshes[1], initialRotMat[1])

	mid = time.time()
	print('Signed distance fields calculated in {0:.3f} seconds!'.format(mid - start))

else:
	mid = time.time()
	print('Signed distance fields succesfully loaded in {0:.3f} seconds!'.format(mid - start))


# ==== initialise variables and precalculations ====


# get dimensions of target object

dimComp = np.isclose(dims[:,None], dims[None,:]).sum(axis = 1)

# determine fitted shape

shapeCheck = dimComp.max() == 3 # if true is sphere (i.e., X, Y, Z axis dimensions are identical), otherwise is cylinder 

# set bounds

if shapeCheck: # sphere

	bound = np.clip(dims, 0, gridSize)[0] # clamp dims to gridSize
	bounds = tuple([(-bound, bound)] * 3) # bounds for translations (X,Y,Z)

else: # cylinder or elipsoid

	bound = np.clip(dims * 0.75, 0, gridSize) # +/- 1.5 times the radius for each axis (X,Y,Z)
	bounds = tuple((-b, b) for b in bound)

# create 3D grid for rotations

xRots = np.arange(xBounds[0], ceil(xBounds[1] + interval / 2), interval, dtype = float)
yRots = np.arange(yBounds[0], ceil(yBounds[1] + interval / 2), interval, dtype = float)
zRots = np.arange(zBounds[0], ceil(zBounds[1] + interval / 2), interval, dtype = float)

rotX, rotY, rotZ = np.meshgrid(xRots, yRots, zRots, indexing='ij')
rotations = np.vstack((rotX.ravel(), rotY.ravel(), rotZ.ravel())).T 

numFrames = len(rotations)

# set time to 1 or to start frame

if ContinueKeys:
	lastKey = cmds.keyframe(jointName, attribute = 'rotateX', query = True, index = (1, cmds.keyframe(jointName, attribute = 'rotateX', query = True, keyframeCount = True)))[-1]

	xRot = cmds.keyframe(jointName, attribute = 'rotateX', query = True, eval = True, time = (lastKey,lastKey))[0]
	yRot = cmds.keyframe(jointName, attribute = 'rotateY', query = True, eval = True, time = (lastKey,lastKey))[0]
	zRot = cmds.keyframe(jointName, attribute = 'rotateZ', query = True, eval = True, time = (lastKey,lastKey))[0]

	matches = np.all(np.isclose(rotations, np.array([xRot,yRot,zRot])), axis=1)

	idxs = np.flatnonzero(matches)

	if idxs.size != 0: 
		rotIdx = idxs[0] # get index of last keyed frame
		minKeys = rotIdx + 2 # get index of next frame to be keyed
	else: # key not part of rotations
		minKeys = 1
		cmds.cutKey(jointName, option = 'keys') # delete all previous keyframes
		lastKey = 0

	# set current time

	cmds.currentTime(lastKey + 1)

else:
	if StartFrame is None:
		minKeys = 1 # get index of next frame to be keyed
		cmds.cutKey(jointName, option = 'keys') # delete all previous keyframes
	else: 
		minKeys = StartFrame

	lastKey = 0	

	# set current time

	cmds.currentTime(minKeys)

# get total number of frames to be keyed

if FrameInterval is None or minKeys + FrameInterval > numFrames:
	keyframes = numFrames
	keyDiff = keyframes - minKeys + 1
else:
	keyframes = minKeys + FrameInterval
	keyDiff = FrameInterval

keyDiff = max(1, keyDiff)

if debug == 1 and FrameInterval is not None: # randomly assign rotations for testing
	rng = np.random.default_rng(seed = 42)
	rotations = rotations[rng.integers(0,rotations.shape[0],FrameInterval)]

# define progress bar

cmds.progressWindow(title = 'Translation optimisation in progress...',
				    progress = 1,
		    		status = 'Processing frame {0} of {1} frames'.format(1,keyDiff),
		    		isInterruptable = True,
		    		max = keyDiff)

print('Translation optimisation in progress...')

# get joint exclusive transformation matrix (parent)

jDag = dagObjFromName(jointName)[1]

# get joint exclusive transformation matrix (parent)

jExclmat = jDag.exclusiveMatrix()
jExclNPMat = np.array(jExclmat).reshape(4,4) # world rotation matrix of parent joint as numpy 4x4 array
jExclNPMatInv = np.array(jExclmat.inverse()).reshape(4,4) # inverse of parent rotMat (prox) as numpy 4x4 array

# define initial guess condition

initial_guess = np.zeros(3)


# ==== optimise translations ====


frame = 0
tol = 1 + tolerance

# go through all possible combinations

for i in range(keyDiff):

	if minKeys > keyframes:
		break

	j = minKeys - 1 + i # starts at 0, hence need to substract 1

	# check if progress is interupted

	if cmds.progressWindow(query = True, isCancelled = True):
		break

	# extract rotation

	rotation = rotations[j,:]

	# get joint inclusive transformation matrix (child)

	transMat = np.eye(4)
	transMat[0:3,0:3] = sp.spatial.transform.Rotation.from_euler('ZYX', rotation, degrees = True).as_matrix()[::-1,::-1] # inverse matrix directions to be consistent with previous approach (i.e., converting SciPy’s (x,y,z) basis into Maya’s (z,y,x) basis)

	# define initial guess condition

	if not shapeCheck: # cylinder or ellipsoid
			
		initial_guess = (np.array((1.1 * meanR, 0.0, 0.0, 1.0)) @ transMat)[0:3] # set initial guess as 1.1 times the radius in X-axis direction (joint distraction)

		# clip initial guess to cylinder bounds

		bnds = np.array(bounds)
		initial_guess = np.clip(initial_guess, bnds[:,0], bnds[:,1])
		
	# get rotation matrices

	rotMat = []
	rotMat.append(jExclNPMat) # append parent rotMat (prox) as numpy 4x4 array
	rotMat.append(transMat @ jExclNPMat) # append child rotMat (dist) as numpy 4x4 array
	rotMat.append(jExclNPMatInv) # append inverse of parent rotMat (prox) as numpy 4x4 array

	# optimise the joint translations

	coords, viable = optimisePosition(proxArr, 		# 2D array of proximal articular surface vertex coordinates for fast computation of relative coordinates 
									  distArr, 		# 2D array of distal articular surface vertex coordinates for fast computation of relative coordinates 
									  distMeshArr,  # 2D array of distal mesh vertex coordinates for fast computation of relative coordinates
									  SDF, 			# list containing multiple signed distance fields in tricubic form (e.g., [ipProx, ipDist])
									  rotMat, 		# array with the transformation matrices of the joint and its parent
									  thickness, 	# thickness measure correlated with joint spacing
									  weights, 		# weights for the individual cost function terms
									  initial_guess,# initual guess condition for optimiser 
									  bounds, 		# bounds for optimisation
									  scaleFactor, 	# scale factor to roughly check if joint is disarticulated
									  maxIter, 		# maximum number of iterations
									  tol, 			# tolerance for joint proximity
									  cutOff)		# cut off value for signed distance fields

	# check if pose was viable

	if viable:

		frame = cmds.currentTime(query = True)

		# key rotation to animate joint

		cmds.setKeyframe(jointName, at = 'rotateX', v = rotation[0])
		cmds.setKeyframe(jointName, at = 'rotateY', v = rotation[1])
		cmds.setKeyframe(jointName, at = 'rotateZ', v = rotation[2])

		# key translation

		cmds.setKeyframe(jointName, at = 'translateX', v = coords[0])
		cmds.setKeyframe(jointName, at = 'translateY', v = coords[1])
		cmds.setKeyframe(jointName, at = 'translateZ', v = coords[2])

		# update time

		cmds.currentTime(frame + 1)

	# update progress bar

	cmds.progressWindow(edit = True, progress = i + 1, status = 'Processing frame {0} of {1} frames'.format(i + 1, keyDiff))

# when done close progress bar

end = time.time()

if cmds.progressWindow(query = True, isCancelled = True):
	print('# Abort: Translation optimisation cancelled after {0:.3f} seconds. Total {1} frames tested and keyed {2} viable frames'.format(end - mid, i + 1, int(frame - lastKey)))
else:
	print('# Result: Translation optimisation done in {0:.3f} seconds! Successfully tested {1} frames and keyed {2} viable frames.'.format(end - mid, keyDiff, int(frame - lastKey)))

cmds.progressWindow(edit = True, endProgress = True)
