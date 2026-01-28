#	runROMmapper.py
#
#	This script estimates the (mobile) joint centre position and moves the distal bone
#	mesh into position for a set of presribed joint orientations. Only feasible joint 
#	positions will be keyed. The resulting set of keyed frames represent vialbe joint 
#	positions and orientations. It is an implementation of the Marai et al., 2006 and 
#	Lee et al., 2023 approach for Autodesk Maya.
#
#	Written by Oliver Demuth
#	Last updated 28.01.2026 - Oliver Demuth
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
gridScale = 1							# Float value for the scale factor of the cubic grid (i.e., 1.5 initialises the grid from -1.5 to 1.5)
interval = 5							# sampling interval, see Manafzadeh & Padian, 2018, (e.g., for FE and LAR -180:interval:180, and for ABAD -90:interval:90)
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

sphereRad = meanRad(fittedShape)
thickness = sphereRad/2
gridSize = 8 * sphereRad

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

	nvit = []

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

# create 3D grid for rotations

xRots = np.arange(xBounds[0], ceil(xBounds[1]+interval/2), interval, dtype = float)
yRots = np.arange(yBounds[0], ceil(yBounds[1]+interval/2), interval, dtype = float)
zRots = np.arange(zBounds[0], ceil(zBounds[1]+interval/2), interval, dtype = float)

rotX, rotY, rotZ = np.meshgrid(xRots, yRots, zRots, indexing='ij')
rotations = np.vstack((rotX.ravel(), rotY.ravel(), rotZ.ravel())).T 

numFrames = len(rotations)

# set time to 1 or to start frame

if ContinueKeys:
	lastKey = cmds.keyframe(jointName, attribute='rotateX', query=True, index=(1, cmds.keyframe(jointName, attribute='rotateX', query=True, keyframeCount=True)))[-1]

	xRot = round(cmds.keyframe(jointName, attribute='rotateX', query=True, eval=True, time=(lastKey,lastKey))[0],6)
	yRot = round(cmds.keyframe(jointName, attribute='rotateY', query=True, eval=True, time=(lastKey,lastKey))[0],6)
	zRot = round(cmds.keyframe(jointName, attribute='rotateZ', query=True, eval=True, time=(lastKey,lastKey))[0],6)

	rotIdx = rotations.index([xRot,yRot,zRot]) # get index of last keyed frame

	minKeys = rotIdx + 2 # get index of next frame to be keyed

	# set current time

	cmds.currentTime(lastKey + 1)

else:
	if StartFrame == None:
		minKeys = 1 # get index of next frame to be keyed
		cmds.cutKey(jointName, option="keys") # delete all previous keyframes
	else: 
		minKeys = StartFrame

	lastKey = 0	

	# set current time

	cmds.currentTime(minKeys)

# get total number of frames to be keyed

if FrameInterval == None or (minKeys + FrameInterval) > numFrames:
	keyframes = numFrames
	keyDiff = keyframes - minKeys + 1

else:
	keyframes = minKeys + FrameInterval
	keyDiff = FrameInterval

if keyDiff <= 0:
	keyDiff = 1

if debug == 1 and FrameInterval != None: # randomly assign rotations for testing
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

jExclNPMat = np.array(jDag.exclusiveMatrix()).reshape(4,4) # world rotation matrix of parent joint as numpy 4x4 array
jExclNPMatInv = np.linalg.solve(jExclNPMat, np.eye(4)) # inverse of parent rotMat (prox) as numpy 4x4 array

# define initial guess condition

initial_guess = np.zeros(3)

# ==== optimise translations ====

frame = 0

# go through all possible combinations

for i in range(keyDiff):

	if minKeys > keyframes:
		break

	j = minKeys - 1 + i # starts at 0, hence need to substract 1

	# check if progress is interupted

	if cmds.progressWindow(query=True, isCancelled=True):
			break

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

	coords, viable = optimisePosition(proxArr, distArr, distMeshArr, SDF, rotMat, thickness, initial_guess, gridSize)

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
	print('# Abort: Translation optimisation cancelled after {0:.3f} seconds. Total {1} frames tested and keyed {2} viable frames'.format(end - mid,i+1,int(frame-lastKey)))
else:
	print('# Result: Translation optimisation done in {0:.3f} seconds! Successfully tested {1} frames and keyed {2} viable frames.'.format(end - mid,keyDiff,int(frame-lastKey)))

cmds.progressWindow(edit = True, endProgress = True)
