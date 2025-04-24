#	runROMmapper.py
#
#	This script estimates the (mobile) joint centre position and moves the distal bone
#	mesh into position for a set of presribed joint orientations. Only feasible joint 
#	positions will be keyed. The resulting set of keyed frames represent vialbe joint 
#	positions and orientations. It is an implementation of the Marai et al., 2006 and 
#	Lee et al., 2023 approach for Autodesk Maya.
#
#	Written by Oliver Demuth
#	Last updated 24.04.2025 - Oliver Demuth
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
meshes = ['RLP3_scapulocoracoid', 			# specify according to meshes in the Maya scene
	  'RLP3_humerus',
	  'RLP3_convex_hull']				
congruencyMeshes = ['RLP3_glenoid_art_surf', 		# specify according to meshes in the Maya scene
		    'RLP3_humeral_head']	
fittedShape = 'RLP3_glenoid_fitted_sphere_Mesh'		# specify according to meshes in the Maya scene		
xBounds = [-180,180]					# bounds for X-axis rotation in the form of [min, max] (i.e., LAR, e.g., [-180,180] for spherical joints or [-90,90] for hinge joints)
yBounds = [-90,90]					# bounds for Y-axis rotation in the form of [min, max] (i.e., ABAD, e.g.,  [-90,90] for spherical joints or [-90,90] for hinge joints)
zBounds = [-180,180]					# bounds for Z-axis rotation in the form of [min, max] (i.e., FE, e.g.,  [-180,180] for spherical joints or [0,180] for hinge joints)
gridSubdiv = 100					# integer value for the subdivision of the cube, i.e., number of grid points per axis (e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points)
gridScale = 1						# Float value for the scale factor of the cubic grid (i.e., 1.5 initialises the grid from -1.5 to 1.5)
interval = 5						# sampling interval, see Manafzadeh & Padian, 2018, (e.g., for FE and LAR -180:interval:180, and for ABAD -90:interval:90)
StartFrame = None 					# Integer value to specify the start frame. If all frames are to be keyed from the beginning (Frame 1) set to standard value: None or 1.
FrameInterval = None					# Integer value to specify number of frames to be keyed. If all frames are to be keyed set to standard value: None
ContinueKeys = False					# Boolean value (True or False) to specify whether a previous simulation should be continued (under the assumption that the interval has not changed)
debug = 0 						# Debug mode to check if signed distance fields have already been calculated


#################################################
# ==========    main script below    ========== #
#################################################


# ========== load modules ==========

import maya.api.OpenMaya as om
from maya.api.OpenMaya import MVector, MPoint, MTransformationMatrix
import maya.cmds as cmds
import numpy as np
import scipy as sp
import time
import random

from tricubic import tricubic
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

	SDF, initialRotMat = sigDistField(jointName, meshes, gridSubdiv, gridSize, gridScale)
	
	# calculate relative position of articular surfaces

	proxArr = relVtcPos(congruencyMeshes[0], initialRotMat[0])
	distArr = relVtcPos(congruencyMeshes[1], initialRotMat[1])
	distMeshArr = relVtcPos(meshes[1], initialRotMat[1])

# get inverse of rotation matrix for default cubic grid coordinate system

gridVec = 2 * gridScale / gridSubdiv
gridRotMat = np.linalg.inv(np.array([[gridVec, 0, 0, 0], # x direction
				     [0, gridVec, 0, 0], # y direction
				     [0, 0, gridVec, 0], # z direction
				     [-gridScale, -gridScale, -gridScale, 1]])) # origin

mid = time.time()

if not var_exists:
	print('Signed distance fields calculated in {0:.3f} seconds!'.format(mid - start))
else:
	print('Signed distance fields succesfully loaded in {0:.3f} seconds!'.format(mid - start))

# create 3D grid for rotations

xRots = np.arange(xBounds[0], ceil(xBounds[1]+interval/2), interval, dtype = int)
yRots = np.arange(yBounds[0], ceil(yBounds[1]+interval/2), interval, dtype = int)
zRots = np.arange(zBounds[0], ceil(zBounds[1]+interval/2), interval, dtype = int)

rotations = [[x, y, z] for x in xRots  # length along x
		       for y in yRots  # length along y
		       for z in zRots] # length along z

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
	rotations = random.sample(rotations, FrameInterval)

# define progress bar

cmds.progressWindow(title = 'Translation optimisation in progress...',
		    progress = 1,
		    status = 'Processing frame {0} of {1} frames'.format(1,keyDiff),
		    isInterruptable = True,
		    max = keyDiff)

print('Translation optimisation in progress...')

# get joint exclusive transformation matrix (parent)

jExclMat = jDag.exclusiveMatrix() # world rotation matrix of parent joint
jExclNPMat = np.array(jExclMat).reshape(4,4) # convert into numpy 4x4 array

jExclTransMat = MTransformationMatrix(jExclMat) # world transformation matrix of parent joint
jExclTransMat.setScale([gridSize,gridSize,gridSize],4) # set scale in world space (om.MSpace.kWorld = 4)
jExclTransNPMat = np.array(jExclTransMat.asMatrix()).reshape(4,4) # convert into numpy 4x4 array
jExclTransNPMatInv = np.linalg.inv(jExclTransNPMat) # inverse of parent rotMat (prox) as numpy 4x4 array

# go through all possible combinations

for i in range(keyDiff):

	if minKeys > keyframes:
		break

	j = minKeys - 1 + i # starts at 0, hence need to substract 1

	# check if progress is interupted

	if cmds.progressWindow(query=True, isCancelled=True):
			break

	# extract rotation

	rotation = [float(rotations[j][0]),float(rotations[j][1]),float(rotations[j][2])]

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
	rotMat.append(jExclTransNPMatInv) # append inverse of parent rotMat (prox) as numpy 4x4 array

	# optimise the joint translations

	coords, viable, results = optimisePosition(proxArr, distArr, distMeshArr, SDF, gridRotMat, rotMat, thickness)

	# check if pose was viable
	
	if debug == 1:
		print(results.nit, viable)

		if viable == 1:
			nvit.append(results.nit)
	    
	if viable == 1:

		frame = cmds.currentTime(query=True)

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

	cmds.progressWindow(edit=True, progress=i+1, status='Processing frame {0} of {1} frames'.format(i+1,keyDiff))

# when done close progress bar

end = time.time()

if cmds.progressWindow(query=True, isCancelled=True):
	print('# Abort: Translation optimisation cancelled after {0:.3f} seconds. Total {1} frames tested and keyed {2} viable frames'.format(end - mid,i+1,int(frame-lastKey)))
else:
	print('# Result: Translation optimisation done in {0:.3f} seconds! Successfully tested {1} frames and keyed {2} viable frames.'.format(end - mid,keyDiff,int(frame-lastKey)))

cmds.progressWindow(edit=True, endProgress=True)

if debug == 1 and len(nvit) > 0:
    print(max(nvit))
