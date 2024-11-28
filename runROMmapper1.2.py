#	runROMmapper1.2.py
#
#	This script estimates the (mobile) joint centre position and moves the distal bone
#	mesh into position for a set of presribed joint orientations. Only feasible joint 
#	positions will be keyed. The resulting set of keyed frames represent vialbe joint 
#	positions and orientations. It is an implementation of the Marai et al., 2006. 
#	approach for Autodesk Maya.
#
#	Written by Oliver Demuth
#	Last updated 28.11.2024 - Oliver Demuth
#
#
#	Rename the strings in the user defined variables below according to the objects in
#	your Maya scene and define the other attributes accordingly.
#	
#
#	This script relies on the following other (Python) script(s) which need to be run
#	in the Maya script editor before executing this script:
#
#		- 'ROMmapper1.2.py'
#
#	For further information please check the Python script(s) referenced above


#################################################
# ========== user defined variables  ========== #
#################################################


jointName = 'myJoint' 						# specify according to the joint centre in the Maya scene, i.e. the name of a locator or joint (e.g. 'myJoint' if following the ROM mapping protocol of Manafzadeh & Padian 2018)
meshes = ['RLP3_scapulocoracoid', 				# specify according to meshes in the Maya scene
	  'RLP3_humerus']				
congruencyMeshes = ['RLP3_glenoid_art_surf', 			# specify according to meshes in the Maya scene
		    'RLP3_humeral_head']	
fittedShapes = ['RLP3_glenoid_fitted_sphere_Mesh',		# specify according to meshes in the Maya scene
		'RLP3_humeral_head_fitted_ellipsoid_Mesh']				
gridSubdiv = 100						# integer value for the subdivision of the cube, i.e., number of grid points per axis (e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points)
interval = 5							# sampling interval, see Manafzadeh & Padian, 2018, (e.g., for FE and LAR -180:interval:180, and for ABAD -90:interval:90)
StartFrame = None 						# Integer value to specify the start frame. If all frames are to be keyed from the beginning (Frame 1) set to standard value: None or 1.
FrameInterval = None						# Integer value to specify number of frames to be keyed. If all frames are to be keyed set to standard value: None
ContinueKeys = False						# Boolean value (True or False) to specify whether a previous simulation should be continued (under the assumption that the interval has not changed)
debug = 0 							# Debug mode to check if signed distance fields have already been calculated


#################################################
# ==========    main script below    ========== #
#################################################


# ========== load modules ==========

import maya.api.OpenMaya as om
from maya.api.OpenMaya import MVector, MMatrix, MPoint
import maya.cmds as cmds
import numpy as np
import scipy as sp
import time

from tricubic import tricubic
from math import ceil

# ========================================

# reset joint

cmds.rotate(0, 0, 0, jointName)
cmds.move(0, 0, 0, jointName)

# get dag path for joint

jDag = dagObjFromName(jointName)[1]

# get gridsize from glenoid sphere radius

sphereRad = meanRad(fittedShapes[0])
thickness = sphereRad/2
gridSize = 8 * sphereRad

# initialise signed distance fields

start = time.time()

if debug == 1 or ContinueKeys == True:

	# check if distance fields have already been calculated

	try:
		sigDistFieldArray
	except NameError:
		var_exists = False
	else:
		var_exists = True # signed distance field already calculated, no need to do it again

else:
	var_exists = False

if var_exists == False:
	# calculate signed distance fields

	print('Calculating signed distance fields...')

	sigDistFieldArray, localPoints, initialRotMat = sigDistField(jointName, meshes, gridSubdiv, gridSize)
	
	# calculate relative position of articular surfaces

	proxCoords = relVtcPos(congruencyMeshes[0], initialRotMat[0])
	distCoords = relVtcPos(congruencyMeshes[1], initialRotMat[1])

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

# get inverse of rotation matrix of default cubic grid coordinate system

gridRotMat = np.linalg.inv(np.array([[xDir.x, xDir.y, xDir.z, 0],
				     [yDir.x, yDir.y, yDir.z, 0],
				     [zDir.x, zDir.y, zDir.z, 0],
				     [origPos.x, origPos.y, origPos.z, 1]])) 

mid = time.time()

print('Signed distance fields calculated in {0:.3f} seconds!'.format(mid - start))

# create 3D grid for rotations

xRots = zRots = np.arange(-180, ceil(180+interval/2), interval, dtype = int)
yRots = np.arange(-90, ceil(90+interval/2), interval, dtype = int)

rotations = [[x, y, z] for x in xRots  # length along x
		       for y in yRots  # length along y
		       for z in zRots] # length along z

numFrames = len(rotations)

# set time to 1 or to start frame

if ContinueKeys == True:
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

if keyDiff <=0:
	keyDiff = 1


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

jExclTransMat = om.MTransformationMatrix(jExclMat) # world transformation matrix of parent joint
jExclTransMat.setScale([gridSize,gridSize,gridSize],4) # set scale in world space (om.MSpace.kWorld = 4)
jExclTransNPMat = np.array(jExclTransMat.asMatrix()).reshape(4,4) # convert into numpy 4x4 array

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

	localTransMat = om.MTransformationMatrix()
	localTransMat.setRotation(om.MEulerRotation(np.deg2rad(rotation), order=0)) # set rotation (om.MEulerRotation.kXYZ = 0)
	localTransMat.setTranslation(MVector([0,0,0]),2) # reset translation (om.MSpace.kObject = 2)

	jInclTransMat = localTransMat.asMatrix()*gridSize*jExclMat
	jInclTransMat[-1] = 1 # reset last element to 1
	
	# get rotation matrices

	rotMat = []
	rotMat.append(jExclTransNPMat) # parent rotMat (prox) as numpy 4x4 array
	rotMat.append(np.array(jInclTransMat).reshape(4,4)) # child rotMat (dist) as numpy 4x4 array
	rotMat.append(jExclNPMat) # get parent rotMat (prox) without scale as numpy 4x4 array

	# optimise the joint translations

	coords, viable, results = optimisePosition(proxCoords, distCoords, ipProx, ipDist, gridRotMat, rotMat, thickness)

	# check if pose was viable
	
	if debug == 1:
	    print(results.nit)

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
	print('# Result: Translation optimisation done in {0:.3f} seconds! Successfully tested {1} frames append keyed {2} viable frames.'.format(end - mid,keyDiff,int(frame-lastKey)))

cmds.progressWindow(edit=True, endProgress=True)



