#	runROMmapper1.1.py
#
#	This script estimates the (mobile) joint centre position and moves the distal bone
#	mesh into position for a set of presribed joint orientations. Only feasible joint 
#	positions will be keyed. The resulting set of keyed frames represent vialbe joint 
#	positions and orientations. It is an implementation of the Marai et al., 2006. 
#	approach for Autodesk Maya.
#
#	Written by Oliver Demuth
#	Last updated 13.11.2024 - Oliver Demuth
#
#
#	Rename the strings in the user defined variables below according to the objects in
#	your Maya scene and define the other attributes accordingly.
#	
#
#	This script relies on the following other (Python) script(s) which need to be run
#	in the Maya script editor before executing this script:
#
#		- 'ROMmapper1.1.py'
#
#	For further information please check the Python script(s) referenced above


#################################################
# ========== user defined variables  ========== #
#################################################


jointName = 'myJoint' 						# specify according to the joint centre in the Maya scene, i.e. the name of a locator or joint (e.g. 'myJoint' if following the ROM mapping protocol of Manafzadeh & Padian 2018)

meshes = ['RLP3_scapulocoracoid', 'RLP3_humerus']		# specify according to meshes in the Maya scene

artSurfMeshes = ['RLP3_glenoid_art_surf', 'RLP3_humerus']	# specify according to meshes in the Maya scene

proxFittedShape = 'RLP3_glenoid_fitted_sphere_Mesh'		# specify according to mesh in the Maya scene

gridSubdiv = 100						# integer value for the subdivision of the cube, i.e., number of grid points per axis (e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points)

interval = 90							# sampling interval, see Manafzadeh & Padian, 2018, (e.g., for FE and LAR -180:interval:180, and for ABAD -90:interval:90)

debug = 0 							# debug mode to check if signed distance fields have already been calculated


#################################################
# ==========    main script below    ========== #
#################################################


# ========== load modules ==========

import maya.api.OpenMaya as om
import maya.cmds as cmds
import numpy as np
import scipy as sp
import time

from tricubic import tricubic
from math import ceil

# ========================================


# set current time to 1

cmds.currentTime(1)
frame = cmds.currentTime(query=True)

start = time.time()

# reset joint

cmds.rotate(0, 0, 0, jointName)
cmds.move(0, 0, 0, jointName)

if debug == 1:
	# check if distance fields have already been calculated
	try:
		sigDistFieldArray
	except NameError:
		var_exists = False
	else:
		var_exists = True # signed distance field already calculated, no need to do it again
else:
	var_exists = False

# get gridsize from glenoid sphere radius

sphereRad = meanRad(proxFittedShape)
thickness = sphereRad/2
gridSize = 6 * sphereRad

if var_exists == False:
	# calculate signed distance fields
	print('Calculating signed distance fields...')
	sigDistFieldArray, localPoints, worldPoints, initialRotMat = sigDistField(jointName, meshes, gridSubdiv, gridSize)

# get dimensions of cubic grids

dims = sigDistFieldArray[0].shape
proxSigDistList = sigDistFieldArray[0].tolist()
distSigDistList = sigDistFieldArray[1].tolist()

# initialise tricubic interpolator with signed distance data on default cubic grid

ipProx = tricubic(proxSigDistList, [dims[0], dims[1], dims[2]]) # grid will be initialised in its relative coordinate system from [0,0,0] to [gridSubdiv+1, gridSubdiv+1, gridSubdiv+1].
ipDist = tricubic(distSigDistList, [dims[0], dims[1], dims[2]]) # grid will be initialised in its relative coordinate system from [0,0,0] to [gridSubdiv+1, gridSubdiv+1, gridSubdiv+1].

# get corner points of cubic grids (both grids are set up identically)

origPos = om.MVector(localPoints[0])
zVecPos = om.MVector(localPoints[dims[1] - 1])
yVecPos = om.MVector(localPoints[dims[1] * (dims[2] - 1)])
xVecPos = om.MVector(localPoints[dims[1] * dims[2] * (dims[0] - 1)])

# get direction vectors to cubic grid corners and normalize by number of grid subdivisions

xDir = (xVecPos - origPos) / (dims[0] - 1)
yDir = (yVecPos - origPos) / (dims[1] - 1)
zDir = (zVecPos - origPos) / (dims[2] - 1)

# get rotation matrix of default cubic grid coordinate system

gridRotMat = om.MMatrix([xDir.x, 	xDir.y,		xDir.z, 	0,
			 yDir.x, 	yDir.y, 	yDir.z, 	0,
			 zDir.x, 	zDir.y, 	zDir.z, 	0,
			 origPos.x,	origPos.y,	origPos.z,	1]) 

mid = time.time()

print('Signed distance fields calculated in {0:.3f} seconds!'.format(mid - start))

# create 3D grid for rotations

xRots = zRots = np.arange(-180, ceil(180+interval/2), interval, dtype = int)
yRots = np.arange(-90, ceil(90+interval/2), interval, dtype = int)

rotations = [[x, y, z] for x in xRots  # length along x
		       for y in yRots  # length along y
		       for z in zRots] # length along z

# define progress bar

numFrames = len(rotations)

cmds.progressWindow(title = 'Translation optimisation in progress...',
		    progress = 1,
		    status = 'Processing frame {0} of {1} frames'.format(1,numFrames),
		    isInterruptable = True, 
		    max = numFrames)

print('Translation optimisation in progress...')

# get dag path for joint

jDag = dagObjFromName(jointName)[1]

# calculate relative position of articular surfaces

proxCoords = relVtcPos(artSurfMeshes[0], initialRotMat[0])
distCoords = relVtcPos(artSurfMeshes[1], initialRotMat[1])

# get joint exclusive transformation matrix (parent)

jExclTransMat = om.MTransformationMatrix(jDag.exclusiveMatrix()) # world transformation matrix of parent of joint
jExclTransMat.setScale([gridSize,gridSize,gridSize],om.MSpace.kWorld) # set scale in world space

# go through all possible combinations

for i in range(numFrames):

	# check if progress is interupted

	if cmds.progressWindow(query=True, isCancelled=True):
		break
			
	# reset joint

	cmds.rotate(0, 0, 0, jointName)
	cmds.move(0, 0, 0, jointName)

	# set rotation

	rotation = [float(np.round(rotations[i][0], 10)),float(np.round(rotations[i][1], 10)),float(np.round(rotations[i][2], 10))]
	cmds.rotate(rotation[0], rotation[1], rotation[2], jointName)

	# get joint inclusive transformation matrix (child)

	jInclTransMat = om.MTransformationMatrix(jDag.inclusiveMatrix()) # world transformation matrix of joint
	jInclMat = om.MTransformationMatrix(jDag.inclusiveMatrix())
	jInclTransMat.setScale([gridSize,gridSize,gridSize],om.MSpace.kWorld) # set scale in world space

	# get rotation matrices

	rotMat = []
	rotMat.append(om.MMatrix(jExclTransMat.asMatrix())) # parent rotMat (prox)
	rotMat.append(om.MMatrix(jInclTransMat.asMatrix())) # child rotMat (dist)
	rotMat.append(jDag.exclusiveMatrix()) # get parent rotMat (prox) without scale

	# optimise the joint translations

	coords, viable, results = optimisePosition(proxCoords, distCoords, ipProx, ipDist, gridRotMat, rotMat, thickness)

	# check if pose was viable

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

	cmds.progressWindow(edit=True, progress=i+1, status='Processing frame {0} of {1} frames'.format(i+1,numFrames))

# when done close progress bar

end = time.time()

if cmds.progressWindow(query=True, isCancelled=True):
	print('# Abort: Translation optimisation cancelled after {0:.3f} seconds. Total {1} frames tested and keyed {2} viable frames'.format(end - mid,i+1,int(frame-1)))
else:
	print('# Result: Translation optimisation done in {0:.3f} seconds! Successfully tested {1} frames append keyed {2} viable frames.'.format(end - mid,i+1,int(frame-1)))

cmds.progressWindow(edit=True, endProgress=True)
