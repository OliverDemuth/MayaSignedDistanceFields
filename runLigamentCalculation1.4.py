#	runLigamentCalculation1.4.py
#
#	This script calculates and keys the length of a ligament from origin to insertion,
#	wrapping around the proximal and distal bone meshes, for each frame. The script can
#	be apported by pressing 'esc' and the already keyed frames will not be lost.
#
#	Written by Oliver Demuth
#	Last updated 28.01.2025 - Oliver Demuth
#
#
#	Note, for each ligament create a float attribute at 'jointName' and name it 
#	accordingly. Rename the strings in the user defined variables below according to the
#	objects in your Maya scene and make sure that the naming convention for the ligament 
#	origins and insertions is correct (i.e., the locators should be named 'ligament*_orig'
#	and 'ligament*_ins' for an attribute in 'jointName' called 'ligament*').
#	
#
#	This script relies on the following other (Python) script(s) which need to be run
#	in the Maya script editor before executing this script:
#
#		- 'ligamentCalculation1.4.py'
#
#	For further information please check the Python script(s) referenced above


#################################################
# ========== user defined variables  ========== #
#################################################


jointName = 'myJoint' 			# specify according to the joint centre in the Maya scene, i.e. the name of a locator or joint (e.g. 'myJoint' if following the ROM mapping protocol of Manafzadeh & Padian 2018)
meshes = ['prox_mesh', 'dist_mesh']	# specify according to meshes or boolean object in the Maya scene
gridSubdiv = 100			# Integer value for the subdivision of the cube, i.e., number of grid points per axis (e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points)
ligSubdiv = 20				# Integer value for the number of ligament segments (e.g., 20, see Marai et al., 2004 for details)
StartFrame = None 			# Integer value to specify the start frame. If all frames are to be keyed from the beginning (Frame 1) set to standard value: None or 1.
FrameInterval = None			# Integer value to specify number of frames to be keyed. If all frames are to be keyed set to standard value: None
KeyPathPoints = False			# Boolean to specify whether ligament point positions are to be keyed or not. True = yes, False = no
debug = 0 				# Debug mode to check if signed distance fields have already been calculated



#################################################
# ==========    main script below    ========== #
#################################################


# ============= load modules =============

import maya.cmds as cmds
import numpy as np
import scipy as sp
import maya.api.OpenMaya as om
import time

from maya.api.OpenMaya import MPoint, MVector, MTransformationMatrix
from math import sqrt
from tricubic import tricubic

# ========================================


# set time to 1 or to start frame

frame = cmds.currentTime(query=True)

if StartFrame == None:
	minKeys = 1
else: 
	minKeys = StartFrame

# get total number of keyed frames from 'jointName'

maxKeys = cmds.keyframe(jointName, attribute='rotateX', query=True, keyframeCount=True)

if FrameInterval == None or (minKeys + FrameInterval) > maxKeys:
    keyframes = maxKeys
    keyDiff = keyframes - minKeys + 1
	
else:
    keyframes = minKeys + FrameInterval
    keyDiff = keyframes - minKeys


if keyDiff <= 0:
	keyDiff = 1

start = time.time()

var_exists = False

if debug == 1:

	# check if distance fields have already been calculated

	try:
		sigDistFieldArray
	except NameError:
		var_exists = False
	else:
		var_exists = True # signed distance field already calculated, no need to do it again

if not var_exists:
	# calculate signed distance fields

	print('Calculating signed distance fields...')
	
	cmds.currentTime(0)
	cmds.move(0,0,0, jointName, localSpace=True)
	cmds.rotate(0,0,0,jointName)
	
	sigDistFieldArray, localPoints, LigAttributes, gridScale = sigDistField(jointName, meshes, gridSubdiv)
	
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

if not var_exists:
	print('Signed distance fields calculated in {0:.3f} seconds!'.format(mid - start))
else:
	print('Signed distance fields succesfully loaded in {0:.3f} seconds!'.format(mid - start))

# define progress bar

cmds.progressWindow(title = 'Ligament calculation in progress...',
		    progress = 1,
		    status = 'Processing frame {0} of {1} frames'.format(1,keyDiff),
		    isInterruptable = True, 
		    max = keyDiff)

print('Ligament calculation in progress...')

# set current time

cmds.currentTime(minKeys)

# check if ligament points are to be keyed

if KeyPathPoints:

	for ligament in LigAttributes:

		# check if groups and their locators exist

		lig_GRP = ligament + '_LOC_GRP'

		if not cmds.objExists(lig_GRP): 
			cmds.group(em = True, name = lig_GRP) # create group if it doesn't exist already

		for k in range(ligSubdiv + 1):

			# get locator name

			loc = ligament + '_LOC_' + str(k)

			# check if locators exists

			if not cmds.objExists(loc):
			       cmds.spaceLocator(name = loc) # create locator
			       cmds.parent(loc, lig_GRP) # parent locator under their ligament locator group

# define constant x coords

x = np.linspace(0.0, 1.0, num = ligSubdiv + 1, endpoint=True)

# go through each frame and key ligament lengths into attributes

for i in range(keyDiff):

	if minKeys > keyframes:
		break

	j = minKeys + i

	# check if progress is interupted

	if cmds.progressWindow(query=True, isCancelled=True):
			break

	# get joint centre position

	jPos = getWSPos(jointName)
	jDag = dagObjFromName(jointName)[1]
	jInclTransMat = MTransformationMatrix(jDag.inclusiveMatrix()) # world transformation matrix of joint
	jExclTransMat = MTransformationMatrix(jDag.exclusiveMatrix()) # world transformation matrix of parent of joint

	# normalize matrices by gridScale

	jInclTransMat.setScale([gridScale,gridScale,gridScale],4) # set scale in world space (om.MSpace.kWorld = 4)
	jExclTransMat.setScale([gridScale,gridScale,gridScale],4) # set scale in world space (om.MSpace.kWorld = 4)

	# get rotation matrices

	jExclMatInv = np.linalg.inv(np.array(jExclTransMat.asMatrix()).reshape(4,4))
	jInclMatInv = np.linalg.inv(np.array(jInclTransMat.asMatrix()).reshape(4,4))

	rotMat = []
	rotMat.append(np.dot(jExclMatInv,gridRotMat)) # inverse of parent rotMat (prox)
	rotMat.append(np.dot(jInclMatInv,gridRotMat)) # inverse of child rotMat (dist)

	# calculate the length of each ligament 

	pathLengths,ligPoints,results = ligCalc(x, jPos, ipProx, ipDist, rotMat, LigAttributes, KeyPathPoints)

	# key the attributes on the animated joint

	for index, ligament in enumerate(LigAttributes):
	
		cmds.setKeyframe(jointName, at = ligament, v = pathLengths[index])
		
		if debug == 1 and results[index].status != 0: # optimisation not successful, print info why not
		    print(ligament, results[index])

		# check if ligament points are to be keyed

		if KeyPathPoints:

			for k in range(len(ligPoints[index])):

				# get locator name

				loc = ligament + '_LOC_' + str(k)

				# key ligament point positions to locator

				cmds.setKeyframe(loc, at = 'translateX', v = ligPoints[index][k][0])
				cmds.setKeyframe(loc, at = 'translateY', v = ligPoints[index][k][1])
				cmds.setKeyframe(loc, at = 'translateZ', v = ligPoints[index][k][2])

	# update progress bar and time

	cmds.progressWindow(edit=True, progress=i+1, status='Processing frame {0} of {1} frames'.format(i+1,keyDiff))
	cmds.currentTime(j+1)

# when done close progress bar

end = time.time()

if cmds.progressWindow( query=True, isCancelled=True ):
	print('# Abort: Ligament calculation cancelled after {0:.3f} seconds. Total frames keyed: {1}'.format(end - mid,i))
else:
	print('# Result: Ligament calculation done in {0:.3f} seconds! Successfully keyed {1} frames.'.format(end - mid,keyDiff))

cmds.progressWindow( edit=True, endProgress=True )
