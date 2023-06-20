#	runLigamentCalculation.py
#
#	This script calculates and keys the length of a ligament from origin to insertion,
#	wrapping around the proximal and distal bone meshes, for each frame. The script can
#	be apported by pressing 'esc' and the already keyed frames will not be lost.
#
#	Written by Oliver Demuth
#	Last updated 09.06.2023 - Oliver Demuth
#
#
#	Note, for each ligament create a float attribute at 'jointName' and name it 
#	accordingly. Rename the strings in the user defined variables below according to the
#	objects in your Maya scene and make sure that the naming convention for the ligament 
#	origins and insertions is correct, i.e. the locators should be named 'ligament*_orig'
#	and 'ligament*_ins' for an attribute in 'jointName' called 'ligament*'.
#	
#
#	This script relies on the following other (Python) script(s) which need to be run
#	in the Maya script editor before executing this script:
#
#		- 'ligamentCalculation.py'
#
#	For further information please check the Python script(s) referenced above


#################################################
# ========== user defined variables  ========== #
#################################################


jointName = 'myJoint' 		# specify according to the joint centre in the Maya scene, i.e. the name of a locator or joint, e.g. 'myJoint' if following the ROM mapping protocol of Manafzadeh & Padian 2018

meshes = ['bones_boo']		# specify according to meshes or boolean object in the Maya scene

gridSubdiv = 16			# Integer value for the subdivision of the cube, i.e., number of grid points per axis, e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points

ligSubdiv = 20			# Integer value for the number of ligament segments, e.g., 20, see Marai et al., 2004 for details

StartFrame = None 		# Integer value to specify the start frame. If all frames are to be keyed from the beginning (Frame 1) set to standard value: None or 1.

FrameInterval = None		# Integer value to specify number of frames to be keyed. If all frames are to be keyed set to standard value: None

KeyPathPoints = False		# Boolean to specify whether ligament point positions are to be keyed or not. True = yes, False = no




#################################################
# ==========    main script below    ========== #
#################################################


# ============= load modules =============

import maya.cmds as cmds
import time

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
else:
	keyframes = minKeys + FrameInterval

keyDiff = keyframes - minKeys + 1

if keyDiff <=0:
	keyDiff = 1

# set current time

cmds.currentTime(minKeys)
start = time.time()

# define progress bar

cmds.progressWindow(title = 'Ligament calculation in progress...',
		    progress = 1,
		    status = 'Processing frame {0} of {1} frames'.format(1,keyDiff),
		    isInterruptable = True, 
		    max = keyDiff)

print('Ligament calculation in progress...')

# check if ligament points are to be keyed

if KeyPathPoints == True:

	LigAttributes = cmds.listAttr(jointName,ud=True) # get user defined attributes of 'jointName', i.e. the float attributes that will contain the ligament lengths

	for ligament in LigAttributes:

		# check if groups and their locators exist

		if not cmds.objExists(ligament + '_LOC_GRP'): 
			cmds.group(em = True, name = ligament + '_LOC_GRP') # create group if it doesn't exist already

		for k in range(ligSubdiv + 1):

			# get locator name

			loc = ligament + '_LOC_' + str(k)

			# check if locators exists

			if not cmds.objExists(loc):
			       cmds.spaceLocator(name = loc) # create locator
			       cmds.parent(loc, ligament + '_LOC_GRP') # parent locator under their ligament locator group

# go through each frame and key ligament lengths into attributes

for i in range(keyDiff):

	if minKeys > keyframes:
		break

	j = minKeys + i

	# check if progress is interupted

	if cmds.progressWindow(query=True, isCancelled=True):
			break

	# calculate the length of each ligament 

	ligAttributes,pathLengths,ligPoints,results = ligCalc(jointName, meshes, gridSubdiv ,ligSubdiv)

	# key the attributes on the animated joint

	for index, ligament in enumerate(ligAttributes):
	    
		if results[index].status == 0:
			cmds.setKeyframe(jointName, at = ligament, v = pathLengths[index])
		else:
			cmds.setKeyframe(jointName, at = ligament, v = -1)

		# check if ligament points are to be keyed

		if KeyPathPoints == True:

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
	print('# Abort: Ligament calculation cancelled after {0:.3f} seconds. Total frames keyed: {1}'.format(end - start,i))
else:
	print('# Result: Ligament calculation done in {0:.3f} seconds! Successfully keyed {1} frames.'.format(end - start,keyDiff))

cmds.progressWindow( edit=True, endProgress=True )


