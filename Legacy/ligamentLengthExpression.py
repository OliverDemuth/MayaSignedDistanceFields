#	ligamentLengthExpression.py
#
#	This script calculates and keys the length of a ligament from origin to insertion,
#	wrapping around the proximal and distal bone meshes, for each frame. The script can
#	be apported by pressing 'esc' and the already keyed frames will not be lost.
#
#	Written by Oliver Demuth 01.08.2022
#	Last updated 08.08.2022 - Oliver Demuth
#
#
#	Note, for each ligament create a float attribute at 'JointName' and name it accordingly.  
#	Rename the strings in the user defined variables below according to the objects in your 
#	Maya scene and make sure that the naming convention for the ligament origins and
#	insertions is correct, i.e. the locators should be named 'ligament1_orig' and
#	'ligament1_ins' for an attribute in 'JointName' called 'ligament1'.
#	
#
#	This script relies on the following other (PyMEL/Python) script(s) which need to be run
#	before executing this script:
#
#		- 'ligamentLength.py'
#
#	For further information check the PyMEL script(s) referenced above


# ===============================================
# ==========  user defined variables   ==========
# ===============================================


JointName = 'myJoint' 			# specify according to the joint centre in the Maya scene, i.e. the name of a locator or joint, e.g. 'myJoint' if following the ROM mapping protocol of Manafzadeh & Padian 2018

ligPlane ='ligament_plane'		# specify according to poly plane in the Maya scene that will act as cutting plane

booObject = 'bones_boo'			# specify according to boolean object in the Maya scene

FrameInterval = None			# integer value to specify number of frames to be keyed, e.g., 100. If all frames are to be keyed set to standard value: None


# ===============================================
# ==========     main script below     ==========
# ===============================================


# load plugins

import pymel.core as pm
import time

# get number of ligaments and their names 

LigAttributes = pm.listAttr(JointName,ud=True) # get user defined attributes of 'JointName', i.e. the float attributes that will contain the ligament lengths
LigOrigins = []
LigInsertions = []
prevKeyedFrames = []

# get number of frames of previously calculated ligament lengths

for i in range(len(LigAttributes)):

	# get number of previously keyed frames per custom attribute

	prevKeyedFrames.append(pm.keyframe(JointName, attribute=LigAttributes[i], query=True, keyframeCount=True))

	# get names of ligament origins, insertions and boolean objects

	LigOrigins.append(LigAttributes[i]+'_orig')
	LigInsertions.append(LigAttributes[i]+'_ins')

prevKeyedFrames.sort()
minKeys = prevKeyedFrames[0]

# set time to 1 or to previous frame

frame = pm.currentTime(query=True)

if frame != 1:
	if prevKeyedFrames[0] == 0:
		pm.currentTime(1)
	else: 
		pm.currentTime(minKeys+1)

# get total number of keyed frames from 'JointName'

maxKeys = pm.keyframe(JointName, attribute='rotateX', query=True, keyframeCount=True)

if FrameInterval == None or (minKeys+FrameInterval) > maxKeys:
	keyframes = maxKeys
else:
	keyframes = minKeys+FrameInterval

keyDiff = keyframes-minKeys

if keyDiff <=0:
	keyDiff = 1

# define progress bar

pm.progressWindow(title='Ligament calculation in progress...',
				  progress=1,
				  status='Processing frame {0} of {1} frames'.format(1,keyDiff),
				  isInterruptable=True, 
				  max=keyDiff)

print('Ligament calculation in progress...')

start = time.time()

# go through each frame and key ligament lengths into attributes

for i in range(keyDiff):

	if minKeys > keyframes:
		break

	j = minKeys+i+1

	# check if progress is interupted

	if pm.progressWindow( query=True, isCancelled=True ):
			break

	# calculate the length of each ligament and key the attribute on the animated joint

	for index, ligament in enumerate(LigAttributes):

		ligament = ligLength(LigOrigins[index], LigInsertions[index], JointName, ligPlane, booObject)
		if ligament[0] == 0:
			ligament = ligLength(LigOrigins[index], LigInsertions[index], JointName, ligPlane, booObject, reverse = True)
		pm.setKeyframe(JointName, at = LigAttributes[index], v = ligament[1])

	# update progress bar and time

	pm.progressWindow( edit=True, progress=i+1, status='Processing frame {0} of {1} frames'.format(i+1,keyDiff) )
	pm.currentTime(j+1)

# when done close progress bar

end = time.time()

if pm.progressWindow( query=True, isCancelled=True ):
    print('# Abort: Ligament calculation cancelled after {0} seconds. Total frames keyed: {1}'.format(end - start,i))
else:
    print('# Result: Ligament calculation done in {} seconds!'.format(end - start))

pm.progressWindow( edit=True, endProgress=True )


