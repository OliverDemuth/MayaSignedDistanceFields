#	ligamentCalculationWrapper.py
#
#	Mayapy file for running the ligament calculations on multiple models/trials. Models/
#	trials should be set up and arranged within a single folder, then the ligament 
#	wrapper can be run directly from command line using mayapy. 
#
#	This script wraps the ligament length calculation and exports the length of a 
#	ligament from origin to insertion, wrapping around the proximal and distal bone 
#	meshes, for each frame. 
#
#	Written by Oliver Demuth
#	Last updated 17.02.2023 - Oliver Demuth
#
#
#	Note, for each ligament create a float attribute at 'jointName' and name it 
#	accordingly. Rename the strings in the user defined variables below according to the
#	objects in your Maya scene and make sure that the naming convention for the ligament 
#	origins and insertions is correct, i.e. the locators should be named 'ligament*_orig'
#	and 'ligament*_ins' for an attribute in 'jointName' called 'ligament*'.
#	
#
#	This script relies on the following other (Python) script(s) which need to be in the
#	same folder before executing this script
#
#		- 'ligamentCalculation.py'
#		- 'progressBar.py'
#
#	For further information please check the Python script(s) referenced above
#
#	How to execute this script:
#
#		On Windows:	
#			Change the directory to your Maya Python 3 Interpretor in the command line: 
#				cd C:\Program Files\Autodesk\Maya<VersionNumber>\bin\
#			then run the following command:
#				mayapy.exe "path/to/ligamentCalculationWrapper.py"
#
#		On macOS:
#			Change the directory to your Maya Python 3 Interpretor in the terminal:
#				cd /Applications/Autodesk/maya<VersionNumber>/Maya.app/Contents/bin/
#			then run the following command:
#				./mayapy "path/to/ligamentCalculationWrapper.py"
#
#	Note, 'path/to/ligamentCalculationWrapper.py' represents your file path to this 
#	Python script and shoud be the same as the path variable specified below.


#################################################
# ========== user defined variables  ========== #
#################################################

# ========== set variables ========== 

jointName = 'myJoint' 				# specify according to the joint centre in the Maya scene, i.e. the name of a locator or joint, e.g. 'myJoint' if following the ROM mapping protocol of Manafzadeh & Padian 2018
meshes = ['bones_boo']				# specify according to meshes or boolean object in the Maya scene
gridSubdiv = 16						# Integer value for the subdivision of the cube, i.e., number of grid points for any given axis, e.g., 20 will result in a cube grid with 21 x 11 x 11 grid points. Please note this scales to O((n+1)^3)
ligSubdiv = 20						# Integer value for the number of ligament segments, e.g., 20, see. Marai et al., 2004 for details
FrameInterval = 300					# Integer value to specify number of frames to be keyed. If all frames are to be keyed set to standard value: None


# ========== set directories ========== 

path = '\\Volumes\\PhD2\\XROMM\\Maya\\ligaments\\python' # add your file path. Make sure 'ligamentLength.py' is in this folder

fileDir = '/Volumes/PhD2/XROMM/Maya/ligaments/maya files' # path for Maya files
outDir = '/Volumes/PhD2/XROMM/Maya/ligaments/results' # path for Maya output

#########################################################################################

#################################################
# ==========    main script below    ========== #
#################################################

# ========== initialise ========== 

import maya.standalone
maya.standalone.initialize(name='python')

import sys
sys.path.append(path)

# ========== load plugins  ========== 

import maya.api.OpenMaya as om
import maya.cmds as cmds
import numpy as np
import scipy as sp
import progressBar as pb
import os
import time

from tricubic import tricubic
from ligamentCalculation import * # source the ligament functions


# ========== simulation setup ========== 

# supress error messages

cmds.scriptEditorInfo(sw=True,se=True)

# get Maya files

models = os.listdir(fileDir)
models[:] = [item for item in models if not item.startswith('._')] # remove macOS specific files from list

# go through each Maya file individually

for mod in range(0,len(models)):
	
	# open Maya scene and initialise calculations 

	modi = fileDir + '/' + models[mod]
	cmds.file(modi, open=True, force=True)

	# set to starting frame 

	cmds.currentTime(1)

	# get number of ligaments and their names 

	LigAttributes = cmds.listAttr(jointName,ud=True) # get user defined attributes of 'jointName', i.e. the float attributes that will contain the ligament lengths

	# get ligament names

	ligNames = []
	for i in range(len(LigAttributes)):
		ligNames.append(LigAttributes[i])

	# initialise results array

	ligRes = []
	ligRes.append(ligNames) # set array header

	# get total number of keyed frames from 'jointName', i.e., max number of frames to be calculated

	if FrameInterval == None:
		frames = cmds.keyframe(jointName, attribute='rotateX', query=True, keyframeCount=True)
	else:
		frames = FrameInterval

	# initialise progress bar

	start = time.time()

	print('Current file: ', models[mod])
	pb.printProgressBar(0, frames, prefix = 'Ligament calculation in progress:', suffix = 'Complete. Calculating {} frames'.format(frames), length = 50)


	# ========== run ligamentCalculation script ==========

	# go through each frame and calculate ligament lengths 

	for j in range(frames):

		# calculate the length of the ligaments for the current frame and append it to results array

		ligRes.append(ligCalc(jointName, meshes, gridSubdiv ,ligSubdiv)[1]) # add row of ligament lengths to result array
		
		# update progress bar

		progress = j + 1
		pb.printProgressBar(progress, frames, prefix = 'Ligament calculation in progress:', suffix = 'Complete. Frame {0} of {1}.          '.format(progress,frames), length = 50)

		# go to next frame and update progress bar

		cmds.currentTime(j + 2)
		
	
	# ========== Save results ==========

	# define outpule file name

	namei = models[mod]
	namei= namei.replace('.mb','')
	fname = outDir + '/' + namei + '.csv'
	
	# write output file

	with open(fname, 'w') as filehandle:
		for listitem in ligRes:
			s = ",".join(map(str, listitem))
			filehandle.write('%s\n' % s)
	
	# shut down the Maya scene

	cmds.file(modified=0) 

	# print simulation time for each Maya scene

	end = time.time()
	convert = time.strftime("%H hours %M minutes %S seconds", time.gmtime(end - start))
	print('Ligament calculation for {0} done in {1}!'.format(models[mod],convert))

	cmds.file(new=True)

# close Maya

maya.standalone.uninitialize()

