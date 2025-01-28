#	ligamentCalculationWrapper.py
#
#	Mayapy file for running the ligament calculations on multiple models/trials. Models/
#	trials should be set up and arranged within a single folder, then the ligament 
#	wrapper can be run directly from command line using mayapy. 
#
#	This script wraps the ligament length calculation and exports the length of a 
#	ligament from origin to insertion wrapping around the proximal and distal bone 
#	meshes for each frame. 
#
#	Written by Oliver Demuth
#	Last updated 28.01.2025 - Oliver Demuth
#
#
#	Note, for each ligament create a float attribute at 'jointName' and name it 
#	accordingly. Rename the strings in the user defined variables below according to the
#	objects in your Maya scene and make sure that the naming convention for the ligament 
#	origins and insertions is correct (i.e. the locators should be named 'ligament*_orig'
#	and 'ligament*_ins' for an attribute in 'jointName' called 'ligament*').
#
#
#	This script relies on the following other (Python) script(s) which need to be in the
#	same folder before executing this script
#
#		- 'ligamentCalculationBatch.py'
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

jointName = 'myJoint' 		# Name of the joint centre, i.e. the name of a locator or joint (e.g., 'myJoint' if following the ROM mapping protocol of Manafzadeh & Padian 2018)
meshes = ['prox_mesh', 		# Names of the two bone meshes (e.g., individual meshes in the form of ['prox_mesh','dist_mesh'])
	  'dist_mesh']	
gridSubdiv = 100		# Integer value for the subdivision of the cube, i.e., number of grid points per axis (e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points)
ligSubdiv = 20			# Integer value for the number of ligament points (e.g., 20 will divide the ligament into 20 equidistant segments, see Marai et al., 2004 for details)
FrameInterval = None		# Integer value to specify number of frames to be keyed. If all frames are to be keyed set to standard value: None
cores = 8			# Integer value to specify number of CPU cores to be assigned. Depending on the number of files and/or avialable CPU cores the actual number can be lower. Maximally two thirds of all cores will be assigned.

# ========== set directories ========== 

# make sure you have set directories as follows:
# 	project folder directory: 	"path/to/project folder"		Project folder with the following subfolders: 'python', 'maya files' and 'results'
#	python scripts directory:	"path/to/project folder/python"		Ligament calculation python scripts go in here
#	Maya files directory:		"path/to/project folder/maya files"	Maya scenes (.mb) to be processed go in here
#	output/results directory:	"path/to/project folder/results"	CSV files will be saved here
# ======================================== #

path = '\\path\\to\\python files' # add your file path. Make sure 'ligamenCalculationWrapper.py' and 'ligamenCalculationBatch.py' are in this folder

fileDir = '/path/to/maya files' # path for Maya files
outDir = '/path/to/results' # path for Maya output


#################################################
# ==========    main script below    ========== #
#################################################

# ========== load modules  ========== 

import sys
import maya.standalone
import maya.api.OpenMaya as om
import maya.cmds as cmds
import numpy as np
import scipy as sp
import functools
import os
import time

from tricubic import tricubic
from maya.api.OpenMaya import MVector, MPoint, MTransformationMatrix
from math import sqrt, floor
from multiprocessing import cpu_count, Process, Queue
from ligamentCalculationBatch import * # source the ligament functions

# ========== append path to python files ========== 

sys.path.append(path)

# ========== simulation setup ========== 

if __name__ == "__main__":

	# get Maya files

	mayaFiles = os.listdir(fileDir)
	mayaFiles[:] = [item for item in mayaFiles if not item.startswith('._') if item.endswith('.mb')] # get Maya scenes and remove macOS specific files from list

	numFiles = len(mayaFiles)

	print('Detected following {0} Maya files within the \'{1}\' directory:'.format(numFiles,fileDir))
	[print(file) for file in mayaFiles]
		
	filePaths = [fileDir + '/' + mayaFile for mayaFile in mayaFiles]

	# get available CPU cores (max two thirds)

	availableCores = floor(cpu_count()/3*2)
	maxCores = availableCores if availableCores <= numFiles else numFiles 
	CPUcores = cores if cores and cores <= maxCores else maxCores

	print('Assigning {0} cores to process {1} Maya files.'.format(CPUcores,numFiles))

	# append maya files to queue

	q = Queue()

	for file in filePaths:
		q.put(file)

	# create tuple for arguments passed to ligament calculation functions

	arguments = (jointName, meshes, gridSubdiv, ligSubdiv, FrameInterval, outDir)

	# initialise multiprocessing

	processes = [Process(target = processMayaFiles, args = (q,arguments)) for _ in range(CPUcores)]

	for process in processes:
		process.start()

	for process in processes:
		process.join()


