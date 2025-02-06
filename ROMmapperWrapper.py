#	ROMmapperWrapper.py
#
#	Mayapy file for running the ROM mapper on multiple models/trials. Models/
#	trials should be set up and arranged within a single folder, then the ROM 
#	mapper wrapper can be run directly from command line using mayapy. 
#
#	This script wraps the contact-based positional optimiser and exports the 
#	translations and rotations of viable joint poses for each frame. 
#
#	Written by Oliver Demuth
#	Last updated 06.02.2025 - Oliver Demuth
#
#
#
#	This script relies on the following other (Python) script(s) which need to be in the
#	same folder before executing this script
#
#		- 'ROMmapperBatch.py'
#
#	For further information please check the Python script(s) referenced above
#
#	How to execute this script:
#
#		On Windows:	
#			Change the directory to your Maya Python 3 Interpretor in the command line: 
#				cd C:\Program Files\Autodesk\Maya<VersionNumber>\bin\
#			then run the following command:
#				mayapy.exe "path/to/ROMmapperWrapper.py"
#
#		On macOS:
#			Change the directory to your Maya Python 3 Interpretor in the terminal:
#				cd /Applications/Autodesk/maya<VersionNumber>/Maya.app/Contents/bin/
#			then run the following command:
#				./mayapy "path/to/ROMmapperWrapper.py"
#
#	Note, 'path/to/ROMmapperWrapper.py' represents your file path to this 
#	Python script and shoud be the same as the path variable specified below.


#################################################
# ========== user defined variables  ========== #
#################################################

# ========== set variables ========== 

jointName = 'myJoint' 			# specify according to the joint centre in the Maya scene, i.e. the name of a locator or joint (e.g. 'myJoint' if following the ROM mapping protocol of Manafzadeh & Padian 2018)
meshes = ['prox_mesh', 			# specify according to meshes in the Maya scene
	  'dist_mesh']				
congruencyMeshes = ['prox_art_surf', 	# specify according to meshes in the Maya scene
		    'dist_art_surf']	
fittedShape = 'prox_sphere'		# specify according to meshes in the Maya scene		
xBounds = [-180,180]			# bounds for X-axis rotation in the form of [min, max] (i.e., LAR, e.g., [-180,180] for spherical joints or [-90,90] for hinge joints)
yBounds = [-90,90]			# bounds for Y-axis rotation in the form of [min, max] (i.e., ABAD, e.g.,  [-90,90] for spherical joints or [-90,90] for hinge joints)
zBounds = [-180,180]			# bounds for Z-axis rotation in the form of [min, max] (i.e., FE, e.g.,  [-180,180] for spherical joints or [0,180] for hinge joints)
interval = 5				# sampling interval, see Manafzadeh & Padian, 2018, (e.g., for FE and LAR -180:interval:180, and for ABAD -90:interval:90)
gridSubdiv = 100			# integer value for the subdivision of the cube, i.e., number of grid points per axis (e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points)
cores = 8				# Integer value to specify number of CPU cores to be assigned

# ========== set directories ========== 

# make sure you have set directories as follows:
# 	project folder directory: 	"path/to/project folder"		Project folder with the following subfolders: 'python', 'maya files' and 'results'
#	python scripts directory:	"path/to/project folder/python"		Ligament calculation python scripts go in here
#	Maya files directory:		"path/to/project folder/maya files"	Maya scenes (.mb) to be processed go in here
#	output/results directory:	"path/to/project folder/results"	CSV files will be saved here
# ======================================== #

path = '\\Users\\itz\\Documents\\Cambridge\\PhD\\Data_Chapter_02\\Maya\\ROM\\python' # add your file path. Make sure 'ligamenCalculationWrapper.py' and 'ligamenCalculationBatch.py' are in this folder

fileDir = '/Users/itz/Documents/Cambridge/PhD/Data_Chapter_02/Maya/ROM/maya files' # path for Maya files
outDir = '/Users/itz/Documents/Cambridge/PhD/Data_Chapter_02/Maya/ROM/results' # path for Maya output


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
from ROMmapperBatch import * # source the ligament functions

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

	arguments = (jointName, meshes, congruencyMeshes, fittedShape, gridSubdiv, [xBounds,yBounds,zBounds], interval, outDir)

	# initialise multiprocessing

	processes = [Process(target = processMayaFiles, args = (q,arguments)) for _ in range(CPUcores)]

	for process in processes:
		process.start()

	for process in processes:
		process.join()

