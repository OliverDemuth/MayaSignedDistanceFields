#   runLigamentCalculation.py
#
#   This script calculates and keys the length of a ligament from origin to insertion,
#   wrapping around the proximal and distal bone meshes, for each frame. The script can
#   be apported by pressing 'esc' and the already keyed frames will not be lost.
#
#   Written by Oliver Demuth
#   Last updated 29.05.2026 - Oliver Demuth
#
#
#   Note, for each ligament create a float attribute at 'jointName' and name it 
#   accordingly. Rename the strings in the user defined variables below according to the
#   objects in your Maya scene and make sure that the naming convention for the ligament 
#   origins and insertions is correct (i.e., the locators should be named 'ligament*_orig'
#	and 'ligament*_ins' for an attribute in 'jointName' called 'ligament*').
#   
#
#   This script relies on the following other (Python) script(s) which need to be run
#   in the Maya script editor before executing this script:
#
#       - 'ligamentCalculation.py'
#
#   For further information please check the Python script(s) referenced above


#################################################
# ========== user defined variables  ========== #
#################################################


jointName = 'myJoint'               # specify according to the joint centre in the Maya scene, i.e. the name of a locator or joint (e.g. 'myJoint' if following the ROM mapping protocol of Manafzadeh & Padian 2018)
meshes = ['prox_mesh', 'dist_mesh'] # specify according to meshes or boolean object in the Maya scene
gridSubdiv = 100                    # Integer value for the subdivision of the cube, i.e., number of grid points per axis (e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points)
gridScale = 1.5                     # Float value for the scale factor of the cubic grid (i.e., 1.5 initialises the grid from -1.5 to 1.5)
ligSubdiv = 20                      # Integer value for the number of ligament segments (e.g., 20, see Marai et al., 2004 for details)
StartFrame = None                   # Integer value to specify the start frame. If all frames are to be keyed from the beginning (Frame 1) set to standard value: None or 1.
FrameInterval = None                # Integer value to specify number of frames to be keyed. If all frames are to be keyed set to standard value: None
maxIter = 100					 	# Integer value specifying the maximum number of iterations for the SLSQP optimiser
keyPathPoints = False               # Boolean to specify whether ligament point positions are to be keyed or not. True = yes, False = no
debug = 0                           # Debug mode to check if signed distance fields have already been calculated



#################################################
# ==========    main script below    ========== #
#################################################


# ============= load modules =============


import maya.api.OpenMaya as om
import maya.api.OpenMayaAnim as oma
import maya.cmds as cmds
import numpy as np
import scipy as sp
import time


# ========================================

start = time.time()

var_exists = False

if debug == 1:

	# check if distance fields have already been calculated

	try:
		SDFs
	except NameError:
		var_exists = False
	else:
		var_exists = True # signed distance field already calculated, no need to do it again

if not var_exists:

	print('Calculating signed distance fields...')
	
	# get dag path for joint

	jNode, jDag = dagObjFromName(jointName)

	# reset joint

	eyeMat = om.MTransformationMatrix(om.MMatrix(np.eye(4)))
	om.MFnTransform(jDag).setTransformation(eyeMat)

	# calculate signed distance fields
	
	SDFs, ligAttributes, oDags, iDags, maxDist = sigDistField(jointName, meshes, gridSubdiv, gridScale)

part1 = time.time()

if not var_exists:
	print('Signed distance fields calculated in {0:.3f} seconds!'.format(part1 - start))
else:
	print('Signed distance fields succesfully loaded in {0:.3f} seconds!'.format(part1 - start))


# ==== extract joint transformations from keyframes ====


# get reference transformation matrices

jExclInv = jDag.exclusiveMatrix().inverse()
jInclInv = jDag.inclusiveMatrix().inverse()

# get local offsets of origins and insertions

oRelPos = np.array([np.array(orig.inclusiveMatrix() * jExclInv) for orig in oDags]).reshape(len(oDags),4,4)[:,3,:]
iRelPos = np.array([np.array(ins.inclusiveMatrix()  * jInclInv) for ins  in iDags]).reshape(len(oDags),4,4)[:,3,:]

# get total number of keyed frames from 'jointName', i.e., max number of frames to be calculated

attributes = ["translateX","translateY","translateZ","rotateX","rotateY","rotateZ"]
maxFrames = 0

for attr in attributes:
	attr_node = om.MSelectionList().add(f"{jointName}_{attr}").getDependNode(0)
	attr_curve = oma.MFnAnimCurve(attr_node)
	maxFrames = max(maxFrames,attr_curve.numKeys)

if FrameInterval and FrameInterval < maxFrames:
	frames = FrameInterval
else:
	frames = maxFrames

inclMat = np.empty((frames, 16), dtype=np.float64)
exclMat = np.empty((frames, 16), dtype=np.float64)

# cycle through frames and calculate matrix transformations

j_incl_plug = om.MFnDependencyNode(jNode).findPlug('worldMatrix', False).elementByPhysicalIndex(0)
j_excl_plug = om.MFnDependencyNode(jNode).findPlug('parentMatrix', False).elementByPhysicalIndex(0)

for frame in range(frames):

	# get frame

	context = om.MDGContext(om.MTime(frame + 1, 6)) # om.MTime.uiUnit() = 6

	# get joint transformation matrices

	inclMat[frame,:] = om.MFnMatrixData(j_incl_plug.asMObject(context)).matrix() # extract matrix without updating viewport
	
	# check if parent has incoming connections, otherwise use static

	jExclPlug = j_excl_plug.asMObject(context)

	# get parent transformation matrices

	if jExclPlug.isNull(): # check whether parent is stationary or dynamic (has incoming connections)
		exclMat[frame,:] = om.MFnMatrixData(j_excl_plug.asMObject()).matrix()
	else:
		exclMat[frame,:] = om.MFnMatrixData(jExclPlug).matrix()

# reshape arrays into Nx4x4 transformation matrix arrays

transMat = np.empty((frames,2,4,4))
transMat[:,0,:,:] = exclMat.reshape(frames,4,4)
transMat[:,1,:,:] = inclMat.reshape(frames,4,4)

# calculate world coordinates of joint and ligament attachments

jPos = transMat[:,1,3,0:3]
oPos = oRelPos @ transMat[:,0,:,:]
iPos = iRelPos @ transMat[:,1,:,:]

# get inverse of both parent and child rotation matrices

invTransMat = np.linalg.inv(transMat)

part2 = time.time()

print('Calculated {0} joint transformations in {1:.3f} seconds.'.format(frames, part2 - part1))


# ==== calculate ligament lengths ====


numPoints = ligSubdiv + 1

# define constant x coords

ligArr = np.stack([np.array([0.0,0.0,0.0,1.0])] * numPoints, axis = 0)
ligArr[:,0] = np.linspace(0.0, 1.0, num = numPoints, endpoint = True) # constant X coordinates

# maximal offset for path constraint

maxOffset = 3 / (ligSubdiv ** 2) # max squared mediolateral offset (i.e., arctan(offset/dist) ≤ 60° as tan(60°) = sqrt(3))

# define initual guess condition for optimiser

initial_guess = np.zeros(2 * numPoints)

# set bounds

bounds = [(-maxDist, maxDist) for _ in range(2 * numPoints)]
bounds[0] = bounds[1] = bounds[-2] = bounds[-1] = (0,0)
bounds = tuple(bounds)

# get ligament animation curves

lig_curves = [getAnimCurve(jointName, lig) for lig in ligAttributes]

# check if ligament points are to be keyed

if keyPathPoints:

	loc_curves = {}

	# create locators for ligament path points and create their animation curves

	for ligament in ligAttributes:

		# check if groups and their locators exist

		lig_GRP = ligament + '_LOC_GRP'

		if not cmds.objExists(lig_GRP): 
			cmds.group(em = True, name = lig_GRP) # create group if it doesn't exist already

		for k in range(numPoints):

			# get locator name

			loc = f'{ligament}_LOC_{k}'

			# check if locators exists

			if not cmds.objExists(loc):
				cmds.spaceLocator(name = loc) # create locator
				cmds.parent(loc, lig_GRP) # parent locator under their ligament locator group

			# get animation curves

			loc_curves[loc] = {'x': getAnimCurve(loc, 'translateX'),
							   'y': getAnimCurve(loc, 'translateY'),
							   'z': getAnimCurve(loc, 'translateZ')}

# get number of keyframes

if StartFrame is None:
	minKeys = 1
else: 
	minKeys = StartFrame

if FrameInterval is None or (minKeys + FrameInterval) > maxFrames:
	keyframes = maxFrames
	keyDiff = max(1, keyframes - minKeys + 1)
else:
	keyframes = minKeys + FrameInterval
	keyDiff = keyframes - minKeys

# define progress bar

cmds.progressWindow(title = 'Ligament calculation in progress...',
					progress = 1,
					status = 'Processing frame {0} of {1} frames'.format(1, frames),
					isInterruptable = True, 
					max = keyDiff)

print('Ligament calculation in progress...')

# go through each frame and key ligament lengths into attributes

for i in range(keyDiff):

	# check if progress is interupted

	if cmds.progressWindow(query = True, isCancelled = True):
		break

	ligRotMats, offsets = getLigTransMat(oPos[i,:], iPos[i,:], jPos[i,:])

	# calculate the length of each ligament 

	pathLengths, ligPoints, results = ligCalc(initial_guess, ligArr, SDFs[0], SDFs[1], invTransMat[i,:,:,:], ligRotMats, offsets, keyPathPoints, maxOffset, numPoints, bounds, maxIter)

	# get time for current frame

	mTime = om.MTime(minKeys + i, 6) # om.MTime.uiUnit() = 6

	for index, ligament in enumerate(ligAttributes):

		# key the attributes on the animated joint

		lig_curves[index].addKey(mTime, pathLengths[index]) # append key to ligament curves

		if debug == 1 and results[index].status != 0: # optimisation not successful, print info why not
			print(ligament, results[index])

		# check if ligament points are to be keyed

		if keyPathPoints:

			for k, ligpoint in enumerate(ligPoints[index]):

				# get locator name

				loc = f'{ligament}_LOC_{k}'

				# inject ligament point positions into locator animation curves

				loc_curves[loc]['x'].addKey(mTime, ligpoint[0]) # X coordinates of point k for ligament index
				loc_curves[loc]['y'].addKey(mTime, ligpoint[1]) # Y coordinates of point k for ligament index
				loc_curves[loc]['z'].addKey(mTime, ligpoint[2]) # Z coordinates of point k for ligament index

	# update progress bar and time

	cmds.progressWindow(edit = True, progress = i + 1, status = 'Processing frame {0} of {1} frames'.format(i + 1, keyDiff))

# when done close progress bar

oma.MAnimControl.setCurrentTime(mTime) # set current time to final time

end = time.time()

if cmds.progressWindow(query = True, isCancelled = True):
	print('# Abort: Ligament calculation cancelled after {0:.3f} seconds. Total frames keyed: {1}'.format(end - part2, i))
else:
	print('# Result: Ligament calculation done in {0:.3f} seconds! Successfully keyed {1} frames.'.format(end - part2, keyDiff))

cmds.progressWindow(edit = True, endProgress = True)

