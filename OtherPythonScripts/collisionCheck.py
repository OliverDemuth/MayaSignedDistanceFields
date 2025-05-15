#	collisionCheck.py
#
#	This script checks for collision between two meshes with one of them ('proxMesh') 
#	represented by a signed distance field (SDF). This script is equivalent to the 
#	Boolean method first proposed by Manafzadeh & Padian (2018), however, it is 
#	several orders of magnitude faster. The runtime is <0.01 seconds per frame based 
#	on a target mesh ('distMesh') with ~5000 vertices.
#
#	Written by Oliver Demuth
#	Last updated 15.05.2025 - Oliver Demuth
#
#
#	IMPORTANT notes:
#		
#	(1) Rename the strings in the user-defined variables below according to the 
#	    objects in your Maya scene and set the other attributes accordingly.
#	(2) Meshes need realtively uniform face areas, otherwise large faces might skew 
#	    vertex normals in their direction. It is, therefore, important to extrude 
#	    edges around large faces prior to closing the hole at edges with otherwise 
#	    acute angles to circumvent this issue, e.g., if meshes have been cut to 
#	    reduce polycount, prior to executing the Python script. If the proximal mesh
#	    (i.e., 'proxMesh' below), is a convex hull it first needs to be remeshed and 
#	    retopologised (using the standard settings in Maya) otherwise the sign 
#	    determination using the surface normals might be inaccurate.
#	(3) Make sure to have a Boolean attribute called 'viable' at your animated joint 
#	    (e.g., 'myJoint' if following the ROM mapping protocol of Manafzadeh & Padian
#	    2018).
#	(4) This script requires several modules for Python (see README files). Make sure
#	    to have the following external modules installed for the mayapy application:
#
#		- 'numpy' 	NumPy:			https://numpy.org/about/
#		- 'tricubic' 	Daniel Guterding: 	https://github.com/danielguterding/pytricubic
#
#	    For further information regarding them, please check the website(s) referenced 
#	    above.


#################################################
# ========== user defined variables  ========== #
#################################################


jointName = 'myJoint' 		# Specify according to the joint centre in the Maya scene (i.e. the name of a locator or joint; e.g. 'myJoint' if following the ROM mapping protocol of Manafzadeh & Padian 2018)
proxMesh = 'RLP3_conv_hull'	# Specify according to the proximal mesh in the Maya scene (i.e., parented under the parent of 'jointName'/hierarchically on the same layer as 'jointName'; e.g., 'myJoint' if following Manafzadeh & Padian 2018)
distMesh = 'RLP3_humerus'	# Specify according to the distal mesh in the Maya scene (i.e., parented under 'jointName'; e.g., 'myJoint' if following Manafzadeh & Padian 2018)
gridSubdiv = 100		# Integer value for the subdivision of the cube (i.e., number of grid points per axis; e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points)
gridSize = 50			# Dimensions of cubic grid (in the Maya working units) which will be initialised from -gridSize to gridSize (e.g., 10 will result in a cubic grid with an edge length of 20 from -10 to 10). Set sufficiently large to make sure that all vertices of the target mesh are within the grid for all joint orientations
StartFrame = None 		# Integer value to specify the start frame. If all frames are to be keyed from the beginning (Frame 1) set to standard value: None or 1.
FrameInterval = None		# Integer value to specify number of frames to be keyed. If all frames are to be keyed set to standard value: None
debug = 0 			# Integer value to specify whether signed distance field calculations should be skipped (0 = false, 1 = true)


# ========== load modules ==========

import maya.api.OpenMaya as om
from maya.api.OpenMaya import MPoint, MTransformationMatrix, MSelectionList
import maya.cmds as cmds
import numpy as np
import time

from tricubic import tricubic


#################################################
# ==========     functions below     ========== #
#################################################


# ========== get dag path function ==========

def dagObjFromName(name):

	# Input variables:
	#	name = string representing object name
	# ======================================== #
	
	sel = MSelectionList()
	sel.add(name)

	return sel.getDependNode(0), sel.getDagPath(0)


# ========== get vertices position in reference frame ==========

def relVtcPos(mesh, rotMat):

	# Input variables:
	#	mesh = name of the mesh for which the signed distance field is calculated
	#	rotMat = transformation matrix of parent (joint) of the mesh
	# ======================================== #

	# get dag paths

	dag = dagObjFromName(mesh)[1]

	# get vertices in global coordinate system

	vertices = np.array(om.MFnMesh(dag).getPoints(4)) # get world position of all vertices (om.MSpace.kWorld = 4)

	# get coordinates and transform them into matrices

	vtxArr = np.stack([np.eye(4)] * vertices.shape[0], axis = 0)
	vtxArr[:,3,:] = vertices # append coordinates to rotation matrix array

	return np.dot(vtxArr,np.linalg.inv(rotMat)) # calculate new coordinates and return 3D array


# ========== signed distance field per mesh function ==========

def sigDistMesh(mesh, rotMat, subdivision):

	# Input variables:
	#	mesh = name of the mesh for which the signed distance field is calculated
	#	rotMat = transformation matrix of parent (joint) of the mesh
	#	subdivision = number of elements per axis, e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points
	# ======================================== #

	# get dag paths

	dag = dagObjFromName(mesh)[1]

	# create MObject

	shape = dag.extendToShape()
	mObj = shape.node()
	
	# get the meshs transformation matrix

	meshMat = dag.inclusiveMatrix()
	meshMatInv = np.linalg.inv(np.array(meshMat).reshape(4,4)) # get inverse of mesh matrix as numpy 4x4 array

	# create the intersector

	polyIntersect = om.MMeshIntersector()
	polyIntersect.create(mObj,meshMat)
	ptON = om.MPointOnMesh()

	# create 3D grid

	elements = np.linspace(-1, 1, num = subdivision + 1, endpoint=True, dtype=float)
	
	points = np.array([[x, y, z] for x in elements
				     for y in elements 
				     for z in elements])

	# get coordinates and transform them into matrices

	gridArr = np.stack([np.eye(4)] * points.shape[0], axis = 0)
	gridArr[:,3,0:3] = points # append coordinates to rotation matrix array

	# calculate position of vertices relative to cubic grid

	gridWSArr = np.dot(gridArr,rotMat)
	gridWSList = gridWSArr[:,3,0:3].tolist()

	# go through grid points and calculate signed distance for each of them

	P = np.zeros((points.shape[0],3))
	N = np.zeros((points.shape[0],3))

	for i, gridPoint in enumerate(gridWSList):
		ptON = polyIntersect.getClosestPoint(MPoint(gridPoint)) # get point on mesh
		P[i,:] = [ptON.point.x, ptON.point.y, ptON.point.z] # coordinates of point on mesh in mesh coordinate system
		N[i,:] = [ptON.normal.x, ptON.normal.y, ptON.normal.z] # surface normal at point on mesh

	# get vectors from gridPoints to points on mesh

	diff = np.dot(gridWSArr,meshMatInv)[:,3,0:3] - P

	# get distances from gridPoints to points on mesh

	dist = np.linalg.norm(diff, axis = 1)

	# normalise vector to get direction from gridPoints to points on mesh

	normDiff = diff/dist.reshape(-1,1)

	# calculate dot product between the normal at ptON and vector to check if point is inside or outside of mesh

	dot = np.sum(N * normDiff, axis = 1)

	# get sign for distance from dot product

	sigDist = dist * np.sign(dot)

	return sigDist.reshape(subdivision + 1, subdivision + 1, subdivision + 1) # convert signed distance array into cubic grid format



# ========== get distances between articular surface and distal mesh ==========

def artDist(jointName, vtxArr, SDF, gridRotMat, gridSize):

	# Input variables:
	#	jointName = name of joint centre (represented by object, e.g. joint or locator, in Maya scene)
	#	vtxArr = 3D array of vertex transformation matrices for fast computation of relative coordinates 
	#	SDF = signed distance field in tricubic form
	#	gridRotMat = rotation matrix of default cubic grid
	#	gridSize = dimension of the cubic grid
	# ======================================== #


	# get rotation matrices of joint

	jDag = dagObjFromName(jointName)[1]
	jInclTransMat = MTransformationMatrix(jDag.inclusiveMatrix()) # world transformation matrix of joint
	jExclTransMat = MTransformationMatrix(jDag.exclusiveMatrix()) # world transformation matrix of parent of joint

	# normalize matrices by size

	jInclTransMat.setScale([gridSize,gridSize,gridSize],4) # set scale in world space (om.MSpace.kWorld = 4)
	jExclTransMat.setScale([gridSize,gridSize,gridSize],4) # set scale in world space (om.MSpace.kWorld = 4)

	# get rotation matrices

	rotMatProx = np.array(jExclTransMat.asMatrix()).reshape(4,4) # parent rotMat (prox) as numpy 4x4 array
	rotMatDist = np.array(jInclTransMat.asMatrix()).reshape(4,4) # child rotMat (dist) as numpy 4x4 array

	# calculate position of vertices relative to cubic grid

	vtcRelArr = np.dot(vtxArr,np.dot(np.dot(rotMatDist,np.linalg.inv(rotMatProx)),gridRotMat))[:,3,0:3].tolist()

	# calculate distances 

	return [SDF.ip(vtx) for vtx in vtcRelArr]


#################################################
# ==========    main script below    ========== #
#################################################


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
		sigDistArray
	except NameError:
		var_exists = False
	else:
		var_exists = True # signed distance field already calculated, no need to do it again

else:
	var_exists = False

if not var_exists:
	# calculate signed distance fields

	print('Calculating signed distance field...')

	cmds.currentTime(0)
	cmds.move(0,0,0, jointName, localSpace=True)
	cmds.rotate(0,0,0,jointName)

	# get rotation matrices of joint

	jDag = dagObjFromName(jointName)[1]
	jInclTransMat = MTransformationMatrix(jDag.inclusiveMatrix()) # world transformation matrix of joint
	jExclTransMat = MTransformationMatrix(jDag.exclusiveMatrix()) # world transformation matrix of parent of joint

	# normalize matrices by size

	jInclTransMat.setScale([gridSize,gridSize,gridSize],4) # set scale in world space (om.MSpace.kWorld = 4)
	jExclTransMat.setScale([gridSize,gridSize,gridSize],4) # set scale in world space (om.MSpace.kWorld = 4)

	# only need to calculate one signed distance field (i.e., for 'proxMesh')

	sigDistArray = sigDistMesh(proxMesh, np.array(jExclTransMat.asMatrix()).reshape(4,4), gridSubdiv)

	# calculate relative position of articular surfaces

	vtxArr = relVtcPos(distMesh,np.array(jInclTransMat.asMatrix()).reshape(4,4))

# initialise tricubic interpolator with signed distance data on default cubic grid

SDF = tricubic(sigDistArray.tolist(), list(sigDistArray.shape)) # grid will be initialised in its relative coordinate system from [0,0,0] to [sigDistArray.shape[0], sigDistArray.shape[1], sigDistArray.shape[2]].

# get inverse of rotation matrix for default cubic grid coordinate system

gridVec = 2 / gridSubdiv
gridRotMat = np.linalg.inv(np.array([[gridVec, 0, 0, 0], # x direction
				     [0, gridVec, 0, 0], # y direction
				     [0, 0, gridVec, 0], # z direction
				     [-1, -1, -1, 1]])) # origin

mid = time.time()

if not var_exists:
	print('Signed distance fields calculated in {0:.3f} seconds!'.format(mid - start))
else:
	print('Signed distance fields succesfully loaded in {0:.3f} seconds!'.format(mid - start))

# define progress bar

cmds.progressWindow(title = 'Checking for mesh intersections...',
		    progress = 1,
		    status = 'Processing frame {0} of {1} frames'.format(1,keyDiff),
		    isInterruptable = True,
		    max = keyDiff)

print('Checking mesh intersections...')

# set current time

cmds.currentTime(minKeys)

counter = 0

for i in range(keyDiff):

	signDist = artDist(jointName, vtxArr, SDF, gridRotMat, gridSize)

	# key viable attribute at joint

	if all(dist > 0 for dist in signDist):
		cmds.setKeyframe(jointName, at = 'viable', v = 1)
		counter += 1
	else:
		cmds.setKeyframe(jointName, at = 'viable', v = 0)

	cmds.currentTime(i+minKeys+1)

	# update progress bar

	cmds.progressWindow(edit=True, progress=i+1, status='Processing frame {0} of {1} frames'.format(i+1,keyDiff))

# when done close progress bar

end = time.time()

if cmds.progressWindow(query=True, isCancelled=True):
	print('# Abort: Mesh intersection check cancelled after {0:.3f} seconds. Total {1} frames tested and keyed {2} viable frames'.format(end - mid,i+1,int(counter)))
else:
	print('# Result: Mesh intersection check completed in {0:.3f} seconds! Successfully tested {1} frames and keyed {2} viable frames.'.format(end - mid,keyDiff,int(counter)))

cmds.progressWindow(edit=True, endProgress=True)

