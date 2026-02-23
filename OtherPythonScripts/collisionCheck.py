#	collisionCheck.py
#
#	This script checks for collision between two meshes with one of them ('proxMesh') 
#	represented by a signed distance field (SDF). This script is equivalent to the 
#	Boolean method first proposed by Manafzadeh & Padian (2018), however, it is 
#	several orders of magnitude faster. The runtime is >150 frames per second
#	on a target mesh ('distMesh') with ~5000 vertices.
#
#	Written by Oliver Demuth
#	Last updated 16.02.2026 - Oliver Demuth
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
#		- 'numpy' 	NumPy:		https://numpy.org/about/
#		- 'scipy' 	SciPy: 		https://scipy.org/about/
#
#	    For further information regarding them, please check the website(s) referenced 
#	    above.
#	(5) To execute this script copy and paste it into the Python Script Editor in Maya,
#	    adjust the user-defined variables below and hit run.


#################################################
# ========== user defined variables  ========== #
#################################################


jointName = 'myJoint' 	# Specify according to the joint centre in the Maya scene (i.e. the name of a locator or joint; e.g. 'myJoint' if following the ROM mapping protocol of Manafzadeh & Padian 2018)
proxMesh = 'prox_mesh'	# Specify according to the proximal mesh in the Maya scene (i.e., parented under the parent of 'jointName'/hierarchically on the same layer as 'jointName'; e.g., 'myJoint' if following Manafzadeh & Padian 2018)
distMesh = 'dist_mesh'	# Specify according to the distal mesh in the Maya scene (i.e., parented under 'jointName'; e.g., 'myJoint' if following Manafzadeh & Padian 2018)
gridSubdiv = 100		# Integer value for the subdivision of the cube (i.e., number of grid points per axis; e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points)
gridSize = 20			# Dimensions of cubic grid (in the Maya working units) which will be initialised from -gridSize to gridSize (e.g., 10 will result in a cubic grid with an edge length of 20 from -10 to 10). Set sufficiently large to make sure that all vertices of the target mesh are within the grid for all joint orientations
StartFrame = None 		# Integer value to specify the start frame. If all frames are to be keyed from the beginning (Frame 1) set to standard value: None or 1.
FrameInterval = None	# Integer value to specify number of frames to be keyed. If all frames are to be keyed set to standard value: None
debug = 0 				# Integer value to specify whether signed distance field calculations should be skipped (0 = false, 1 = true)


# ========== load modules ==========

import maya.api.OpenMaya as om
import maya.cmds as cmds
import numpy as np
import scipy as sp
import time


#################################################
# ==========     functions below     ========== #
#################################################


# ========== get dag path function ==========

def dagObjFromName(name):

	# Input variables:
	#	name = string representing object name
	# ======================================== #

	sel = om.MSelectionList()
	sel.add(name)

	return sel.getDependNode(0), sel.getDagPath(0)


# ========== signed distance field per mesh function ==========

def sigDistMesh(mesh, rotMat, subdivision, scale):

	# Input variables:
	#	mesh = name of the mesh for which the signed distance field is calculated
	#	rotMat = transformation matrix of parent (joint) of the mesh
	#	subdivision = number of elements per axis (e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points)
	#	scale = scale factor for the cubic grid dimensions
	# ======================================== #

	# get dag paths

	meshDag = dagObjFromName(mesh)[1]

	# create MObject

	shape = meshDag.extendToShape()
	mObj = shape.node()

	# get the meshs transformation matrix

	meshMat = meshDag.inclusiveMatrix()
	meshMatInv = np.array(meshDag.inclusiveMatrix().inverse()).reshape(4,4)

	# create the intersector

	polyIntersect = om.MMeshIntersector()
	polyIntersect.create(mObj,meshMat)
	get_closest_point = polyIntersect.getClosestPoint # extract function from intersector
	ptON = om.MPointOnMesh()

	# create 3D grid

	elements = np.linspace(-scale, scale, num = subdivision + 1, endpoint = True, dtype = float)
	X, Y, Z = np.meshgrid(elements, elements, elements, indexing = 'ij')
	points = np.vstack((X.ravel(), Y.ravel(), Z.ravel(), np.ones(X.size))).T 

	# calculate position of vertices relative to cubic grid

	gridWsPos = (points @ rotMat)
	gridWSArr = gridWsPos[:,0:3]

	# go through grid points and calculate signed distance for each of them

	P = np.zeros(points.shape)
	N = np.zeros((X.size,3))
	ptRel = om.MPoint()

	for i, gridPoint in enumerate(gridWSArr):
		ptRel.x, ptRel.y, ptRel.z = gridPoint # extract coordinates from gridPoint and feed into preallocated MPoint
		ptON = get_closest_point(ptRel) # get point on mesh
		P[i,:] = ptON.point # point on mesh coordinates in mesh coordinate system
		N[i,:] = ptON.normal # normal at point on mesh

	# get vectors from gridPoints to their closest points on mesh

	diff = (gridWsPos @ meshMatInv)[:,0:3] - P[:,0:3]

	# get length of vectors (distance)

	dist = np.linalg.norm(diff, axis = 1)

	# get the vectors' direction from gridPoints to points on mesh

	normDiff = diff / dist.reshape(-1,1) # direction of point relative to cubic grid

	# calculate dot product between the normal at ptON and vector to check if point is inside or outside of mesh

	dot = np.sum(N * normDiff, axis = 1)

	# get sign for distance from dot product

	sigDist = (dist * np.sign(dot)).reshape(subdivision + 1, subdivision + 1, subdivision + 1)

	# convert signed distance array into cubic grid format

	return sp.interpolate.RegularGridInterpolator((elements, elements, elements), sigDist, method = 'cubic', bounds_error = False,  fill_value = -1) # grid will be initialised in its relative coordinate system from scaled [-size,-size,-size] to [size,size,size]


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
		SDF
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

	# initialise sp.interpolate.RegularGridInterpolator with signed distance data on default cubic grid for one signed distance field (i.e., for 'proxMesh')

	SDF = sigDistMesh(proxMesh, np.array(jDag.exclusiveMatrix()).reshape(4,4), gridSubdiv, gridSize)  # world transformation matrix of parent of joint

	# calculate relative position of articular surfaces

	mesh = dagObjFromName(distMesh)[1]
	vertices = np.array(om.MFnMesh(mesh).getPoints(4)) # world space coordinates of vertices
	vtxArr = vertices @ np.array(jDag.inclusiveMatrix().inverse()).reshape(4,4) # homogenous vertex coordinates relative to joint coordinate system

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

invMat = np.array(jDag.exclusiveMatrix().inverse()).reshape(4,4) # inverse of parent rotMat (prox) as numpy 4x4 array

for i in range(keyDiff):

	rotMat = np.array(jDag.inclusiveMatrix()).reshape(4,4)
	signDist = SDF((vtxArr @ (rotMat @ invMat))[:,0:3])

	# key viable attribute at joint

	if signDist[signDist != -1].min() > 0:
		cmds.setKeyframe(jointName, at = 'viable', v = 1)
		counter += 1
	else:
		cmds.setKeyframe(jointName, at = 'viable', v = 0)

	cmds.currentTime(i + minKeys + 1)

	# update progress bar

	cmds.progressWindow(edit = True, progress = i + 1, status = 'Processing frame {0} of {1} frames'.format(i + 1, keyDiff))

# when done close progress bar

end = time.time()

if cmds.progressWindow(query = True, isCancelled = True):
	print('# Abort: Mesh intersection check cancelled after {0:.3f} seconds. Total {1} frames tested and keyed {2} viable frames'.format(end - mid,i + 1, counter))
else:
	print('# Result: Mesh intersection check completed in {0:.3f} seconds! Successfully tested {1} frames and keyed {2} viable frames.'.format(end - mid, keyDiff, counter))

cmds.progressWindow(edit = True, endProgress = True)
