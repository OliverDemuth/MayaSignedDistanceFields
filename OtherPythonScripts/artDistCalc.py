#	artDistCalc.py
#
#	This script calculates the articular disrance(s) within a joint capsule for one
#	or multiple joint orientations. The disal mesh is represented as a signed
#	distance field (SDF) to measure the relative distances of the proximal articular 
#	surface vertices. The overlap between the articular surfaces is defined as the
#	proportion of the articular distances that remain below the target threshold.
#	Target threshold is defined as 1/2 mean radius of the fitted sphere to the 
#	proximal articular surface.
#
#	Written by Oliver Demuth
#	Last updated 30.04.2025 - Oliver Demuth
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
#	    reduce polycount, prior to executing the Python script.
#	(3) This script requires several modules for Python (see README files). Make sure
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


jointName = 'myJoint' 				# specify according to the joint centre in the Maya scene (i.e. the name of a locator or joint; e.g. 'myJoint' if following the ROM mapping protocol of Manafzadeh & Padian 2018)
mesh = 'RLP3_humerus'				# specify according to meshes in the Maya scene (i.e., distal/reference mesh; e.g., humerus)
artSurfMesh = 'RLP3_glenoid_art_surf_mesh'	# specify according to meshes in the Maya scene (i.e., proximal articular surface mesh; e.g., glenoid articular surface)
referenceShape = "RLP3_glenoid_sphere_mesh"	# fitted shape to proximal articular surfaces
gridSubdiv = 100				# integer value for the subdivision of the cube (i.e., number of grid points per axis; e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points)
gridScale = 1 					# scale factor for the cubic grid dimensions
StartFrame = None 				# Integer value to specify the start frame. If all frames are to be keyed from the beginning (Frame 1) set to standard value: None or 1.
FrameInterval = None				# Integer value to specify number of frames to be keyed. If all frames are to be keyed set to standard value: None
debug = 0 					# Integer value to specify whether signed distance field calculations should be skipped (0 = false, 1 = true)




# ========== load modules ==========

import maya.api.OpenMaya as om
from maya.api.OpenMaya import MPoint, MTransformationMatrix, MSelectionList
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


# ========== get mean distance from center ==========

def meanRad(mesh):

	# Input variables:
	#	mesh = name of the mesh in scene
	# ======================================== #

	# get dag paths

	dag = dagObjFromName(mesh)[1]
	shape = dag.extendToShape()

	transMat = np.array(dag.inclusiveMatrix()).reshape(4,4)

	# get mesh object

	fnMesh = om.MFnMesh(shape)

	fnMeshMat = np.eye(4)
	fnMeshMat[3,:] = np.array(fnMesh.boundingBox.center)

	centerPos = np.dot(fnMeshMat,transMat)[3,:]

	# get world position of all vertices (om.MSpace.kWorld = 4)

	vertices = np.array(fnMesh.getPoints(4))

	# substract center position from vertices and return mean distance

	return np.linalg.norm((vertices - centerPos), axis = 1).mean() 


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

def sigDistMesh(mesh, rotMat, subdivision, gridScale):

	# Input variables:
	#	mesh = name of the mesh for which the signed distance field is calculated
	#	rotMat = transformation matrix of parent (joint) of the mesh
	#	subdivision = number of elements per axis, e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points
	# 	gridScale = scale factor for the cubic grid dimensions
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

	elements = np.linspace(-gridScale, gridScale, num = subdivision + 1, endpoint=True, dtype=float)
	
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

	# get sign from dot product for distance

	return dist * np.sign(dot)


# ========== get distances between articular surface and distal mesh ==========

def artDist(jointName, vtxArr, SDF, gridRotMat, gridSize, threshold):

	# Input variables:
	#	jointName = name of joint centre (represented by object, e.g. joint or locator, in Maya scene)
	#	vtxArr = 3D array of vertex transformation matrices for fast computation of relative coordinates 
	#	SDF = signed distance field in tricubic form
	#	gridRotMat = rotation matrix of default cubic grid
	#	gridSize = dimension of the cubic grid
	#	threshold = target distance
	# ======================================== #


	# get joint inclusive transformation matrix (child)

	jDag = dagObjFromName(jointName)[1]
	jInclTransMat = MTransformationMatrix(jDag.inclusiveMatrix()) # world transformation matrix of joint
	jInclTransMat.setScale([gridSize,gridSize,gridSize],4) # set scale in world space

	jExclTransMat = MTransformationMatrix(jDag.exclusiveMatrix()) # world transformation matrix of joint
	jExclTransMat.setScale([gridSize,gridSize,gridSize],4) # set scale in world space

	# get rotation matrices

	rotMatProx = np.array(jExclTransMat.asMatrix()).reshape(4,4) # parent rotMat (prox) as numpy 4x4 array
	rotMatDist = np.array(jInclTransMat.asMatrix()).reshape(4,4) # child rotMat (dist) as numpy 4x4 array

	# calculate position of vertices relative to cubic grid

	vtcRelArr = np.dot(vtxArr,np.dot(np.dot(rotMatProx,np.linalg.inv(rotMatDist)),gridRotMat))[:,3,0:3].tolist()

	# calculate distance 

	signDist = [SDF.ip(vtx) for vtx in vtcRelArr]

	return signDist, sum(signDist)/len(signDist), sum(signDist < threshold)/len(signDist) # signed distances, average distance and overlap


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

refRad = meanRad(referenceShape)
gridSize = refRad * 10 

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

	# get joint centre position

	jDag = dagObjFromName(jointName)[1]
	jInclTransMat = MTransformationMatrix(jDag.inclusiveMatrix()) # world transformation matrix of parent of joint
	jInclTransMat.setScale([gridSize,gridSize,gridSize],4) # set scale in world space (om.MSpace.kWorld = 4)

	jExclTransMat = MTransformationMatrix(jDag.exclusiveMatrix()) # world transformation matrix of parent of joint
	jExclTransMat.setScale([gridSize,gridSize,gridSize],4) # set scale in world space (om.MSpace.kWorld = 4)

	# only need to calculate one signed distance field (dist mesh)

	meshSigDist = sigDistMesh(mesh, np.array(jInclTransMat.asMatrix()).reshape(4,4), gridSubdiv, gridScale)
	sigDistArray = meshSigDist.reshape(gridSubdiv + 1, gridSubdiv + 1, gridSubdiv + 1)

	# calculate relative position of articular surfaces

	vtxArr = relVtcPos(artSurfMesh,np.array(jExclTransMat.asMatrix()).reshape(4,4))

# initialise tricubic interpolator with signed distance data on default cubic grid

SDF = tricubic(sigDistArray.tolist(), list(sigDistArray.shape)) # grid will be initialised in its relative coordinate system from [0,0,0] to [sigDistArray.shape[0], sigDistArray.shape[1], sigDistArray.shape[2]].

# get inverse of rotation matrix for default cubic grid coordinate system

gridVec = 2 * gridScale / gridSubdiv
gridRotMat = np.linalg.inv(np.array([[gridVec, 0, 0, 0], # x direction
				     [0, gridVec, 0, 0], # y direction
				     [0, 0, gridVec, 0], # z direction
				     [-gridScale, -gridScale, -gridScale, 1]])) # origin

mid = time.time()

if not var_exists:
	print('Signed distance fields calculated in {0:.3f} seconds!'.format(mid - start))
else:
	print('Signed distance fields succesfully loaded in {0:.3f} seconds!'.format(mid - start))

print('Calculating articular distances...')

# set current time

cmds.currentTime(minKeys)

threshold = refRad/2

avgDistArr = []
overlapArr = []

for i in range(keyDiff):

	signedDistances, avgerageDistance, overlap = artDist(jointName, vtxArr, SDF, gridRotMat, gridSize, threshold)

	avgDistArr.append(avgerageDistance)
	overlapArr.append(overlap)

	cmds.currentTime(i+1)

end = time.time()

# best fit so far is 1/2 of glenoid sphere radius, which gives an overlap (sensu Bishop et al. 2023) of just approximately 0.5 and is almost equal to average distance for CT scan positions
# alternative with 2/3 of ellipsoid mean raduis gets close with overlap of 0.46 and difference to average distance of -0.12

print('max average dist: ', max(avgDistArr))
print('min average dist: ', min(avgDistArr))
print('overlap: ',sum(overlapArr)/keyDiff)
print('average dist: ',sum(avgDistArr)/keyDiff)
print('threshold: ', threshold)
print('reference shape radius : ',meanRad(referenceShape))

print('Articular distances calculated in {0:.3f} seconds!'.format((end - mid)))

