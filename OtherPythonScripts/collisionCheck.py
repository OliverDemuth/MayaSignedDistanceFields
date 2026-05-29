#	collisionCheck.py
#
#	This script checks for collision between two meshes with one of them ('proxMesh') 
#	represented by a signed distance field (SDF). This script is equivalent to the 
#	Boolean method first proposed by Manafzadeh & Padian (2018), however, it is 
#	several orders of magnitude faster. The runtime is ~500 frames per second
#	on a target mesh ('distMesh') with ~5000 vertices. The script automatically
#	creates a 'viable' attribute that is keyed throughout.
#
#	Written by Oliver Demuth
#	Last updated 29.05.2026 - Oliver Demuth
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
#	(3) This script requires several modules for Python (see README files). Make sure
#	    to have the following external modules installed for the mayapy application:
#
#		- 'numpy' 	NumPy:		https://numpy.org/about/
#		- 'scipy' 	SciPy: 		https://scipy.org/about/
#
#	    For further information regarding them, please check the website(s) referenced 
#	    above.
#	(4) To execute this script copy and paste it into the Python Script Editor in Maya,
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
import maya.api.OpenMayaAnim as oma
import maya.cmds as cmds
import numpy as np
import scipy as sp
import time


#################################################
# ==========     functions below     ========== #
#################################################


# ========== signed distance field per mesh function ==========

def sigDistMesh(mesh, rotMat, subdivision, scale):

	# Input variables:
	#	mesh = name of the mesh for which the signed distance field is calculated
	#	rotMat = transformation matrix of parent (joint) of the mesh
	#	subdivision = number of elements per axis (e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points)
	#	scale = scale factor for the cubic grid dimensions
	# ======================================== #

	# get dag paths

	meshDag = om.MGlobal.getSelectionListByName(mesh).getDagPath(0)

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

	gridWsPos = points @ rotMat
	gridWSArr = gridWsPos[:,0:3]

	# go through grid points and calculate signed distance for each of them

	P = np.zeros(points.shape)
	N = np.zeros((X.size,3))
	ptRel = om.MPoint()

	# set up progress bar

	cmds.progressWindow(title = 'Calculating signed distance field...',
						progress = 1,
						status = 'Processing point {0} of {1} points'.format(1,X.size),
						isInterruptable = True,
						max = X.size)

	for i, gridPoint in enumerate(gridWSArr):
		ptRel.x, ptRel.y, ptRel.z = gridPoint # extract coordinates from gridPoint and feed into preallocated MPoint
		ptON = get_closest_point(ptRel) # get point on mesh
		P[i,:] = ptON.point # point on mesh coordinates in mesh coordinate system
		N[i,:] = ptON.normal # normal at point on mesh

		# log progress

		cmds.progressWindow(edit = True, progress = i + 1, status = 'Processing frame {0} of {1} frames'.format(i + 1, X.size))

	# close progress window when done

	cmds.progressWindow(edit = True, endProgress = True)

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

if StartFrame == None:
	minFrames = 1
else: 
	minFrames = StartFrame

# get joint dag path and dedependency node

j_sel = om.MGlobal.getSelectionListByName(jointName)
j_node = om.MFnDependencyNode(j_sel.getDependNode(0))
j_dag = j_sel.getDagPath(0)

if not j_node.hasAttribute('viable'):
	viable_attr = om.MFnNumericAttribute()
	viable_obj = viable_attr.create('viable', 'viable', om.MFnNumericData.kInt, 0)
	viable_attr.keyable = True
	j_node.addAttribute(viable_obj)
	
# check if 'viable' has keys and strip them if so

viable_plug = j_node.findPlug('viable', False)

if viable_plug.isDestination:
	source =  viable_plug.source()
	anim_node = source.node()
	
	# delete the node using MDGModifier
	
	if anim_node.hasFn(om.MFn.kAnimCurve):
		dg_mod = om.MDGModifier()
		dg_mod.deleteNode(anim_node)
		dg_mod.doIt()

# get total number of keyed frames from 'jointName'

attributes = ["translateX","translateY","translateZ","rotateX","rotateY","rotateZ"]
maxFrames = 0

for attr in attributes:
	attr_node = om.MSelectionList().add(f"{jointName}_{attr}").getDependNode(0)
	attr_curve = oma.MFnAnimCurve(attr_node)
	maxFrames = max(maxFrames,attr_curve.numKeys)

# set frame interval to be tested

if FrameInterval is None or (minFrames + FrameInterval) > maxFrames:
	keyframes = maxFrames
	frames = keyframes - minFrames + 1
else:
	keyframes = minFrames + FrameInterval
	frames = keyframes - minFrames

if frames <= 0:
	frames = 1

start = time.time()

# calculate signed distance fields

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

	oma.MAnimControl.setCurrentTime(om.MTime(0)) # set fast time

	# reset transformations

	eyeMat = om.MTransformationMatrix(om.MMatrix(np.eye(4)))
	om.MFnTransform(j_dag).setTransformation(eyeMat)

	# initialise sp.interpolate.RegularGridInterpolator with signed distance data on default cubic grid for one signed distance field (i.e., for 'proxMesh')

	SDF = sigDistMesh(proxMesh, np.array(j_dag.exclusiveMatrix()).reshape(4,4), gridSubdiv, gridSize)  # world transformation matrix of parent of joint

	# calculate relative position of articular surfaces

	mesh = om.MGlobal.getSelectionListByName(distMesh).getDagPath(0)
	vertices = np.array(om.MFnMesh(mesh).getPoints(4)) # world space coordinates of vertices
	vtxArr = vertices @ np.array(j_dag.inclusiveMatrix().inverse()).reshape(4,4) # homogenous vertex coordinates relative to joint coordinate system

part1 = time.time()

if not var_exists:
	print('Signed distance fields calculated in {0:.3f} seconds!'.format(part1 - start))
else:
	print('Signed distance fields succesfully loaded in {0:.3f} seconds!'.format(part1 - start))

# define progress bar

cmds.progressWindow(title = 'Checking for mesh intersections...',
			progress = 1,
			status = 'Processing frame {0} of {1} frames'.format(1,frames),
			isInterruptable = True,
			max = frames)

print('Checking mesh intersections...')

# create empty animation curve

viable_curve = oma.MFnAnimCurve()
viable_curve.create(viable_plug)

# extract keyframes

keyedArr = np.empty((frames,len(attributes)))

for idx, attr in enumerate(attributes):
	attr_node = om.MSelectionList().add(f"{jointName}_{attr}").getDependNode(0)
	attr_curve = oma.MFnAnimCurve(attr_node)
	keyedArr[:, idx] = [attr_curve.value(frame) for frame in range(frames)]

# convert keyframes to transformation matrices

localMat = np.stack([np.eye(4)] * frames, axis = 0)	
localMat[:,3,0:3] = keyedArr[:,0:3]
localMat[:,0:3,0:3] = sp.spatial.transform.Rotation.from_euler('ZYX', keyedArr[:,3:6], degrees = False).as_matrix()[:,::-1,::-1]

part2 = time.time()

print('# Calculated {0} joint transformations in {1:.3f} seconds.'.format(frames, part2 - part1))

# setup progress bar

cmds.progressWindow(title = 'Checking for mesh intersections...',
			progress = 1,
			status = 'Processing frame {0} of {1} frames'.format(1, frames),
			isInterruptable = True,
			max = frames)

# set counter for viable poses

counter = 0

# calculate signed distances and key them

for frame in range(frames):
	
	signDist = SDF((vtxArr @ localMat[i,:,:])[:,0:3])

	# key viable attribute at joint

	if signDist[signDist != -1].min() > 0:
		viable_curve.addKey(om.MTime(i), 1)
		counter += 1
	else:
		viable_curve.addKey(om.MTime(i), 0)

	# log progress

	cmds.progressWindow(edit = True, progress = frame + 1, status = 'Processing frame {0} of {1} frames'.format(frame + 1, frames))

# shut down progress bar

cmds.progressWindow(edit = True, endProgress = True)

end = time.time()

if cmds.progressWindow(query = True, isCancelled = True):
	print('# Abort: Mesh intersection check cancelled after {0:.3f} seconds. Total {1} frames tested and keyed {2} viable frames'.format(end - part2,i + 1, counter))
else:
	print('# Result: Mesh intersection check completed in {0:.3f} seconds! Successfully tested {1} frames and keyed {2} viable frames.'.format(end - part2, frames, counter))

cmds.progressWindow(edit = True, endProgress = True)


