#   ligamentCalculation.py
#
#   This script calculates the shortest distance of a ligament from origin to insertion 
#   wrapping around the bone meshes. It is an implementation of the Marai et al., 2004.
#   approach for Autodesk Maya. It creates signed distance fields for the proximal and
#   distal bone meshes, which are then used to approximate the 3D path of each ligament
#   accross the SDFs to calculate the ligaments' lengths.
#
#   Written by Oliver Demuth and Vittorio la Barbera
#   Last updated 17.02.2026 - Oliver Demuth
#
#   SYNOPSIS:
#
#       INPUT params:
#           string  jointName:      Name of the joint centre (i.e. the name of a locator or joint; e.g., 'myJoint' if following the ROM mapping protocol of Manafzadeh & Padian 2018)
#           string  meshes:         Name(s) of the bone meshes (e.g., several individual meshes in the form of ['prox_mesh','dist_mesh'])
#           int     gridSubdiv:     Integer value for the subdivision of the cube (i.e., number of grid points per axis; e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points)
#           float   gridScale:      Float value for the scale factor of the cubic grid (i.e., 1.5 initialises the grid from -1.5 to 1.5)
#           int     ligSubdiv:      Integer value for the number of ligament points (e.g., 20 will divide the ligament into 20 equidistant segments, see Marai et al., 2004 for details)
#
#       RETURN params:
#           list    pathLengths:    Return value is a list with the path lengths for all ligaments designated as custom attributes in the 'jointName'
#           list    pathPoints:     Return value is a list of lists with the 3D coordinates of the path points in world space for all ligaments designated as custom attributes in the 'jointName'
#           list    results:        Return value is a list of objects of the scipy.optimize.OptimizeResult class. They represent the outputs of the scipy.optimize.minimize() function
#
#
#   IMPORTANT notes:
#
#   (1) Meshes need realtively uniform face areas, otherwise large faces might skew 
#       vertex normals in their direction. It is, therefore, important to extrude 
#       edges around large faces prior to closing the hole at edges with otherwise 
#       acute angles to circumvent this issue (e.g., if meshes have been cut to reduce
#       polycount) prior to executing the Python scripts.
#
#   (2) For each ligament create a float attribute at 'jointName' and name it 
#       accordingly. Make sure that the naming convention for the ligament origins 
#       and insertions is correct, i.e. the locators should be named 'ligament*_orig' 
#       and 'ligament*_ins' for an attribute in 'jointName' called 'ligament*'.
#
#   (3) This script requires several modules for Python, see README file. Make sure to
#       have the following external modules installed for the mayapy application:
#
#           - 'numpy'   NumPy:          https://numpy.org/about/
#           - 'scipy'   SciPy:          https://scipy.org/about/
#
#       For further information regarding them, please check the website(s) referenced 
#       above.


# ========== load modules ==========

import numpy as np
import scipy as sp
import maya.cmds as cmds
import maya.api.OpenMaya as om


################################################
# ========== Maya specific functions ========= #
################################################


# ========== signed distance field per joint function ==========

def sigDistField(jointName, meshes, subdivision, gridScale):

	# Input variables:
	#   jointName = name of joint centre (represented by object in Maya scene; e.g. joint or locator)
	#   meshes = name(s) of the mesh(es) for which the signed distance field is calculated
	#   subdivision = number of elements per axis (e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points)
	#   gridScale = scale factor for the cubic grid dimensions
	# ======================================== #

	# get joint centre position

	jPos, jDag = getWSPos(jointName)

	# get number and names of ligaments

	LigAttributes = cmds.listAttr(jointName, ud = True) # get user defined attributes of 'jointName' (i.e. the float attributes that will contain the ligament lengths)

	# cycle through ligaments and extract their information

	distArr = []
	oDags = []
	iDags = []

	for ligament in LigAttributes:

		# get positions of points of interest

		oPos, oDag = getWSPos(ligament + '_orig')
		iPos, iDag = getWSPos(ligament + '_ins')

		# store dagPaths

		oDags.append(oDag)
		iDags.append(iDag)

		# get distances from origin and insertion to joint centre
				
		distArr.append((jPos - oPos).length()) # Euclidean distance from origin to joint centre (i.e., scale factor for grid point positions)
		distArr.append((jPos - iPos).length()) # Euclidean distance from insertion to joint centre (i.e., scale factor for grid point positions)

	maxDist = max(distArr) * gridScale # maximal distance from joint centre to ligament attachment

	# get rotation matrices

	rotMat = []
	rotMat.append(np.array(jDag.exclusiveMatrix()).reshape(4,4)) # parent rotMat (prox) as numpy 4x4 array
	rotMat.append(np.array(jDag.inclusiveMatrix()).reshape(4,4)) # child rotMat (dist) as numpy 4x4 array

	# cycle through all meshes

	if len(meshes) > 2:
		error('Too many meshes in array. Please specify only the distal and proximal bone mesh in the mesh array.')
	elif len(meshes) < 2:
		error('Too few meshes specified. Please specify TWO meshes in the mesh array.')
	
	SDFs = []
	for i,mesh in enumerate(meshes):
		meshSigDist, elements = sigDistMesh(mesh, rotMat[i], subdivision, maxDist) # get signed distance field for each mesh
		
		# initialise sp.interpolate.RegularGridInterpolator with signed distance data on default cubic grid

		SDFs.append(sp.interpolate.RegularGridInterpolator((elements, elements, elements), meshSigDist, method = 'cubic', bounds_error = False, fill_value = -1)) # grid will be initialised in its relative coordinate system from scaled [-size,-size,-size] to [size,size,size]
	
	return SDFs, LigAttributes, oDags, iDags, jDag, maxDist


# ========== signed distance field per mesh function ==========

def sigDistMesh(mesh, rotMat, subdivision, gridScale):

	# Input variables:
	#   mesh = name of the mesh for which the signed distance field is calculated
	#   rotMat = transformation matrix of parent (joint) of the mesh
	#   subdivision = number of elements per axis (e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points)
	#   gridScale = scale factor for the cubic grid dimensions
	# ======================================== #

	# get dag paths

	dag = dagObjFromName(mesh)[1]

	# create MObject

	shape = dag.extendToShape()
	mObj = shape.node()

	# get the meshs transformation matrix

	meshMat = dag.inclusiveMatrix()
	meshMatInv = np.array(meshMat.inverse()).reshape(4,4)

	# create the intersector

	polyIntersect = om.MMeshIntersector()
	polyIntersect.create(mObj,meshMat)
	get_closest_point = polyIntersect.getClosestPoint
	ptON = om.MPointOnMesh()

	# create 3D grid

	elements = np.linspace(-gridScale, gridScale, num = subdivision + 1, endpoint = True, dtype = float)
	X, Y, Z = np.meshgrid(elements, elements, elements, indexing = 'ij')
	points = np.vstack((X.ravel(), Y.ravel(), Z.ravel(), np.ones(X.size))).T 

	# calculate position of vertices relative to cubic grid

	gridWsPos = points @ rotMat
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

	# get the vector directions from gridPoints to points on mesh

	normDiff = diff / dist.reshape(-1,1) # direction of point relative to cubic grid

	# calculate dot product between the normal at ptON and vector to check if point is inside or outside of mesh

	dot = np.sum(N * normDiff, axis = 1)

	# get sign for distance from dot product

	signDist = dist * np.sign(dot)

	return signDist.reshape(subdivision + 1, subdivision + 1, subdivision + 1), elements # convert signed distance array into cubic grid format


# ========== get ligament transformation matrix function ==========

def getLigTransMat(oPos, iPos, jPos):

	# Input variables:
	#   oPos = origin position in global coordiate system in the form np.array([x, y, z, 1.0])
	#   iPos = insertion position in global coordiate system in the form np.array([x, y, z, 1.0])
	#   jPos = joint centre position in global coordiate system in the form np.array([x, y, z, 1.0])
	# ======================================== #
	
	# get array dimensions and extract xyz columns

	if oPos.ndim == 1:
		o_xyz = oPos[0:3] # shape (3,)
	else:
		o_xyz = oPos[:,0:3] # shape (N,3)

	if iPos.ndim == 1:
		i_xyz = iPos[0:3] # shape (3,)
	else:
		i_xyz = iPos[:,0:3] # shape (N,3)

	j_xyz = jPos[0:3] # shape (3,)

	# get axes directions

	x = i_xyz - o_xyz
	offset = np.linalg.norm(x, axis = 1, keepdims = True) # normalised
	x = x / offset # normalized

	v = j_xyz - o_xyz
	z = np.cross(v, x)
	z = z / np.linalg.norm(z, axis = 1, keepdims = True) # normalised

	y = np.cross(z, x)
	y = y / np.linalg.norm(y, axis = 1, keepdims = True) # normalised

	# assemble transformation matrices

	transMat = np.stack([np.eye(4)] * x.shape[0], axis = 0)
	transMat[:,0,0:3] = x * offset # scale by offset
	transMat[:,1,0:3] = y * offset # scale by offset
	transMat[:,2,0:3] = z * offset # scale by offset
	transMat[:,3,0:3] = o_xyz

	# remove unnecessary dimensions

	offset = offset.flatten()

	if offset.size == 1:
		transMat = transMat[0]
		offset = offset[0]

	return transMat, offset


# ========== get coordinates in world space function ==========

def getWSPos(name):

	# Input variables:
	#   name = string representing object name
	# ======================================== #
	
	dag = dagObjFromName(name)[1]
	
	return om.MTransformationMatrix(dag.inclusiveMatrix()).translation(4), dag # extract translation in world space from transformation matrix (3x faster than xform; om.MSpace.kWorld = 4)


# ========== dag path from name function ==========

def dagObjFromName(name):

	# Input variables:
	#   name = string representing object name
	# ======================================== #
	
	sel = om.MSelectionList()
	sel.add(name)

	return sel.getDependNode(0), sel.getDagPath(0)


################################################
# ============ optimiser functions =========== #
################################################


# ========== ligament length calculation function ==========

def ligCalc(initial_guess, ligArr, ipProx, ipDist, rotMat, ligTransforms, offset, keyPathPoints, maxOffset, numPoints, bounds):

	# Input variables:
	#   initial_guess = initual guess condition for optimiser
	#   ligArr = 2D array of ligament point coordinates of shape (N,4)
	#   ipProx = proximal signed distance field in form of scipy.interpolate.RegularGridInterpolator
	#   ipDist = distal signed distance field in form of scipy.interpolate.RegularGridInterpolator
	#   rotMat = array with the transformation matrices of the joint and its parent
	#   ligTransforms = array with transformation matrices representing ligament coordinate systems
	#   offset = array of euclidean distances from origin to insertion for each ligament
	#   keyPathPoints = boolean to specify whether ligament point positions are to be keyed or not. True = yes, False = no
	#   maxOffset = squared maximum mediolateral offset for path constraint function
	#   numPoints = number of ligament points
	#   bounds = bounds for optimisation
	# ======================================== #

	# create variables

	ligLengths = []
	ligPoints = []
	results = []

	for index, ligRotMat in enumerate(ligTransforms):

		# get ligament specific transformation matrix to cubic grid
		
		relRotMat = []
		relRotMat.append(ligRotMat @ rotMat[0])
		relRotMat.append(ligRotMat @ rotMat[1])

		# minimise ligament length through optimiser

		ligArrCopy = ligArr.copy() # copy initial array and only modify this copy
		res = ligLengthOptMin(initial_guess, ligArrCopy, ipProx, ipDist, relRotMat, maxOffset, numPoints, bounds)

		# correct relative ligament length by linear distance between origin and insertion to get actual ligament length

		if res.success: # check if optimiser terminated successfully
			ligLengths.append(res.fun * offset[index])
		else:
			ligLengths.append(-offset[index]) # optimiser was unsuccessful: mark as outlier (negative Euclidean distance between origin and insertion)

		# check if path points are to be calculate

		if keyPathPoints:
			
			# get world position of ligament points

			ligArrCopy[:,1:3] = res.x.reshape(numPoints, 2) # extract y and z coordinates from optimiser results
			ligPoints.append((ligArrCopy @ ligRotMat)[:,0:3].tolist()) # global coordinates of ligament points

		# gather results
		
		results.append(res)

	return ligLengths, ligPoints, results


# ========== ligament length optimiser function ==========

def ligLengthOptMin(initial_guess, ligArr, ipProx, ipDist, rotMat, maxOffset, numPoints, bounds):

	# Input variables:
	#   initial_guess = initual guess condition for optimiser
	#   ipProx = proximal signed distance field in form of scipy.interpolate.RegularGridInterpolator
	#   ipDist = distal signed distance field in form of scipy.interpolate.RegularGridInterpolator
	#   rotMat = array with the transformation matrices of the joint and its parent
	#   maxOffset = squared maximum mediolateral offset for path constraint function
	#   numPoints = number of ligament points
	#   bounds = bounds for optimisation
	# ======================================== #

	# precompute X‑column transform contributions

	xCoords = ligArr[:,0]
	x_prox = xCoords[1:-1, None] * rotMat[0][0,0:3] + rotMat[0][3,0:3] # skip first and last ligament coords (i.e., origin and insertion)
	x_dist = xCoords[1:-1, None] * rotMat[1][0,0:3] + rotMat[1][3,0:3] # skip first and last ligament coords (i.e., origin and insertion)

	# create tuple for arguments passed to both constraints and cost functions

	arguments = (ipProx, ipDist, rotMat, maxOffset, numPoints, x_prox, x_dist, np.empty((numPoints - 2) * 2), ligArr[:,0:3])

	# set constraints functions

	cons = ({'type': 'ineq',            # set type to inequality, which means that it is to be non-negative
			 'fun': sigDist_cons_fun,   # set constraint function
			 'args': arguments},        # set arguments
			{'type': 'ineq',            # set type to inequality, which means that it is to be non-negative
			 'fun': path_cons_fun,      # set constraint function
			 'args': arguments})        # set arguments

	# set options

	options = {"maxiter": 1000} # if it doesn't solve within 1000 iterations it usually won't solve

	# optimization using SLSQP and get results: res.x = ligament point coordinates, res.fun = relative ligament length

	return sp.optimize.minimize(cost_fun, initial_guess, args = arguments, bounds = bounds, method = 'SLSQP', constraints = cons, options = options)


# ========== signed distance field constraint function ==========

def sigDist_cons_fun(params, ipProx, ipDist, rotMat, maxOffset, numPoints, x_prox, x_dist, consArr, costArr): # ligament points cannot impinge bones

	# Input variables:
	#   params = array of Y and Z coordinates of ligament points (i.e., params = [(y_0, z_0),(y_1, z_1), ... ,(y_n-1, z_n-1)]). They are, however, flattened into a single array, i.e., [y0, z0, y1, z1, y2, z2, ... , y_n-1, z_n-1], and therefore need to be extracted.
	#   ipProx = proximal signed distance field in form of scipy.interpolate.RegularGridInterpolator
	#   ipDist = distal signed distance field in form of scipy.interpolate.RegularGridInterpolator
	#   rotMat = array with the transformation matrices of the joint and its parent
	#   maxOffset = squared maximum mediolateral offset for path constraint function. Passed through args, not part of this constraint function
	#   numPoints = number of ligament points
	#   x_prox = precomputed X‑column transform contributions in proximal rotation frame
	#   x_dist = precomputed X‑column transform contributions in distal rotation frame
	#   consArr = preallocated constraint array
	#   costArr = preallocated cost array. Passed through args, not part of constraint function
	# ======================================== #

	# extract y and z coordinates for matrix multiplications

	yz = params.reshape(numPoints, 2)[1:-1] # skip first and last ligament coords (i.e., origin and insertion)

	# get relative coordinates

	relPosProx = x_prox + yz @ rotMat[0][1:3,0:3] # get points in proximal cubic grid coordinates
	relPosDist = x_dist + yz @ rotMat[1][1:3,0:3] # get points in distal cubic grid coordinates

	# get signed distances

	n = numPoints - 2
	consArr[:n] = ipProx(relPosProx)
	consArr[n:] = ipDist(relPosDist)

	return consArr # if any of the ligament points are inside a mesh they will be negative and thus violate the constraint


# ========== path constraint function ==========

def path_cons_fun(params, ipProx, ipDist, rotMat, maxOffset, numPoints, x_prox, x_dist, consArr, costArr): # smoothness constraint

	# Input variables:
	#   params = array of Y and Z coordinates of ligament points (i.e., params = [(y_0, z_0),(y_1, z_1), ... ,(y_n-1, z_n-1)]). They are, however, flattened into a single array, i.e., [y0, z0, y1, z1, y2, z2, ... , y_n-1, z_n-1], and therefore need to be extracted.
	#   ipProx = proximal signed distance field in form of scipy.interpolate.RegularGridInterpolator. Passed through args, not part of this constraint function
	#   ipDist = distal signed distance field in form of scipy.interpolate.RegularGridInterpolator. Passed through args, not part of this constraint function
	#   rotMat = array with the transformation matrices of the joint and its parent. Passed through args, not part of this constraint function
	#   maxOffset = squared maximum mediolateral offset for path constraint function
	#   numPoints = number of ligament points
	#   x_prox = precomputed X‑column transform contributions in proximal rotation frame. Passed through args, not part of this constraint function
	#   x_dist = precomputed X‑column transform contributions in distal rotation frame. Passed through args, not part of this constraint function
	#   consArr = preallocated constraint array. Passed through args, not part of this constraint function
	#   costArr = preallocated cost array. Passed through args, not part of this constraint function
	# ======================================== #

	# view params as (numPoints,2) without copy

	yz = params.reshape(numPoints, 2)

	# calculate mediolatral offset between subsequent points

	diff_yz = yz[1:] - yz[:-1]

	return maxOffset - (diff_yz ** 2).sum(axis = 1) # squared mediolateral offset can be maximally 3 times the squared distance between path segment along X-axis (i.e., arctan(offset/dist) ≤ 60° as tan(60°) = sqrt(3))


# ========== cost function for optimisation ==========

def cost_fun(params, ipProx, ipDist, rotMat, maxOffset, numPoints, x_prox, x_dist, consArr, costArr): # minimise path length

	# Input variables:
	#   params = array of Y and Z coordinates of ligament points (i.e., params = [(y_0, z_0),(y_1, z_1), ... ,(y_n-1, z_n-1)]). They are, however, flattened into a single array, i.e., [y0, z0, y1, z1, y2, z2, ... , y_n-1, z_n-1], and therefore need to be extracted.
	#   ipProx = proximal signed distance field in form of scipy.interpolate.RegularGridInterpolator. Passed through args, not part of cost function
	#   ipDist = distal signed distance field in form of scipy.interpolate.RegularGridInterpolator. Passed through args, not part of cost function
	#   rotMat = array with the transformation matrices of the joint and its parent. Passed through args, not part of cost function
	#   maxOffset = maximum mediolateral offset for path constraint function. Passed through args, not part of cost function
	#   numPoints = number of ligament points
	#   x_prox = precomputed X‑column transform contributions in proximal rotation frame. Passed through args, not part of cost function
	#   x_dist = precomputed X‑column transform contributions in distal rotation frame. Passed through args, not part of cost function
	#   consArr = preallocated constraint array. Passed through args, not part of cost function
	#   costArr = preallocated cost array
	# ======================================== #

	# construct new array for cost function

	costArr[:,1:3] = params.reshape(numPoints, 2) 

	# difference between consecutive xyz points

	diff = costArr[1:] - costArr[:-1]

	return np.sqrt((diff ** 2).sum(axis = 1)).sum() # length of ligament from origin to insertion; y = params[0::2], z = params[1::2]


