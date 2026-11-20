#   ligamentCalculation.py
#
#   This script calculates the shortest distance of a ligament from origin to insertion 
#   wrapping around the bone meshes. It is an implementation of the Marai et al., 2004.
#   approach for Autodesk Maya. It creates signed distance fields for the proximal and
#   distal bone meshes, which are then used to approximate the 3D path of each ligament
#   accross the SDFs to calculate the ligaments' lengths.
#
#   Written by Oliver Demuth and Vittorio la Barbera
#   Last updated 20.11.2026 - Oliver Demuth
#
#   SYNOPSIS:
#
#       INPUT params:
#			string  jointName:		Name of the joint centre, i.e. the name of a locator or joint (e.g., 'myJoint' if following the ROM mapping protocol of Manafzadeh & Padian 2018)
#			string  meshes:			Name(s) of the bone meshes (e.g., several individual meshes in the form of ['prox_mesh','dist_mesh'])
#			int		gridSubdiv:		Integer value for the subdivision of the cube, i.e., number of grid points per axis (e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points)
#			float	gridScale:		Float value for the scale factor of the cubic grid (i.e., 1.5 initialises the grid from -1.5 to 1.5)
#			int 	ligSubdiv:		Integer value for the number of ligament points (e.g., 20 will divide the ligament into 20 equidistant segments, see Marai et al., 2004 for details)
#			int 	frameInterval:	Integer value to specify number of frames to be tested. If all frames are to be tested set to standard value: None
#			int 	maxIter:		Integer value specifying the maximum number of iterations for the SLSQP optimiser
#			string 	outDir:			Output directory
#
#       RETURN params:
#           list    pathLengths:    Return value is a list with the path lengths for all ligaments designated as custom attributes in the 'jointName'
#           list    pathPoints:     Return value is a list of lists with the 3D coordinates of the path points in world space for all ligaments designated as custom attributes in the 'jointName'
#           list    results:        Return value is a list of objects of the custom res class, similar to the previous scipy.optimize.OptimizeResult class
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
#			- 'numpy'	NumPy:			https://numpy.org/about/
#			- 'scipy'	SciPy:			https://scipy.org/about/
#			- 'jax'		JAX:			https://docs.jax.dev/
#			- 'nlopt'	NLopt:			https://nlopt.readthedocs.io/
#
#       For further information regarding them, please check the website(s) referenced 
#       above.


# ==== set environment options ==== 

import os

os.environ["OMP_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["VECLIB_MAXIMUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"
os.environ["JAX_ENABLE_X64"] = "True"
os.environ["JAX_PLATFORMS"] = "cpu" 


# ========== load modules ==========

import maya.api.OpenMaya as om
import maya.api.OpenMayaAnim as oma
import maya.cmds as cmds
import numpy as np
import nlopt
import jax
import jax.numpy as jnp
from jax.tree_util import register_pytree_node_class


################################################
# ========== Maya specific functions ========= #
################################################

# ========== signed distance field per joint function ==========

def sigDistField(jointName, meshes, subdivision, gridScale, cached = None):

	# Input variables:
	#   jointName = name of joint centre (represented by object in Maya scene; e.g. joint or locator)
	#   meshes = name(s) of the mesh(es) for which the signed distance field is calculated
	#   subdivision = number of elements per axis (e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points)
	#   gridScale = scale factor for the cubic grid dimensions
	# ======================================== #

	# get joint centre position

	jPos, jDag = getWSPos(jointName)

	# get number and names of ligaments

	ligAttributes = cmds.listAttr(jointName, ud = True) # get user defined attributes of 'jointName' (i.e. the float attributes that will contain the ligament lengths)

	# cycle through ligaments and extract their information

	distArr = []
	oDags = []
	iDags = []

	for ligament in ligAttributes:

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

	# cycle through all meshes

	if len(meshes) != 2:
		error('Please specify exactly TWO meshes (proximal, distal).')

	# get rotation matrices

	rotMat = np.array([jDag.exclusiveMatrix(), jDag.inclusiveMatrix()]).reshape(2, 4, 4)

	SDFs = []
	meshDistArr = []

	if cached is not None:

		for SDF in cached:

			meshSigDist = np.load(SDF)
			elements = np.linspace(-maxDist, maxDist, num = subdivision + 1, endpoint = True, dtype = float)

			# initialise jax.scipy.interpolate.RegularGridInterpolator with signed distance data on default cubic grid

			SDFs.append(jax.scipy.interpolate.RegularGridInterpolator((elements, elements, elements), meshSigDist, method = 'linear', fill_value = -1.0)) # grid will be initialised in its relative coordinate system from scaled [-size,-size,-size] to [size,size,size]
	else:
		for i, mesh in enumerate(meshes):
			meshSigDist, elements = sigDistMesh(mesh, rotMat[i], subdivision, maxDist) # get signed distance field for each mesh

			# initialise jax.scipy.interpolate.RegularGridInterpolator with signed distance data on default cubic grid

			SDFs.append(jax.scipy.interpolate.RegularGridInterpolator((elements, elements, elements), meshSigDist, method = 'linear', fill_value = -1.0)) # grid will be initialised in its relative coordinate system from scaled [-size,-size,-size] to [size,size,size]
			meshDistArr.append(meshSigDist)

	return SDFs, ligAttributes, oDags, iDags, maxDist, meshDistArr


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

	# create clean copy of mesh

	fnMesh = om.MFnMesh(shape)
	tempMesh = om.MFnMeshData().create()
	fnMesh.copy(mObj,tempMesh)
	tempFnMesh = om.MFnMesh(tempMesh)

	# get edges

	tempFnMesh.unlockVertexNormals(om.MIntArray())
	numEdges = tempFnMesh.numEdges
	edgeIdcs = om.MIntArray(range(numEdges))

	# smooth all edges

	tempFnMesh.setEdgeSmoothings(edgeIdcs, [True] * numEdges)
	tempFnMesh.cleanupEdgeSmoothing()
	tempFnMesh.updateSurface()

	# get the meshs transformation matrix

	meshMat = dag.inclusiveMatrix()
	meshMatInv = np.array(meshMat.inverse()).reshape(4,4)

	# create the intersector

	polyIntersect = om.MMeshIntersector()
	polyIntersect.create(tempMesh, meshMat) # apply intersector on clean copy
	get_closest_point = polyIntersect.getClosestPoint
	ptON = om.MPointOnMesh()

	# create 3D grid

	elements = np.linspace(-gridScale, gridScale, num = subdivision + 1, endpoint = True, dtype = float)
	X, Y, Z = np.meshgrid(elements, elements, elements, indexing='ij')
	points = np.vstack((X.ravel(), Y.ravel(), Z.ravel(), np.ones(X.size))).T

	# calculate position of vertices relative to cubic grid

	gridWsPos = points @ rotMat
	gridWSArr = gridWsPos[:,0:3]

	# go through grid points and calculate signed distance for each of them

	P = np.zeros(points.shape)
	N = np.zeros((X.size,3))
	ptRel = om.MPoint()

	for i, gridPoint in enumerate(gridWSArr):
		ptRel.x, ptRel.y, ptRel.z = gridPoint	# extract coordinates from gridPoint and feed into preallocated MPoint
		ptON = get_closest_point(ptRel)			# get point on mesh
		P[i,:] = ptON.point 					# point on mesh coordinates in mesh coordinate system
		N[i,:] = ptON.normal 					# normal at point on mesh

	# get vectors from gridPoints to their closest points on mesh

	diff = (gridWsPos @ meshMatInv)[:,0:3] - P[:,0:3]

	# get length of vectors (distance)

	dist = np.sqrt(np.einsum('ij,ij->i', diff, diff))
	dist_safe = np.where(dist == 0.0, 1e-8, dist)

	# get the vector directions from gridPoints to points on mesh

	normDiff = diff / dist_safe.reshape(-1,1) # direction of point relative to cubic grid

	# calculate dot product between the normal at ptON and vector to check if point is inside or outside of mesh

	dot = np.sum(N * normDiff, axis = 1)

	# get sign for distance from dot product

	signDist = dist_safe * np.sign(dot)

	return signDist.reshape(subdivision + 1, subdivision + 1, subdivision + 1), elements # convert signed distance array into cubic grid format


# ========== get ligament transformation matrix function ==========

@jax.jit
def getLigTransMat(oPos, iPos, jPos):

	# Input variables:
	#   oPos = origin position in global coordiate system in the form jnp.array([x, y, z, 1.0])
	#   iPos = insertion position in global coordiate system in the form jnp.array([x, y, z, 1.0])
	#   jPos = joint centre position in global coordiate system in the form jnp.array([x, y, z, 1.0])
	# ======================================== #

	# strip homogeneous coordinate and ensure 2D

	o = jnp.atleast_2d(oPos[:,0:3]) # (L,3)
	i = jnp.atleast_2d(iPos[:,0:3]) # (L,3)
	j = jPos[0:3]                   # (3,)
	j = j[None,:]                   # (1,3) broadcast to (L,3)

	# get axes directions

	x = i - o
	offset = jnp.linalg.norm(x, axis = 1, keepdims = True)   # (L,1)
	x = x / offset

	v = j - o
	z = jnp.cross(v, x)
	z = z / jnp.linalg.norm(z, axis = 1, keepdims = True)

	y = jnp.cross(z, x)
	y = y / jnp.linalg.norm(y, axis = 1, keepdims = True)

	# assemble transformation matrices

	trans = jnp.zeros((x.shape[0], 4, 4)) # 3D array of zeros
	trans = trans.at[:,0,0:3].set(x * offset)
	trans = trans.at[:,1,0:3].set(y * offset)
	trans = trans.at[:,2,0:3].set(z * offset)
	trans = trans.at[:,3,0:3].set(o)
	trans = trans.at[:,3,3].set(1) # set bottom right corner of all matrices to 1

	return trans, offset.squeeze(-1)


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


# ========== get animation curve ==========

def getAnimCurve(name, attribute):

	# Input variables:
	#   name = string representing object name
	#   attribute = string representing object's attribute
	# ======================================== #

	# get dependency node

	node = dagObjFromName(name)[0]
	depNode = om.MFnDependencyNode(node)

	# get plug

	plug = depNode.findPlug(attribute, False)

	# check if animation curve exists

	if plug.isDestination:
		sourceNode = plug.source().node()
		if sourceNode.hasFn(om.MFn.kAnimCurve):
			return oma.MFnAnimCurve(sourceNode) # return existing animation curve

	# animation curve does not exist and need to create it

	animCurve = oma.MFnAnimCurve()
	animCurve.create(plug, oma.MFnAnimCurve.kAnimCurveTL)
	
	return animCurve # return newly created animation curve


# ========== evaluate SDFs for cost and constraint functions ==========

@jax.jit
def eval_SDF(SDF, points):
	return SDF(points)


################################################
# ============ optimiser functions =========== #
################################################

# ========== combined signed distance field and path constraint function ==========

def cons_fun(params, args, numPoints, ipProx, ipDist): # ligament points cannot impinge bones

	# Input variables:
	#   params = array of Y and Z coordinates of ligament points (i.e., params = [(y_0, z_0),(y_1, z_1), ... ,(y_n-1, z_n-1)]). They are, however, flattened into a single array, i.e., [y0, z0, y1, z1, y2, z2, ... , y_n-1, z_n-1], and therefore need to be extracted.
	#   args = arguments tuple containing rotation matrices, offset, proximal and distal X coordinates, ligament array template and constraints array
	#   numPoints = number of ligament points
	#   ipProx = proximal signed distance field of class TricubicSDF
	#   ipDist = distal signed distance field of class TricubicSDF
	# ======================================== #

	(relRotMat, maxOffset, xProx, xDist, ligArr, cons) = args

	# extract y and z coordinates for matrix multiplications

	yz = params.reshape(numPoints, 2) # y = params[0::2], z = params[1::2]
	
	# get relative coordinates for SDF constraints (prox + dist) 

	relPosProx = xProx + yz @ relRotMat[0][1:3,0:3]
	relPosDist = xDist + yz @ relRotMat[1][1:3,0:3]

	# if any of the ligament points are inside a mesh they will be negative and thus violate the constraint

	cons_prox = eval_SDF(ipProx, relPosProx) # calculae proximal signed distances
	cons_dist = eval_SDF(ipDist, relPosDist) # calculae distal signed distances

	# path constraints

	pts = ligArr.at[1:-1,1:3].set(yz)
	diff = (pts[1:] - pts[:-1])[:,1:3] # calculate mediolatral offset between subsequent points
	cons_path = maxOffset - jnp.sum(diff * diff, axis = 1) # squared mediolateral offset can be maximally 3 times the squared distance between path segment along X-axis (i.e., arctan(offset/dist) ≤ 60° as tan(60°) = sqrt(3))

	# assemble constraint output

	cons = cons.at[:numPoints].set(cons_prox)			 # proximal SDF constraint
	cons = cons.at[numPoints:2*numPoints].set(cons_dist) # distal SDF constraint
	cons = cons.at[2*numPoints:].set(cons_path)			 # path (smoothness) constraint

	return cons # if any of the ligament points are inside a mesh they will be negative and thus violate the constraint


# ========== cost function for optimisation ==========

def cost_fun(params, args, numPoints): # minimise path length

	# Input variables:
	#   params = array of Y and Z coordinates of ligament points (i.e., params = [(y_0, z_0),(y_1, z_1), ... ,(y_n-1, z_n-1)]). They are, however, flattened into a single array, i.e., [y0, z0, y1, z1, y2, z2, ... , y_n-1, z_n-1], and therefore need to be extracted.
	#   args = arguments tuple containing rotation matrices, offset, proximal and distal X coordinates, ligament array template and constraints array
	#   numPoints = number of ligament points
	# ======================================== #

	(_, _, _, _, ligArr, _) = args

	# construct new array for cost function

	yz = params.reshape(numPoints, 2)
	pts = ligArr.at[1:-1,1:3].set(yz)

	# difference between consecutive xyz points

	diff = pts[1:] - pts[:-1]

	return (diff * diff).sum() # squared length of ligament from origin to insertion


# ========== precompile JAX functions ==========

cost_vg_jit = jax.jit(jax.value_and_grad(cost_fun, argnums = 0), static_argnums = 2)
cons_jit = jax.jit(cons_fun, static_argnums = 2)
jac_jit = jax.jit(jax.jacobian(cons_fun, argnums = 0), static_argnums = 2)


# ========== ligament length calculation function ==========

def ligCalc(initial_guess, ligArr, SDF, rotMat, ligTransforms, offset, keyPathPoints, maxOffset, numPoints, maxIter, maxDist):

	# Input variables:
	#	initial_guess = initual guess condition for optimiser
	#	ligArr = 2D array of ligament point coordinates of shape (N,4)
	#	SDF = list containing proximal and distal signed distance fields of class TricubicSDF
	#	rotMat = array with the transformation matrices of the joint and its parent
	#	ligTransforms = array with transformation matrices representing ligament coordinate systems
	#	offset = array of euclidean distances from origin to insertion for each ligament
	#	keyPathPoints = boolean to specify whether ligament point positions are to be keyed or not. True = yes, False = no
	#	maxOffset = squared maximum mediolateral offset for path constraint function
	#	numPoints = number of ligament points
	#	maxIter = integer value specifying the maximum number of iterations for the optimiser
	#	maxDist = float value of maximum cubic grid dimensions
	# ======================================== #

	# preallocate variables

	numLigs = ligTransforms.shape[0]
	consArr = jnp.zeros((3 * numPoints + 1))
	n_constraints = consArr.shape[0]
	
	results = []
	ligLengths = np.empty(numLigs)
	ligPoints = np.empty((numLigs, numPoints + 2, 3)) if keyPathPoints else None

	relRotMats = ligTransforms[:,None,:,:] @ rotMat[None,:,:,:]
	xCoords = ligArr[:,0]

	# define ligament length optimiser function (per ligament)

	def solve(initial_guess, args, numPoints, ipProx, ipDist):

		# extract arguments

		(relRotMat, maxOffset, xProx, xDist, ligArr_copy, cons) = args

		# define primary solver

		opt = nlopt.opt(nlopt.AUGLAG, initial_guess.size)

		# define secondary solver

		local_opt = nlopt.opt(nlopt.LD_LBFGS, initial_guess.size)
		local_opt.set_xtol_rel(1e-4)
		local_opt.set_ftol_rel(1e-4)
		opt.set_local_optimizer(local_opt)

		# NLopt cost function

		def nlopt_cost(x, grad):
			params = jnp.asarray(x)

			val, gradient = cost_vg_jit(params, args, numPoints)

			# calculate gradient

			if grad.size > 0:
				grad[:] = np.asarray(gradient, dtype = np.float64).flatten()

			return float(val)

		# NLopt constraint function

		def nlopt_constraints(result, x, grad):
			params = jnp.asarray(x)
			
			# get constraints

			val = cons_jit(params, args, numPoints, ipProx, ipDist)
			
			# invert constraints NLopt (c(x) <= 0)

			result[:] = np.asarray(-val, dtype = np.float64)
			
			if grad.size > 0:
				
				# calculate and invert jacobian NLopt (c(x) <= 0)
				
				jac = jac_jit(params, args, numPoints, ipProx, ipDist)

				grad[:] = np.asarray(-jac, dtype = np.float64)

		# define bounds

		lb = np.zeros(initial_guess.size, dtype = np.float64) + np.float64(-maxDist)
		ub = np.zeros(initial_guess.size, dtype = np.float64) + np.float64(maxDist)
		opt.set_lower_bounds(lb)
		opt.set_upper_bounds(ub)

		# set objective function

		opt.set_min_objective(nlopt_cost)
		
		# set constraints function 

		opt.add_inequality_mconstraint(nlopt_constraints, np.zeros(n_constraints, dtype = np.float64) + 1e-5)
		
		# set solver settings

		opt.set_maxeval(int(maxIter))
		opt.set_xtol_rel(1e-5)
		opt.set_ftol_rel(1e-5)

		try:
			opt_x = opt.optimize(initial_guess.astype(np.float64))
			status = opt.last_optimize_result()
			success = (1 <= status <= 4)
		except Exception:
			opt_x = np.copy(initial_guess).astype(np.float64)
			status = -1
			success = False
			
		nit = opt.get_numevals()
		
		# get output class similar to SciPy results

		class res:
			def __init__(self, x, success, status, nit):
				self.x = x
				self.success = success
				self.status = status
				self.nit = nit
		
		return res(opt_x, success, status, nit)

	# calculate length across all ligaments

	for index, ligRotMat in enumerate(ligTransforms):

		# precompute X‑column transform contributions

		xProx = xCoords[1:-1, None] * relRotMats[index,0,0,0:3] + relRotMats[index,0,3,0:3] 
		xDist = xCoords[1:-1, None] * relRotMats[index,1,0,0:3] + relRotMats[index,1,3,0:3] 

		args = (
			jnp.asarray(relRotMats[index]), 
			maxOffset, 
			jnp.asarray(xProx), 
			jnp.asarray(xDist), 
			ligArr, 
			consArr
		)

		# optimisation using NLopt and get results: res.x = ligament point coordinates, res.success optimisation exit status

		res = solve(initial_guess, args, numPoints, SDF[0], SDF[1])

		# extract output

		ligArrCopy = ligArr.at[1:-1,1:3].set(res.x.reshape(numPoints,2)) 
		diff = (ligArrCopy[1:] - ligArrCopy[:-1])[:,0:3]

		# calculate ligament length from points and correct relative length by linear distance between origin and insertion to get actual ligament length

		ligLengths[index] = np.sqrt(np.einsum('ij,ij->i', diff, diff)).sum() * offset[index] if res.success else -offset[index]

		# check if path points are to be calculate

		if keyPathPoints:
			ligPoints[index] = (ligArrCopy @ ligRotMat)[:,0:3]

		# gather results

		results.append(res)

	return ligLengths, ligPoints, results




