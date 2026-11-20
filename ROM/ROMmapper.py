#	ROMmapper.py
#
#	This script optimises contact-based positions across a set of rotational poses for
#	a set of bone meshes. It is an implementation of the Marai et al., 2006 and Lee
#	et al. 2023 approach for Autodesk Maya. It creates signed distance fields for the
#	proximal and distal bone meshes which are then used to caculate intersections
#	between the meshes and the distances between them. 
#
#	Written by Oliver Demuth 
#	Last updated 20.11.2026 - Oliver Demuth
#
#	SYNOPSIS:
#
#		INPUT params:
#			string	jointName:			Name of the joint centre (i.e. the name of a locator or joint; e.g., 'myJoint' if following the ROM mapping protocol of Manafzadeh & Padian 2018)
#			string	meshes:				Names of the two bone meshes and optional convex hull (i.e., several individual meshes representing the bones and rib cage; e.g., in the form of ['prox_mesh','dist_mesh', 'conv_hull'])
#			string	congruencyMeshes:	Names of the meshes to check articular congruency (i.e., several individual meshes; e.g., in the form of ['prox_art_surf','dist_art_surf'])
#			string	fittedShape:		Name of the shape fitted to the proximal articular surface (e.g., in the form 'prox_fitted_sphere') used for proximity estimation
#			float	xBounds:			Float array of bounds for X-axis rotation in the form of [min, max] (i.e., LAR, e.g., [-180,180] for spherical joints or [-90,90] for hinge joints)
#			float	yBounds:			Float array of bounds for Y-axis rotation in the form of [min, max] (i.e., ABAD, e.g.,  [-90,90] for spherical joints or [-90,90] for hinge joints)
#			float	zBounds:			Float array of bounds for Z-axis rotation in the form of [min, max] (i.e., FE, e.g.,  [-180,180] for spherical joints or [0,180] for hinge joints)
#			int		interval:			Integer value for the sampling interval between bounds (e.g., for FE and LAR -180:interval:180, and for ABAD -90:interval:90; see Manafzadeh & Padian, 2018)
#			int		gridSubdiv:			Integer value for the subdivision of the cube (i.e., number of grid points per axis; e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points)
#			float	gridSize:			Float value indicating the size of the cubic grid (i.e., the length of a side; e.g., 10 will result ing a cubic grid with the dimensions 10 x 10 x 10)
#			float	gridScale:			Float value for the scale factor of the cubic grid (i.e., 1.5 initialises the grid from -1.5 to 1.5)
#			float	thickness:			Float value indicating the thickness value which correlates with the joint spacing
#			float	weights:			Float array of weights for the cost function in the form of [w1, w2, w3, w4, w5]
#			float	tolerance: 			Float value for the joint proximity tolerance
#			float	scaleFactor:		Float value for the joint offset to determine joint disarticulation
#			float	cutOff:				Float value for the final SDF interpolation (default is 0.0)
#			int		maxIter:			Integer value specifying the maximum number of iterations for the SLSQP optimiser
#
#		RETURN params:
#			float 	coords:				Return value is a float array of 3D coordinates (X, Y, Z) of the distal bone (ACS) relative to the proximal one 
#			boolean	viable:				Return value is a boolean whether the rotational poses are viable or inviable (i.e, intersection or disarticulation of the bone meshes)
#
#
#	IMPORTANT notes:
#
#	(1) Meshes need realtively uniform face areas, otherwise large faces might skew 
#	    vertex normals in their direction. It is, therefore, important to extrude 
#	    edges around large faces prior to closing the hole at edges with otherwise 
#	    acute angles to circumvent this issue, e.g., if meshes have been cut to reduce
#	    polycount, prior to executing the Python scripts.
#
#	(2) For each bone (*) two sets of meshes are required, the articular surface and the
#	    bone mesh
#
#	(3) This script requires several modules for Python, see README file. Make sure to
#	    have the following external modules installed for the mayapy application:
#
#		- 'numpy' 	NumPy:	https://numpy.org/about/
#		- 'scipy'	SciPy:	https://scipy.org/about/
#		- 'jax'		JAX:	https://docs.jax.dev/
#		- 'nlopt'	NLopt:	https://nlopt.readthedocs.io/
#
#	    For further information regarding them, please check the website(s) referenced 
#	    above.


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

import jax
import nlopt
import jax.numpy as jnp
import numpy as np
import scipy as sp
import maya.cmds as cmds
import maya.api.OpenMaya as om
import maya.api.OpenMayaAnim as oma

from jax.tree_util import register_pytree_node_class


# ========== signed distance field per joint function ==========

def sigDistField(jDag, meshes, subdivision, size, scale):

	# Input variables:
	#	jDag = MDagPath to joint object
	#	meshes = name(s) of the mesh(es) for which the signed distance field is calculated
	#	subdivision = number of elements per axis (e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points)
	#	size = grid size of the cubic grid
	#	scale = scale factor for the cubic grid dimensions
	# ======================================== #

	# get rotation matrices

	rotMat = np.array([jDag.exclusiveMatrix(), jDag.inclusiveMatrix()]).reshape(2, 4, 4)

	if len(meshes) != 2:
		error('Please specify exactly TWO meshes (proximal, distal).')

	SDFs = []
	for i, mesh in enumerate(meshes):
		meshSigDist, elements = sigDistMesh(mesh, rotMat[i], subdivision, size * scale) # get signed distance field for each mesh

		# initialise jax.scipy.interpolate.RegularGridInterpolator with signed distance data on default cubic grid

		SDFs.append(jax.scipy.interpolate.RegularGridInterpolator((elements, elements, elements), meshSigDist, method = 'linear', fill_value = -1.0)) # grid will be initialised in its relative coordinate system from scaled [-size,-size,-size] to [size,size,size]

	return SDFs, rotMat


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

	elements = np.linspace(-gridScale, gridScale, num = subdivision + 1, endpoint = True, dtype = np.float64)
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

	return vertices @ np.linalg.inv(rotMat) # calculate new coordinates and return 3D array


# ========== get dag path function ==========

def dagObjFromName(name):

	# Input variables:
	#	name = string representing object name
	# ======================================== #

	sel = om.MSelectionList()
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

	# get mesh object

	fnMesh = om.MFnMesh(shape)
	vertices = np.array(fnMesh.getPoints(4))

	# get bounding box dimensions

	dims = np.array([fnMesh.boundingBox.width, fnMesh.boundingBox.height, fnMesh.boundingBox.depth])

	# substract center position from vertices and return mean distance

	return np.linalg.norm((vertices - vertices.mean(axis = 0)), axis = 1).mean(), dims


################################################
# ============ optimiser functions =========== #
################################################


# ========== check viability function ==========

def optimisePosition(proxArr, distArr, distMeshArr, paramArr, SDF, rotMat, thickness, weights, initial_guess, bounds, scaleFactor, maxIter, tol, cutOff):

	# Input variables:
	#   proxArr = 2D array of proximal articular surface homogeneous vertex coordinates (Nx4) for fast computation of relative coordinates 
	#   distArr = 2D array of distal articular surface homogeneous vertex coordinates (Mx4) for fast computation of relative coordinates 
	#   distMeshArr = 3D array of distal mesh vertex transformation matrices for fast computation of relative coordinates
	#	paramArr = 1D array of ones
	#   SDF = list containing multiple signed distance fields of class jax.scipy.interpolate.RegularGridInterpolator (e.g., [ipProx, ipDist])
	#   rotMat = array with the transformation matrices of the joint and its parent
	#   thickness = thickness measure correlated with joint spacing
	#   weights = weights for the individual cost function terms
	#   initial_guess = initual guess condition for optimiser
	#   bounds = bounds for optimisation
	#   scaleFactor = scale factor to roughly check if joint is disarticulated
	#   maxIter = maximum number of iterations
	#   tol = tolerance for joint proximity
	#   cutOff = cut off value for signDist 
	# ======================================== #

	# initialise evaluator for translation optimisation

	e = ROMeval(proxArr, distArr, SDF[0], SDF[1], rotMat, thickness, weights)

	# define primary solver

	opt = nlopt.opt(nlopt.AUGLAG, initial_guess.size)

	# define secondary solver

	local_opt = nlopt.opt(nlopt.LD_LBFGS, initial_guess.size)
	local_opt.set_xtol_rel(1e-4)
	local_opt.set_ftol_rel(1e-4)
	opt.set_local_optimizer(local_opt)

	# define bounds

	opt.set_lower_bounds(np.array(bounds[:,0], dtype = np.float64))
	opt.set_upper_bounds(np.array(bounds[:,1], dtype = np.float64))

	# set objective function 

	opt.set_min_objective(e.nlopt_cost)

	# set constraints function 

	opt.add_inequality_constraint(e.nlopt_constraint, 1e-5)

	# set solver settings

	opt.set_maxeval(int(maxIter))
	opt.set_xtol_rel(1e-4)
	opt.set_ftol_rel(1e-4)

	try:
		opt_x = opt.optimize(initial_guess.astype(np.float64))
		status = opt.last_optimize_result()
		success = (1 <= status <= 4)
	except Exception:
		opt_x = np.copy(initial_guess).astype(np.float64)
		status = -1
		success = False

	if not success: 
		return [], False # empty list, viable

	diff = np.sqrt(np.dot(opt_x,opt_x)) # get offset from joint centre. Negligible slower than MVector(results.x).length() but software package agnostic
	
	# check for disarticulation 
	
	if diff > (scaleFactor * thickness): # first crudely (if distal ACS is more than 10% beyond radius of fitted proximal shape; default value is 2.2: thickness = 0.5 * radius)
		return [], False # empty list, viable

	# get transformation matrix

	paramArr[0:3] = opt_x
	transMat = rotMat[1].copy() # transformation matrix
	transMat[3,:] = paramArr @ rotMat[0] # append result world space coordinates to transformation matrix

	# calculate position of vertices relative to cubic grid
	
	artRelArr = jnp.asarray((proxArr @ (rotMat[0] @ np.linalg.inv(transMat)))[:,0:3]) # get inverse of transformation matrix
	meshRelArr = jnp.asarray((distMeshArr @ (transMat @ rotMat[2]))[:,0:3])

	avgdist = np.asarray(eval_SDF(SDF[1],artRelArr))
	avgdist = avgdist[avgdist != -1].mean() # remove outliers that are beyond cubic grid and get mean interarticular distance 

	signDist = np.asarray(eval_SDF(SDF[0],meshRelArr))# make sure that bone meshes do not intersect
	
	# if disarticulated or any points impinge meshes the position becomes inviable

	if avgdist < (tol * thickness) and signDist[signDist != -1].min() > cutOff: # remove outliers that are beyond cubic grid and set target thickness limit to tolerance (i.e., 1.07 based on experimental data)
		return opt_x, True # coords, viable
	else:
		return [], False # empty list, viable


# ========== evaluate SDFs for cost and constraint functions ==========

@jax.jit
def eval_SDF(SDF,points):
	return SDF(points)

# ========== evaluate SDFs for cost and constraint functions ==========

@jax.jit()
def evaluate_SDFs(obj, params, forceBoth = False):
	
	# get transformation matrix from params

	param_hom = jnp.append(params,1.0)
	t_hom = param_hom @ obj.parentMat
	transMat = obj.localMat.at[3,:].set(t_hom)

	# proximal points in distal grid coordinates and evaluate SDF

	def eval_prox():

		# fast transformation matrix inverse

		R = transMat[0:3,0:3]
		t = transMat[3,0:3]
		RT = R.T

		# get scale

		scale_sq = jnp.sum(R[0,:] * R[0,:]) # X-axis will suffice as they should all be uniformly scaled

		# assemble inverse and correct for scaling

		invTrans = jnp.eye(4).at[0:3,0:3].set(RT / scale_sq).at[3,0:3].set((-t @ RT) / scale_sq)

		# get relative coordinates

		parent_invTrans = obj.parentMat @ invTrans
		relProx_hom = obj.proxArr @ parent_invTrans

		return eval_SDF(obj.ipDist,relProx_hom[:,0:3])

	# check if distal SDF is required otherwise skip

	proxDist = jax.lax.cond(obj.useProx or forceBoth, eval_prox, lambda: jnp.zeros_like(obj.proxArr[:,0]))

	# distal points in proximal grid coordinates and evaluate SDF

	def eval_dist():

		# ger relative coordinates

		trans_child = transMat @ obj.childMat
		relDist_hom = obj.distArr @ trans_child
		return eval_SDF(obj.ipProx,relDist_hom[:,0:3])

	# check if proximal SDF is required otherwise skip

	distDist = jax.lax.cond(obj.useDist or forceBoth, eval_dist, lambda: jnp.zeros_like(obj.distArr[:,0]))

	return proxDist, distDist


# ========== cost function for optimisation ==========

@jax.jit
def cost(obj, params):
	
	# evaluate proximal and distal SDF

	proxDist, distDist = evaluate_SDFs(obj, params, forceBoth = False)
	
	# proximal joint spacing (prox art surf in ipDist)

	proxMean = proxDist.mean()
	proxVar = proxDist - proxMean
	proxOffset = (proxMean - obj.thickness)
	term0 = proxOffset * proxOffset

	# proximal joint congruency

	term2 = jnp.dot(proxVar, proxVar) / jnp.maximum(proxVar.size, 1)

	# distal joint spacing (prox art surf in ipDist)

	distMean = distDist.mean()
	distVar = distDist - distMean
	distOffset = distMean - obj.thickness
	term1 = distOffset * distOffset

	# distal joint congruency

	term3 = jnp.dot(distVar, distVar) / jnp.maximum(distVar.size, 1)

	# calculate joint offset 

	paramsEPS = params + 1e-12
	term4 = jnp.sqrt(jnp.dot(paramsEPS, paramsEPS))

	# apply weights to cost terms and sum

	return jnp.dot(obj.w, jnp.array([term0, term1, term2, term3, term4]))


# ========== signed distance field constraint function ==========

@jax.jit
def constraint(obj, params):
	
	# evaluate proximal and distal SDF

	proxDist, distDist = evaluate_SDFs(obj, params, forceBoth = True)

	# find largest impingement

	return jnp.minimum(proxDist.min(),distDist.min())


# ========== precompile cost and constraint functions ==========

cost_vg_jit = jax.jit(jax.value_and_grad(cost, argnums = 1))
cons_vg_jit = jax.jit(jax.value_and_grad(constraint, argnums = 1))



################################################
# ========== shared evaluator class ========== #
################################################

@register_pytree_node_class
class ROMeval:
	def __init__(self, proxArr, distArr, ipProx, ipDist, rotMat, thickness, w, dtype = jnp.float64):

		# Input variables:
		#   proxArr = 2D array of proximal articular surface homogeneous vertex coordinates (Nx4) for fast computation of relative coordinates 
		#   distArr = 2D array of distal articular surface homogeneous vertex coordinates (Mx4) for fast computation of relative coordinates 
		#   ipProx = proximal signed distance field of jax.scipy.interpolate.RegularGridInterpolator class
		#   ipDist = distal signed distance field of jax.scipy.interpolate.RegularGridInterpolator class
		#   rotMat = array with the transformation matrices of the joint and its parent
		#   thickness = thickness measure correlated with joint spacing. Passed through args, not part of this constraint function
		#   w = weights for the individual cost function terms
		# ======================================== #
		
		# store contiguous arrays and callables

		self.proxArr = jnp.asarray(proxArr, dtype = dtype)
		self.distArr = jnp.asarray(distArr, dtype = dtype)
		self.w = jnp.asarray(w, dtype = dtype)
		self.thickness = float(thickness)

		# get rotation matrices is an iterable of 3 (4x4) matrices: [parent, local, child]
		
		self.parentMat = jnp.asarray(rotMat[0], dtype = dtype)
		self.localMat  = jnp.asarray(rotMat[1], dtype = dtype)
		self.childMat  = jnp.asarray(rotMat[2], dtype = dtype)

		# store signed distance fields

		self.ipProx = ipProx
		self.ipDist = ipDist

		# get weight masks

		self.useProx = bool(w[0] != 0 or w[2] != 0)
		self.useDist = bool(w[1] != 0 or w[3] != 0)

		# reset cache

		self.reset_cache()


	# ========== reset cache ==========

	def reset_cache(self):
		self._rom_cache = {
			"params": None,
			"cost_val": None,
			"cost_grad": None,
			"cons_val": None,
			"cons_grad": None
		}


	# ========== flatten and unflatten ==========

	def tree_flatten(self):

		# get dynamic attributes (JAX-arrays)

		children = (self.proxArr, self.distArr, self.w, self.parentMat, self.localMat, self.childMat)
		
		# get static auxiliary metadata

		aux_data = {
			"thickness": self.thickness,
			"ipProx": self.ipProx,
			"ipDist": self.ipDist,
			"useProx": self.useProx,
			"useDist": self.useDist
		}

		return (children, aux_data)

	@classmethod
	def tree_unflatten(cls, aux_data, children):

		# extract dynamic attributes 

		proxArr, distArr, w, parentMat, localMat, childMat = children
		
		# create a fresh instance without invoking __init__ again

		obj = cls.__new__(cls)
		obj.proxArr = proxArr
		obj.distArr = distArr
		obj.w = w
		obj.parentMat = parentMat
		obj.localMat = localMat
		obj.childMat = childMat
		
		# get static data 

		obj.thickness = aux_data["thickness"]
		obj.ipProx  = aux_data["ipProx"]
		obj.ipDist  = aux_data["ipDist"]
		obj.useProx = aux_data["useProx"]
		obj.useDist = aux_data["useDist"]

		obj.reset_cache()
		
		return obj


	# ========== NLopt cost function call with cache ==========

	def nlopt_cost(self, x, grad):

		# Input variables:
		#   x = array of optimised X, Y and Z coordinates of distal element position
		#	grad = gradient object for NLopt
		# ======================================== #
		
		# convert to numpy array

		params= np.asarray(x, dtype = np.float64)
		params_jnp = jnp.asarray(params)
		
		# check cached values and if params unchanged, check validity mask

		if self._rom_cache["params"] is not None and np.array_equal(params, self._rom_cache["params"]):
			if self._rom_cache["cost_val"] is not None:
				if grad.size > 0: 
					grad[:] = self._rom_cache["cost_grad"]
				return self._rom_cache["cost_val"]
		else:

			#partial reset protects cross-talk parameter validation inside NLopt AUGLAG

			self._rom_cache["params"] = np.copy(params)
			self._rom_cache["cost_val"] = None
			self._rom_cache["cons_val"] = None

		# get cost and gradient

		val, gradient = cost_vg_jit(self, params_jnp)
		val = float(val)
		gradient = np.asarray(gradient, dtype = np.float64)

		# cache cost and gradient
		
		self._rom_cache["cost_val"] = val
		self._rom_cache["cost_grad"] = gradient

		# set gradient
		
		if grad.size > 0: 
			grad[:] = gradient

		return val


	# ========== NLopt constraint function call with cache ==========

	def nlopt_constraint(self, x, grad):

		# Input variables:
		#   x = array of optimised X, Y and Z coordinates of distal element position
		#	grad = gradient object for NLopt
		#	cons_vg_jit = jitted function of constraint and jacobian
		# ======================================== #

		# convert to numpy array

		params = np.asarray(x, dtype = np.float64)
		params_jnp = jnp.asarray(params)

		# check cached values and if params unchanged, check validity mask
		
		if self._rom_cache["params"] is not None and np.array_equal(params, self._rom_cache["params"]):
			if self._rom_cache["cons_val"] is not None:
				if grad.size > 0: 
					grad[:] = self._rom_cache["cons_grad"]
				return self._rom_cache["cons_val"]
		else:

			# partial reset protects cross-talk parameter validation inside NLopt AUGLAG

			self._rom_cache["params"] = np.copy(params)
			self._rom_cache["cost_val"] = None
			self._rom_cache["cons_val"] = None
		
		# calculate and invert constraints for NLopt (c(x) <= 0)

		val, jac = cons_vg_jit(self, params_jnp)
		val = float(-val)

		# invert jacobian for NLopt (c(x) <= 0)

		jac = np.asarray(-jac, dtype = np.float64)

		# cache inverted constraints and Jacobian for NLopt 

		self._rom_cache["cons_val"] = val
		self._rom_cache["cons_grad"] = jac

		# set Jacobian
		
		if grad.size > 0: 
			grad[:] = jac

		return val

