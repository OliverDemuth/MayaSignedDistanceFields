#	ROMmapper.py
#
#	This script optimises contact-based positions across a set of rotational poses for
#	a set of bone meshes. It is an implementation of the Marai et al., 2006 and Lee
#	et al. 2023 approach for Autodesk Maya. It creates signed distance fields for the
#	proximal and distal bone meshes which are then used to caculate intersections
#	between the meshes and the distances between them. 
#
#	Written by Oliver Demuth 
#	Last updated 13.02.2026 - Oliver Demuth
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
#		- 'numpy' 	NumPy:			https://numpy.org/about/
#		- 'scipy'	SciPy:			https://scipy.org/about/
#
#	    For further information regarding them, please check the website(s) referenced 
#	    above.


# ========== load modules ==========

import maya.api.OpenMaya as om
import numpy as np
import scipy as sp


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

	rotMat = []
	rotMat.append(np.array(jDag.exclusiveMatrix()).reshape(4,4)) # parent rotMat (prox) as numpy 4x4 array
	rotMat.append(np.array(jDag.inclusiveMatrix()).reshape(4,4)) # child rotMat (dist) as numpy 4x4 array

	# cycle through all meshes

	if len(meshes) < 2:
		error('Too few meshes specified. Please specify at least two meshes in the mesh array.')
	if len(meshes) > 2:
		rotMat.append(rotMat[0]) # parent rotMat (prox) as numpy 4x4 array
	if len(rotMat) < len(meshes):
		error('Fewer rotation matrices than meshes supplied. Please assign corresponding rotation matrices for each mesh.')

	SDFs = []
	for j,mesh in enumerate(meshes):
		meshSigDist, elements = sigDistMesh(mesh, rotMat[j], subdivision, size * scale) # get signed distance field for each mesh

		# initialise tricubic interpolator with signed distance data on default cubic grid

		SDFs.append(sp.interpolate.RegularGridInterpolator((elements, elements, elements), meshSigDist, method = 'cubic', bounds_error = False, fill_value = -1)) # grid will be initialised in its relative coordinate system from scaled [-size,-size,-size] to [size,size,size]

	return SDFs, rotMat


# ========== signed distance field per mesh function ==========

def sigDistMesh(mesh, rotMat, subdivision, scale):

	# Input variables:
	#	mesh = name of the mesh for which the signed distance field is calculated
	#	rotMat = transformation matrix of parent (joint) of the mesh
	#	subdivision = number of elements per axis (e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points)
	#	scale = scale factor for the cubic grid dimensions
	# ======================================== #

	# get dag paths

	dag = dagObjFromName(mesh)[1]

	# create MObject

	shape = dag.extendToShape()
	mObj = shape.node()

	# get the meshs transformation matrix

	meshMat = dag.inclusiveMatrix()
	meshMatInv = np.linalg.solve(np.array(meshMat).reshape(4,4), np.eye(4))

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

	sigDist = dist * np.sign(dot)

	return sigDist.reshape(subdivision + 1, subdivision + 1, subdivision + 1), elements # convert signed distance array into cubic grid format


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

	return vertices @ np.linalg.solve(rotMat, np.eye(4)) # calculate new coordinates and return 3D array


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

def optimisePosition(proxArr, distArr, distMeshArr, SDF, rotMat, thickness, weights, initial_guess, bounds, scaleFactor, maxIter, tol, cutOff):

	# Input variables:
	#   proxArr = 3D array of proximal articular surface vertex transformation matrices for fast computation of relative coordinates 
	#   distArr = 3D array of distal articular surface vertex transformation matrices for fast computation of relative coordinates 
	#   distMeshArr = 3D array of distal mesh vertex transformation matrices for fast computation of relative coordinates
	#   SDF = list containing multiple signed distance fields in tricubic form (e.g., [ipProx, ipDist, ipConv])
	#   rotMat = array with the transformation matrices of the joint and its parent
	#   thickness = thickness measure correlated with joint spacing
	#   weights = weights for the individual cost function terms
	#   initial_guess = initual guess condition for optimiser
	#   bounds = bounds for optimisation
	#   threshold = scale factor to roughly check if joint is disarticulated
	#   maxIter = maximum number of iterations
	#   tol = tolerance for joint proximity
	#   cutOff = cut off value for signDist 
	# ======================================== #

	# initialise evaluator for translation optimisation

	e = ROMeval(proxArr, distArr, SDF[0], SDF[1], rotMat, thickness, weights)

	# set constraints functions

	cons = ({'type': 'ineq',    # set type to inequality, which means that it is to be non-negative
			 'fun': e.constraint})  # set constraint function from evaluator

	# set options

	options = {"maxiter": maxIter} # if it doesn't solve within 25 iterations it usually won't solve. Any more than 50 will just increase computational time without much benefit
	
	# optimise translation for specific rotational pose using SLSQP: results.X = joint centre point coordinates

	results = sp.optimize.minimize(e.cost, initial_guess, bounds = bounds, method = 'SLSQP', constraints = cons, options = options)

	# check if optimisation was successful

	if not results.success: 
		return [], False # empty list, viable

	diff = np.linalg.norm(results.x) # get offset from joint centre. Negligible slower than MVector(results.x).length() but software package agnostic
	
	# check for disarticulation 
	
	if diff > (scaleFactor * thickness): # first crudely (if distal ACS is more than 10% beyond radius of fitted proximal shape; default value is 2.2: thickness = 0.5 * radius)
		return [], False # empty list, viable

	# get transformation matrix

	transMat = rotMat[1] # transformation matrix
	transMat[3,:] = np.append(results.x,1.0) @ rotMat[0] # append result world space coordinates to transformation matrix

	# calculate position of vertices relative to cubic grid
	
	artRelArr = (proxArr @ np.linalg.solve(transMat.T, rotMat[0].T).T)[:,0:3] # get inverse of transformation matrix
	meshRelArr = (distMeshArr @ (transMat @ rotMat[2]))[:,0:3]

	avgdist = SDF[1](artRelArr)
	avgdist = avgdist[avgdist != -1].mean() # remove outliers that are beyond cubic grid and get mean interarticular distance 

	signDist = SDF[0](meshRelArr) # make sure that bone meshes do not intersect
	
	# if disarticulated or any points impinge meshes the position becomes inviable

	if avgdist < (tol * thickness) and signDist[signDist != -1].min() > cutOff: # remove outliers that are beyond cubic grid and set target thickness limit to tolerance (i.e., 1.07 based on experimental data)
		return results.x, True # coords, viable
	else:
		return [], False # empty list, viable



################################################
# ========== shared evaluator class ========== #
################################################


class ROMeval:
	def __init__(self, proxArr, distArr, ipProx, ipDist, rotMat, thickness, w, dtype = np.float64):

		# Input variables:
		#   proxArr = 3D array of proximal articular surface vertex transformation matrices for fast computation of relative coordinates 
		#   distArr = 3D array of distal articular surface vertex transformation matrices for fast computation of relative coordinates 
		#   ipProx = proximal signed distance field in form of scipy.interpolate.RegularGridInterpolator
		#   ipDist = distal signed distance field in form of scipy.interpolate.RegularGridInterpolator
		#   rotMat = array with the transformation matrices of the joint and its parent
		#   thickness = thickness measure correlated with joint spacing. Passed through args, not part of this constraint function
		#   w = weights for the individual cost function terms
		# ======================================== #
		
		# store contiguous arrays and callables

		self.proxArr = np.ascontiguousarray(proxArr, dtype = dtype) # shape (N,4,4)
		self.distArr = np.ascontiguousarray(distArr, dtype = dtype) # shape (M,4,4)
		self.w = np.ascontiguousarray(w, dtype = dtype) # shape (5,1)

		self._proxDist = None
		self._distDist = None
		self._paramCoords = np.ones(4)

		# store signed distance fields

		self.ipProx = ipProx
		self.ipDist = ipDist

		# rotMat is an iterable of 3 (4x4) matrices: [parent, local, child]

		self.rotMat = [np.ascontiguousarray(m, dtype = dtype) for m in rotMat]
		self.thickness = float(thickness)

		# last-eval cache (only keep last to bound memory)

		self._last_params = None


	# ========== compute and cache calculations for cost and constraint functions ==========

	def _evaluate(self, params):

		# Input variables:
		#	params = array of optimised X, Y and Z coordinates of distal element position
		# ======================================== #

		# check cached values

		if self._last_params is not None and all(params == self._last_params): return # params have not changed, no need to recalculate expensive SDF interpolation

		# params are not the same as cached values

		self._paramCoords[0:3] = params # add trailing one to params for matrix multiplications

		# extract coordinates from params and transform them into transformation matrix coords

		transMat = self.rotMat[1] # transformation matrix
		transMat[3,:] = self._paramCoords @ self.rotMat[0] # append result world space coordinates to transformation matrix

		# go through each proximal articular surface point and check if any of them intersect with distal mesh (i.e., signDist < 0)

		self._proxDist = self.ipDist((self.proxArr @ np.linalg.solve(transMat.T, self.rotMat[0].T).T)[:,0:3]) # calculate signed distances for the proximal articular surface
		self._distDist = self.ipProx((self.distArr @ (transMat @ self.rotMat[2]))[:,0:3])

		# update cache

		self._last_params = params.copy()


	# ========== cost function for optimisation ==========

	def cost(self, params):

		# Input variables:
		#   params = array of optimised X, Y and Z coordinates of distal element position
		# ======================================== #

		self._evaluate(params) # evaluate cached values and update if necessary

		# calculate mean distances and variance 

		proxDist = self._proxDist
		meanDist = proxDist.mean() 
		proxVar = proxDist - proxMeanDist

		distDist = self._distDist # distal art surf in proximal SDF
		distMeanDist = distDist.mean()
		distVar = distDist - distMeanDist

		# calculate cost terms

		terms = np.array([(proxMeanDist - self.thickness) ** 2,     # proximal joint spacing (prox art surf in ipDist)
						  (distMeanDist - self.thickness) ** 2,     # distal joint spacing (dist art surf in ipProx)
						  np.dot(proxVar, proxVar) / proxVar.size,  # proximal joint congruency
						  np.dot(distVar, distVar) / distVar.size,  # distal joint congruency
						  np.linalg.norm(params)])                  # joint offset 

		# apply weights to cost terms and sum

		return np.dot(self.w, terms)


	# ========== signed distance field constraint function ==========

	def constraint(self, params):

		# Input variables:
		#   params = array of optimised X, Y and Z coordinates of distal element position
		# ======================================== #

		self._evaluate(params) # evaluate cached values and update if necessary

		# go through each distal articular surface point and check if any of them intersect with proximal mesh (i.e., signDist < 0)

		return min(self._proxDist.min(), self._distDist.min()) # return minimal value, if any of the points are inside a mesh (intersection) it will be negative


