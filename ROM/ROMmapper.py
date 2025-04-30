#	ROMmapper.py
#
#	This script optimises contact-based positions across a set of rotational poses for
#	a set of bone meshes. It is an implementation of the Marai et al., 2006 and Lee
#	et al. 2023 approach for Autodesk Maya. It creates signed distance fields for the
#	proximal and distal bone meshes which are then used to caculate intersections
#	between the meshes and the distance between them. 
#
#	Written by Oliver Demuth 
#	Last updated 30.04.2025 - Oliver Demuth
#
#	SYNOPSIS:
#
#		INPUT params:
#			string	jointName:		Name of the joint centre, i.e. the name of a locator or joint (e.g., 'myJoint' if following the ROM mapping protocol of Manafzadeh & Padian 2018)
#			string	meshes:			Names of the two bone meshes and optional convex hull (i.e., several individual meshes representing the bones and rib cage; e.g., in the form of ['prox_mesh','dist_mesh', 'conv_hull'])
#			string	congruencyMeshes:	Names of the meshes to check articular congruency (i.e., several individual meshes; e.g., in the form of ['prox_art_surf','dist_art_surf'])
#			int	gridSubdiv:		Integer value for the subdivision of the cube (i.e., number of grid points per axis; e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points)
#			float	gridSize:		Float value indicating the size of the cubic grid (i.e., the length of a side; e.g., 10 will result ing a cubic grid with the dimensions 10 x 10 x 10)
#			float	gridScale:		Float value for the scale factor of the cubic grid (i.e., 1.5 initialises the grid from -1.5 to 1.5)
#			float	thickness:		Float value indicating the thickness value which correlates with the joint spacing
#
#		RETURN params:
#			array 	coords:			Return value is a an array of 3D coordinates of the distal bone relative to the proximal one
#			boolean	viable:			Return value is a boolean whether the rotational poses are viable or inviable (i.e, intersection or disarticulation of the bone meshes)
#			object 	results: 		Return value is an object of the scipy.optimize.OptimizeResult class. It represents the output of the scipy.optimize.minimize() function
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
#		- 'tricubic' 	Daniel Guterding: 	https://github.com/danielguterding/pytricubic
#				
#	    For further information regarding them, please check the website(s) referenced 
#	    above.


# ========== load modules ==========

import maya.api.OpenMaya as om
from maya.api.OpenMaya import MVector, MPoint, MTransformationMatrix
import numpy as np
import scipy as sp

from tricubic import tricubic



################################################
# ========== Maya specific functions ========= #
################################################


# ========== signed distance field per joint function ==========

def sigDistField(jointName, meshes, subdivision, size, scale):

	# Input variables:
	#	jointName = name of joint centre (represented by object, e.g. joint or locator, in Maya scene)
	#	meshes = name(s) of the mesh(es) for which the signed distance field is calculated
	#	subdivision = number of elements per axis, e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points
	#	size = grid size of the cubic grid
	#	scale = scale factor for the cubic grid dimensions
	# ======================================== #

	# get joint centre position

	jDag = dagObjFromName(jointName)[1]
	jInclTransMat = MTransformationMatrix(jDag.inclusiveMatrix()) # world transformation matrix of joint
	jExclTransMat = MTransformationMatrix(jDag.exclusiveMatrix()) # world transformation matrix of parent of joint

	# normalize matrices by size

	jInclTransMat.setScale([size,size,size],4) # set scale in world space (om.MSpace.kWorld = 4)
	jExclTransMat.setScale([size,size,size],4) # set scale in world space (om.MSpace.kWorld = 4)

	# get rotation matrices

	rotMat = []
	rotMat.append(np.array(jExclTransMat.asMatrix()).reshape(4,4)) # parent rotMat (prox) as numpy 4x4 array
	rotMat.append(np.array(jInclTransMat.asMatrix()).reshape(4,4)) # child rotMat (dist) as numpy 4x4 array
	rotMat.append(np.array(jExclTransMat.asMatrix()).reshape(4,4)) # parent rotMat (prox) as numpy 4x4 array

	# cycle through all meshes

	if len(meshes) < 2:
		error('Too few meshes specified. Please specify at least two meshes in the mesh array.')
	if len(rotMat) < len(meshes):
		error('Fewer rotation matrices than meshes supplied. Please assign corresponding rotation matrices for each mesh.')

	SDFs = []
	for j,mesh in enumerate(meshes):
		meshSigDist = sigDistMesh(mesh, rotMat[j], subdivision, scale) # get signed distance field for each mesh

		# initialise tricubic interpolator with signed distance data on default cubic grid

		SDFs.append(tricubic(meshSigDist.tolist(), list(meshSigDist.shape))) # grid will be initialised in its relative coordinate system from [0,0,0] to [gridSubdiv+1, gridSubdiv+1, gridSubdiv+1].
	
	return SDFs, rotMat


# ========== signed distance field per mesh function ==========

def sigDistMesh(mesh, rotMat, subdivision, scale):

	# Input variables:
	#	mesh = name of the mesh for which the signed distance field is calculated
	#	rotMat = transformation matrix of parent (joint) of the mesh
	#	subdivision = number of elements per axis, e.g., 20 will result in a cube grid with 21 x 21 x 21 grid points
	#	scale = scale factor for the cubic grid dimensions
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

	elements = np.linspace(-scale, scale, num = subdivision + 1, endpoint=True, dtype=float)

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
		P[i,:] = [ptON.point.x, ptON.point.y, ptON.point.z] # point on mesh coordinates in mesh coordinate system
		N[i,:] = [ptON.normal.x, ptON.normal.y, ptON.normal.z] # normal at point on mesh

	# get vectors from gridPoints to their closest points on mesh

	diff = np.dot(gridWSArr,meshMatInv)[:,3,0:3] - P

	# get length of vectors (distance)

	dist = np.linalg.norm(diff, axis = 1)

	# get the vectors' direction from gridPoints to points on mesh

	normDiff = diff/dist.reshape(-1,1) # direction of points relative to cubic grid points

	# calculate dot product between the normal at ptON and vector to check if point is inside or outside of mesh

	dot = np.sum(N * normDiff, axis = 1)

	# get sign for distance from dot product

	sigDist = dist * np.sign(dot)

	return sigDist.reshape(subdivision + 1, subdivision + 1, subdivision + 1) # convert signed distance array into cubic grid format


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



################################################
# ============ optimiser functions =========== #
################################################


# ========== check viability function ==========

def optimisePosition(proxArr, distArr, distMeshArr, SDF, gridRotMat, rotMat, thickness):

	# Input variables:
	#	proxArr = 3D array of proximal articular surface vertex transformation matrices for fast computation of relative coordinates 
	#	distArr = 3D array of distal articular surface vertex transformation matrices for fast computation of relative coordinates 
	#	distMeshArr = 3D array of distal mesh vertex transformation matrices for fast computation of relative coordinates
	#	SDF = list containing multiple signed distance fields in tricubic form (e.g., [ipProx, ipDist, ipConv])
	#	gridRotMat = rotation matrix of default cubic grid
	#	rotMat = array with the transformation matrices of the joint and its parent
	#	thickness = thickness measure correlated with joint spacing
	# ======================================== #

	# optimise translation for specific rotational pose

	results = posOptMin(proxArr, distArr, SDF[0], SDF[1], gridRotMat, rotMat, thickness)
	
	# check if optimisation was successful
	
	if results.success: 
	
		diff = MVector(results.x).length() # get offset from joint centre

		# check for disarticulation 
						  
		if diff < (1.1 * thickness * 2): # first crudely (if distal ACS is more than 10% beyond radius of fitted proximal shape)

			# get coordinates of results and transform them into transformation matrix coords

			resCoords = np.eye(4)
			resCoords[3,0:3] = results.x

			# get transformation matrix

			transMat = rotMat[1] # transformation matrix
			transMat[3,0:3] = np.dot(resCoords,rotMat[2])[3,0:3] # append result world space coordinates to transformation matrix
			transMatInv = np.linalg.inv(transMat) # get inverse of transformation matrix

			# calculate position of vertices relative to cubic grid

			artRelArr = np.dot(proxArr,np.dot(np.dot(rotMat[0],transMatInv),gridRotMat))[:,3,0:3].tolist()
			meshRelArr = np.dot(distMeshArr,np.dot(np.dot(transMat,rotMat[3]),gridRotMat))[:,3,0:3].tolist() # both the convex hull and the proximal signed distance fields are in the parent coordinate system
			
			avgdist = sum([SDF[1].ip(vtx) for vtx in artRelArr]) / len(artRelArr) # get average interarticular distance 
			signDist = ([SDF[0].ip(vtx) for vtx in meshRelArr]) # make sure that bone meshes do not intersect (i.e., check the distal mesh versus the proximal signed distance field)

			# check if convex hull signed distance field is provided (i.e., representing body shape; e.g., rib cage)
			
			if len(SDF) > 2:
				signDist.extend([SDF[2].ip(vtx) for vtx in meshRelArr]) # make sure that distal meshes does not intersect with convex hull
			
			# if disarticulated or any points penetrate meshes the position becomes inviable
		
			if avgdist < (1.07 * thickness) and all(dist > 0 for dist in signDist): # set target thickness limit to 7% based on experimental data
				viable = True
			else:
				viable = False
		else:
			viable = False		
	else:
		viable = False

	# gather results

	return results.x, viable, results


# ========== translation optimisation ==========

def posOptMin(proxArr, distArr, ipProx, ipDist, gridRotMat, rotMat, thickness):

	# Input variables:
	#	proxArr = 3D array of proximal articular surface vertex transformation matrices for fast computation of relative coordinates 
	#	distArr = 3D array of distal articular surface vertex transformation matrices for fast computation of relative coordinates 
	#	ipProx = proximal signed distance field in tricubic form
	#	ipDist = distal signed distance field in tricubic form
	#	gridRotMat = rotation matrix of default cubic grid
	#	rotMat = array with the transformation matrices of the joint and its parent
	#	thickness = thickness measure correlated with joint spacing
	# ======================================== #

	# define initual guess condition

	initial_guess = np.zeros(3)

	# create 3D arrays for matrix multiplication

	paramCoords = np.eye(4)

	# create tuple for arguments passed to both constraints and cost functions

	arguments = (proxArr, distArr, ipProx, ipDist, rotMat, gridRotMat, thickness, paramCoords)

	# set constraints functions

	cons = ({'type': 'ineq',	# set type to inequality, which means that it is to be non-negative
		 'fun': cons_fun,	# set constraint function
		 'args': arguments})	# pass arguments to constrain function

	# set bounds

	boundsList = [(-10,10)] * (len(initial_guess)) # set x, y and z coordinate boundaries
	bnds = tuple(boundsList)

	# set options

	options = {"maxiter": 50} # if it doesn't solve within 25 iterations it usually won't solve. Any more than 50 will just increase computational time without much benefit

	# optimization using SLSQP

	res = sp.optimize.minimize(cost_fun, initial_guess, args = arguments, bounds = bnds, method = 'SLSQP', constraints = cons, options = options)

	# get results: res.X = joint centre point coordinates, res.fun = mean articular distance - thickness squared + variance

	return res


# ========== signed distance field constraint function ==========

def cons_fun(params, proxArr, distArr, ipProx, ipDist, rotMat, gridRotMat, thickness, paramCoords):

	# Input variables:
	#	params = array of X, Y and Z coordinates of distal element position
	#	proxArr = 3D array of proximal articular surface vertex transformation matrices for fast computation of relative coordinates 
	#	distArr = 3D array of distal articular surface vertex transformation matrices for fast computation of relative coordinates 
	#	ipProx = tricubic interpolation function from tricubic.tricubic() for the signed distance data on the proximal cubic grid
	#	ipDist = tricubic interpolation function from tricubic.tricubic() for the signed distance data on the distal cubic grid
	#	rotMat = array with the transformation matrices of the joint and its parent
	#	gridRotMat = rotation matrix of default cubic grid coordinate system
	#	thickness = thickness measure correlated with joint spacing. Passed through args, not part of this constraint function
	#	paramCoords = 4x4 identity matrix for matrix multiplications
	# ======================================== #

	# extract coordinates from paramsand transform them into transformation matrix coords

	paramCoords[3,0:3] = params

	# get transformation matrix

	transMat = rotMat[1] # transformation matrix
	transMat[3,0:3] = np.dot(paramCoords,rotMat[2])[3,0:3] # append result world space coordinates to transformation matrix
	transMatInv = np.linalg.inv(transMat) # get inverse of transformation matrix

	# calculate position of vertices relative to cubic grid

	proxVtcRelArr = np.dot(proxArr,np.dot(np.dot(rotMat[0],transMatInv),gridRotMat))[:,3,0:3].tolist()
	distVtcRelArr = np.dot(distArr,np.dot(np.dot(transMat,rotMat[3]),gridRotMat))[:,3,0:3].tolist()

	# go through each proximal and distal articular surface point and check if any of them intersect with a mesh (i.e., signDist < 0)

	signDist = [min([ipDist.ip(vtx) for vtx in proxVtcRelArr]), # minimal proximal signed distance
		    min([ipProx.ip(vtx) for vtx in distVtcRelArr])] # minimal distal signed distance

	return min(signDist) # return minimal value, if any of the points are inside a mesh it will be negative


# ========== cost function for optimisation ==========

def cost_fun(params, proxArr, distArr, ipProx, ipDist, rotMat, gridRotMat, thickness, paramCoords):

	# Input variables:
	#	params = array of X, Y and Z coordinates of distal element position
	#	proxArr = 3D array of proximal articular surface vertex transformation matrices for fast computation of relative coordinates 
	#	distArr = 3D array of distal articular surface vertex transformation matrices for fast computation of relative coordinates. Passed through args, not part of cost function
	#	ipProx = tricubic interpolation function from tricubic.tricubic() for the signed distance data on the proximal cubic grid. Passed through args, not part of cost function
	#	ipDist = tricubic interpolation function from tricubic.tricubic() for the signed distance data on the distal cubic grid
	#	rotMat = array with the transformation matrices of the joint and its parent
	#	gridRotMat = rotation matrix of default cubic grid coordinate system
	#	thickness = thickness measure correlated with joint spacing
	#	paramCoords = 4x4 identity matrix for matrix multiplications
	# ======================================== #
	
	# extract coordinates from params and transform them into transformation matrix coords

	paramCoords[3,0:3] = params

	# get transformation matrix

	transMat = rotMat[1] # transformation matrix
	transMat[3,0:3] = np.dot(paramCoords,rotMat[2])[3,0:3] # append result world space coordinates to transformation matrix
	transMatInv = np.linalg.inv(transMat) # get inverse of transformation matrix

	# calculate position of vertices relative to cubic grid

	vtcRelArr = np.dot(proxArr,np.dot(np.dot(rotMat[0],transMatInv),gridRotMat))[:,3,0:3].tolist()

	# calculate distance 

	signDist = [ipDist.ip(vtx) for vtx in vtcRelArr]
	meanDist = sum(signDist)/len(signDist)

	# calculate variance

	var = sum([(dist - meanDist) ** 2 for dist in signDist]) / len(signDist)
		
	return (meanDist - thickness) ** 2 + var # minimising variance enforces largest possible overlap


