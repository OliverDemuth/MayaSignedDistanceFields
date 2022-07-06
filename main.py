import numpy as np

import pymel.core as pm
import pymel.core.datatypes as dt

from pymel.core.system import openFile


def ligLength(origin, insertion, jointCentre, proxMesh, distMesh, resolution=None):

    # Input variables:
    #   origin = name of ligament origin (represented by object, e.g. locator, in Maya scene)
    #   insertion = name of ligament insertion (represented by object, e.g. locator, in Maya scene)
    #   jointCentre = name of joint centre (represented by object, e.g. joint or locator, in Maya scene)
    #   proxMesh = name of proximal bone mesh to which ligament origin attaches
    #   distMesh = name of distal bone mesh to which ligament insertion attaches
    #   res = resolution for mesh slice approximation
    # ======================================== #

    global counter
    counter = 0

    # data house keeping
    duplicate_prox_name = proxMesh + '_duplicate'
    pm.duplicate(proxMesh, n=duplicate_prox_name)

    duplicate_dist_name = distMesh + '_duplicate'
    pm.duplicate(distMesh, n=duplicate_dist_name)

    # get positions of points of interest

    oPos = np.array(pm.xform(origin,
        query=True, worldSpace=True, translation=True))
    iPos = np.array(pm.xform(insertion,
        query=True, worldSpace=True, translation=True))
    jPos = np.array(pm.xform(jointCentre,
        query=True, worldSpace=True, translation=True))

    # calculate vectors from origin to insertion

    LigDir = oPos - iPos
    JointDir = oPos - jPos
    Offset = np.linalg.norm(oPos - iPos)

    # calculate cut plane direction

    uCross = np.cross(LigDir, JointDir)
    uCross = uCross / np.linalg.norm(uCross)

    vCross = np.cross(LigDir, uCross)
    vCross = vCross / np.linalg.norm(vCross)

    cutAim = pm.angleBetween(euler=True,
        v1=(0.0, 0.0, 1.0),
        v2=(uCross[0], uCross[1], uCross[2]))

    # normalize vectors by distance between origin and insertion

    LigDirNorm = LigDir / np.linalg.norm(LigDir) / Offset
    vCrossNorm = vCross / Offset

    print("LigDirNorm", LigDirNorm)
    print("vCrossNorm", vCrossNorm)

    # get obstacle slices, their vertices and edges

    # VtxSet, EdgeSet = polyApproxSlice(oPos, cutAim, LigDirNorm, vCrossNorm,
    #     proxMesh, resolution)  # approximated slice of proximal bone mesh

    # VtxSet2, EdgeSet2 = polyApproxSlice(oPos, cutAim, LigDirNorm, vCrossNorm,
    #     distMesh, resolution)  # approximated slice of distal bone mesh


    # VtxSet.extend(VtxSet2)
    # EdgeSet.extend(EdgeSet2)

    # # run A* search

    # ligLength = astar(oPos, iPos, VtxSet, EdgeSet)
    # ligLength = (VtxSet,EdgeSet)

    # clean up

    if pm.objExists('TempGRP'):  # check if temporary group exists
        pm.delete('TempGRP')

    pm.delete(proxMesh, distMesh)
    pm.rename(duplicate_prox_name, proxMesh)
    pm.rename(duplicate_dist_name, distMesh)

    return ligLength


if __name__ == "__main__":
    # Load Scene
    scene = openFile('Scene/RLP3_lig_wrapping.mb',
        force=True, type='mayaBinary')

    ligLength(origin='LScH_orig', insertion='LScH_dorsalis_ins',
        jointCentre='myJoint', proxMesh='prox_mesh', distMesh='dist_mesh')
    print("DONE!")
