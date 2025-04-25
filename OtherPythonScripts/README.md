## Additional Python scripts
This folder contains additional Python scripts for data processing and analysis: 
### circumference.py
This scripts calculates the circumference of a 2D slice in Autodesk Maya based on a cutting plane using an implementation of the [Andrew (1979)](doi.org/10.1016/0020-0190(79)90072-3) monotone chain convex hull algorithm.
### collisionCheck.py
This script checks for the penetration/impingement of two meshes for a set of keyed joint poses using a signed distance field (SDF). It is equivalent to the Boolean method first proposed by [Manafzadeh & Padian (2018)](doi.org/10.1098/rspb.2018.0727), however, it is several orders of magnitude faster (<0.01 seconds per frame based on a target mesh with ~5000 vertices) due to the much faster interpolated scalar field representation of the proximal bone rather than relying on the computationally expensive 3D mesh. It no longer requires a Boolean object in Maya as the proximal mesh is represented as a SDF. Joint poses will be classed as viable (1; if no collision/impingement is detected) or inviable (0; if collision occurs) in a custom attribute called $'viable'$ at the animated joint object (e.g., $'myJoint'$ if following Manafzadeh & Padian 2018).

###### The Python scripts were written in Python 3 and tested with Python 3.11 and Autodesk Maya 2025
