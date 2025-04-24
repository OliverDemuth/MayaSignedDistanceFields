# Implementation of signed distance field based simulations in Autodesk Maya

## ROM simulation
Automated estimation of joint range of motion via optimisation of joint translations using signed distance fields (SDFs). The scripts optimise contact-based positions across a given set of rotational poses for a set of bone meshes. This is an implementation of the [Marai et al., 2006](https://doi.org/10.1109/IEMBS.2006.259742) and [Lee et al. 2023](https://doi.org/10.1098/rspb.2023.1446) approach for Autodesk Maya. It creates SDFs for the proximal and distal bone meshes which are then used to calculate intersections between the meshes and the distance between the bones. The (mobile) joint centre position is estimated, and the distal bone mesh is moved into position for a set of prescribed joint orientations. Only feasible joint position are be keyed/exported. The resulting set of frames represent viable joint positions and orientations.

The multiterm objective function $C_{ROM}$ to optimise the translational joint position for each given joint orientation was implemented as follows:

$$C_{ROM}=\underbrace{\left(\frac{1}{n_v} \sum_{v=1}^{n_v} d_v -d_t \right)^2}\_{\text{joint proximity goal}} + \underbrace{\frac{1}{n_v} \sum_{v=1}^{n_v} \left(d_v - \bar{d} \right)^2}\_{\text{joint congruency goal}} $$

subject to the inequality constraint imposed by the SDFs 

$$f(x_v,y_v,z_v ) \geq 0$$

where $d_v$ is the calculated signed distance for vertex $v$, $n_v$ is the number of vertices of the articular surfaces, $d_t$ is the target joint proximity, $\bar{d}$ is the mean joint proximity and $x$, $y$ and $z$ are the Cartesian 3D coordinates of the sample point. 

Only joint poses where the average intraarticular distance did not exceed the target joint proximity by more than 7% were deemed viable, as the joints would otherwise have been likely disarticulated. The target joint proximity was determined as half the radius of the sphere fitted to the glenoid articular surfaces (which was a close match to the measured average intra-articular distances from the CT scans, yet does not rely on articulated remains), and represents a joint overlap (sensu [Bishop et al. 2023](https://doi.org/10.1111/2041-210X.14016)) of  approximately 0.5. 

Information about how to run the ROM simulations can be found [here](https://github.com/OliverDemuth/MayaSignedDistanceFields/tree/main/ROM).

## Ligament simulation and length calculation
Automated estimation of 3D ligament path wrapping around bone meshes using SDFs.
The scripts calculate the shortest distance of a ligament from origin to insertion wrapping around the bone meshes. This is an implementation of the [Marai et al., 2004](https://doi.org/10.1109/TBME.2004.826606) approach for Autodesk Maya. It creates SDFs for the proximal and distal bone meshes, which are then used to approximate the 3D path of each ligament across them to calculate their shortest path length from origin to insertion while preventing penetration of the bones.

The ligament path was formulated as the following optimisation problem: Find the coordinates of the n-1 points between $p_0$ and $p_n$, so that the Euclidean distance of the path along $p_0,p_1,p_2,\dotsc,p_n$ is minimal while the distance between the path points and the bony obstacles is non-negative. In the initial guess the points were equally spaced along the X-axis (i.e., their distance was constant, and their $y$ and $z$ values were set to 0). Thus the length of the shortest path could be approximated by minimising its Euclidean distance only over the $y$ and $z$ coordinates for each point, which was implemented as the following cost function:

$$C_{lig} = \sum_{i=0}^{n-1} \sqrt{const^2+(y_{i+1}-y_i )^2+(z_{i+1}-z_i )^2 }$$

subject to the inequality constraint imposed by the SDFs

$$f(x_v,y_v,z_v ) \geq 0$$

and the additional inequality constraint to ensure the smoothness of the optimised ligament paths

$$\tan^{-1}\left(\frac{\sqrt{\max\limits_i⁡ \lbrace (y_{i+1}-y_i )^2+(z_{i+1}-z_i )^2 \rbrace }}{const}\right) \leq  \frac{π}{3}$$

where $x_{+1}-x_i =  {}^1/_n = const$ and $i = 0,\dotsc,n-1$. This prevents the ligaments from abruptly changing direction or intersecting the bone meshes between two path points. 

Information about how to run the ligament simulations can be found [here](https://github.com/OliverDemuth/MayaSignedDistanceFields/tree/main/ligaments).

## Installation 
#### Make sure to have the required Python modules installed for Autodesk Maya

[NumPy](https://numpy.org/) (1.24.4)  
[SciPy](https://scipy.org/) (1.15.0)   
[pytricubic](https://github.com/danielguterding/pytricubic) (1.0.4)  

For **Windows** in the command prompt execute the following 
```
cd C:\Program Files\Autodesk\Maya<VersionNumber>\bin
mayapy -m pip install numpy
mayapy -m pip install scipy
mayapy -m pip install tricubic
```
For **macOS** in the terminal execute the following
```
cd /Applications/Autodesk/maya<VersionNumber>/Maya.app/Contents/bin
sudo ./mayapy -m pip install numpy
sudo ./mayapy -m pip install scipy
sudo ./mayapy -m pip install tricubic
```
For more information about managing Python packages with mayapy and pip see [here](https://help.autodesk.com/view/MAYAUL/2025/ENU/?guid=GUID-72A245EC-CDB4-46AB-BEE0-4BBBF9791627).

**Note**, installing *pytricubic* requires *CMake*. For more information see [here](https://github.com/danielguterding/pytricubic).

#### Download the following folders/Python scripts from this repository and save them in a folder on your PC/Mac
```
ROM/
  ROMmapper.py
  runROMmapper.py
  ROMmapperBatch.py
  ROMmapperWrapper.py

ligaments/
  ligamentCalculation.py  
  runLigamentCalculation.py  
  ligamentCalculationBatch.py
  ligamentCalculationWrapper.py
```
###### The Python scripts were written in Python 3 and tested with Python 3.11 and Autodesk Maya 2025
