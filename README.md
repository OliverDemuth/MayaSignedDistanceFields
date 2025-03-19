# Implementation of signed distance field based simulations in Autodesk Maya

### ROM simulations
Automated estimation of joint range of motion via optimisation of joint translations using signed distance fields (SDF). The scripts optimise contact-based positions across a given set of rotational poses for a set of bone meshes. It is an implementation of the [Marai et al., 2006](https://doi.org/10.1109/IEMBS.2006.259742) and [Lee et al. 2023](https://doi.org/10.1098/rspb.2023.1446) approach for Autodesk Maya. It creates SDFs for the proximal and distal bone meshes which are then used to caculate intersections between the meshes and the distance between them. The (mobile) joint centre position is estimated and the distal bone mesh is moved into position for a set of presribed joint orientations. Only feasible joint position are be keyed/exported. The resulting set of frames represent vialbe joint positions and orientations.

The following multiterm objective function $C$ to optimise the translational joint position for each given joint orientation was implemented

$$C=\left(\frac{1}{N_V} \sum_{v=1}^{N_V} d_v -d_t \right)^2 + \frac{1}{N_V} \sum_{v=1}^{N_V} \left(d_v - \bar{d} \right)^2 $$

subject to 

$$d_v= f(x_v,y_v,z_v )>0$$

where $d_v$ is the calculated signed distance for vertex $v$, $N_V$ is the number of vertices of the articular surface, $d_t$ is the target joint proximity and $\bar{d}$ is the mean joint proximity.

### Ligament calculations
Automated estimation of 3D ligament path wrapping around bone meshes using SDFs.
The scripts calculate the shortest distance of a ligament from origin to insertion wrapping around the bone meshes. It is an implementation of the [Marai et al., 2004](https://doi.org/10.1109/TBME.2004.826606) approach for Autodesk Maya. It creates SDFs for the proximal and distal bone meshes, which are then used to approximate the 3D path of each ligament accross them to calculate their shortest path length from origin to insertion while preventing penetration of the bones.

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
For more information about managing Python packages with mayapy and pip see [here](https://help.autodesk.com/view/MAYAUL/2025/ENU/?guid=GUID-72A245EC-CDB4-46AB-BEE0-4BBBF9791627)

**Note**, installing *pytricubic* requires *CMake*. For more information see [here](https://github.com/danielguterding/pytricubic)

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

###### The Python scripts have been written in Python 3 and were tested with Python 3.11 and Autodesk Maya 2025
