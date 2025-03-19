# Implementation of signed distance field based simulations in Autodesk Maya

Automated estimation of (1) joint range of motion via optimisation of joint translations (2) 3D ligament paths wrapping around bone meshes using signed distance fields (SDF). These are implementations of the [Marai et al., 2004](https://doi.org/10.1109/TBME.2004.826606) and [Marai et al., 2006](https://doi.org/10.1109/IEMBS.2006.259742) approaches for Autodesk Maya. The scripts create SDFs of the bone meshes which are then used for subsequent simulations.

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
ligaments/
  ligamentCalculation.py  
  runLigamentCalculation.py  
  ligamentCalculationBatch.py
  ligamentCalculationWrapper.py

ROM/
  ROMmapper.py
  runROMmapper.py
  ROMmapperBatch.py
  ROMmapperWrapper.py
```

###### The Python scripts have been written in Python 3 and were tested with Python 3.11 and Autodesk Maya 2025
