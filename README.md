# Compute Ligament Length

Automated estimation of 3D ligaments wrapping around bone meshes. It is an implementation of the [Marai et al., 2004](https://doi.org/10.1109/TBME.2004.826606) approach for Autodesk Maya. The scripts create a signed distance field of the bone meshes for each ligament which is then used to approximate its 3D path and calculate its length. 

## 1. Installation 
#### Make sure to have the required Python modules installed for Autodesk Maya

For **Windows** in the command prompt execute the following 
```
cd C:\Program Files\Autodesk\Maya<VersionNumber>\bin
mayapy -m pip install scipy
mayapy -m pip install numpy
mayapy -m pip install pytricubic
```
For **macOS** in the terminal execute the following
```
cd /Applications/Autodesk/maya<VersionNumber>/Maya.app/Contents/bin
sudo ./mayapy -m pip install scipy
sudo ./mayapy -m pip install numpy
sudo ./mayapy -m pip install pytricubic
```
For more information about managing Python packages with mayapy and pip see [here](https://knowledge.autodesk.com/support/maya/learn-explore/caas/CloudHelp/cloudhelp/2023/ENU/Maya-Scripting/files/GUID-72A245EC-CDB4-46AB-BEE0-4BBBF9791627-htm.html)

**Note**, installing *pytricubic* requires *CMake*. For more information see [here](https://github.com/danielguterding/pytricubic)

#### Download the following scripts from this repository and save them in a folder on your PC/Mac
```
ligamentCalculation.py
runLigamentCalculation.py
ligamentCalculationWrapper.py
progressBar.py
```

## 2. How to run the ligament calculations

### Prepare the Maya scene

1. Create two *locators* for each ligament, i.e., the origin and insertion, position them on the meshes and parent them underneath the respective elements/joints.

2. For each ligament create a float attribute at 'jointName' and name it accordingly. Make sure that the naming convention for the ligament origins  and insertions is correct, i.e. the locators should be named 'ligament*_orig' and 'ligament*_ins' for an attribute in 'jointName' called 'ligament*'. Make sure to remove the 'viable' attribute from 'jointName' if previously followed [Manafzadeh & Padian, 2018](https://doi.org/10.1098/rspb.2018.0727) before executing the ligament calculations.

3. Meshes need realtively uniform face areas, otherwise large faces might skew vertex normals towards their direction. It is, therefore, important to extrude edges around large faces prior to closing the hole at edges with otherwise acute angles to circumvent this issue, e.g., if meshes have been cut to reduce polycount, prior to executing the Python scripts.

### Execute the scripts
#### From within Maya

1. Copy and paste the *ligamentCalculation.py* script into the *Python Script Editor* and execute it
2. Copy and paste the *runLigamentCalculation.py* script into the *Python Script Editor*
3. Adjust the user defined variables in the scirpt and execute it

The ligament lengths will be keyed into the attributes of 'jointName' for each frame. Maya will become unresponsive until the calculations are done. A progress bar will be updated according to the progress. The ligament length calculations can be cancelled at any time by pressing *esc* without losing any progress.

#### From the command line 
Autodesk Maya does not need to be open.
1. Create the following exemplary folder structure and change the directories in the *ligamentCalculationWrapper.py* script
```
path = '\\your\\file\\path\\ligaments\\python' 
fileDir = '/your/file/path/ligaments/maya files'
outDir = '/your/file/path/ligaments/results' 
```
2. Copy the following scripts into the folder according to the *path* specified in the *ligamentCalculationWrapper.py* script as above
```
ligamentCalculation.py
ligamentCalculationWrapper.py
progressBar.py
```
3. Adjust the user defined variables in *ligamentCalculationWrapper.py* and save it
4. Execute the script

For **Windows** in the command prompt execute the following
```
cd C:\Program Files\Autodesk\Maya<VersionNumber>\bin\
mayapy.exe "path/to/ligamentCalculationWrapper.py"
```
For **macOS** in the terminal execute the following
```
cd /Applications/Autodesk/maya<VersionNumber>/Maya.app/Contents/bin/
./mayapy "path/to/ligamentCalculationWrapper.py"
```
The ligament lengths will be saved as .csv files in the results folder. The ligament length calculations cannot directly be abborted unless the command prompt/terminal is closed, however, all progress will be lost.

###### The Python scripts have been written in Python 3 and were tested in Autodesk Maya 2023
