## How to run the ligament calculations

### Prepare the Maya scene

1. Several sets of objects are required:

   
   (1) ``jointName`` - representing the joint (e.g., a joint called ``'myJoint'`` if following [Manafzadeh & Padian, 2018](https://doi.org/10.1098/rspb.2018.0727)).
   
   (2) ``meshes`` - representing the bones (e.g., ``['prox_mesh', 'dist_mesh']``).
   
2. Create two locators for each ligament (i.e., for a ligament called *``lig*``* the origin should be named ``lig*_orig``and the insertion named ``lig*_ins``) and position them on the meshes accordingly and parent them underneath the respective elements/joints.

3. For each ligament create a float attribute at ``jointName`` and name it accordingly. Make sure that the naming convention for each ligament *``lig*``* and the locators representing their origins and insertions are correct (see above). Make sure to remove the *``viable``* attribute from ``jointName`` if previously followed [Manafzadeh & Padian, 2018](https://doi.org/10.1098/rspb.2018.0727) before executing the ligament calculations.

4. Meshes representing the bones (i.e., ``meshes``) need relatively uniform face areas, otherwise large faces might skew vertex normals towards their direction. It is, therefore, important to extrude edges around large faces prior to closing the hole at edges with otherwise acute angles to circumvent this issue (e.g., if the bone meshes have been cut to reduce polycount) prior to executing the Python scripts.

### Execute the scripts
#### From within Maya

1. Copy and paste the *ligamentCalculation.py* script into the *Python Script Editor* and execute it
2. Copy and paste the *runLigamentCalculation.py* script into the *Python Script Editor*
3. Adjust the user-defined variables in the script and execute it

The ligament lengths will be keyed into the attributes of ``jointName`` for each frame. Maya will become unresponsive until the calculations are done. A progress bar will be updated according to the progress made. The ligament length calculations can be cancelled at any time by pressing *Esc* without losing any progress.

#### From the command line (terminal) 
Autodesk Maya does not need to be open.
1. Create the following exemplary folder structure and change the directories in the *ligamentCalculationWrapper.py* script
```
path = '/your/file/path/ligaments/python' 
fileDir = '/your/file/path/ligaments/maya files'
outDir = '/your/file/path/ligaments/results' 
```
2. Copy the following scripts into the python folder according to the *path* specified in the *ligamentCalculationWrapper.py* script as above
```
ligamentCalculationBatch.py
ligamentCalculationWrapper.py
```
3. Adjust the user-defined variables in *ligamentCalculationWrapper.py* and save it
4. Execute the script as follows:

For **Windows** in the command prompt execute the following
```
cd C:\Program Files\Autodesk\Maya<VersionNumber>\bin\
mayapy /your/file/path/ligaments/python/ligamentCalculationWrapper.py
```
For **macOS** in the terminal execute the following
```
cd /Applications/Autodesk/maya<VersionNumber>/Maya.app/Contents/bin/
./mayapy /your/file/path/ligaments/python/ligamentCalculationWrapper.py
```
The ligament lengths will be saved as .csv files in the results folder, named according to the respective Maya file (.mb) in the Maya files folder. The ligament length calculations cannot be safely aborted and can only be cancelled by closing the command prompt/terminal, however, all progress will be lost.

###### The Python scripts were written in Python 3 and tested with Python 3.11 and Autodesk Maya 2025
