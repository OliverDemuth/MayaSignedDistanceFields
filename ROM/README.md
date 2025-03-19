## How to run the ROM simulations

### Prepare the Maya scene

1. Several sets of objects are required:

   
   (1) ``congruencyMeshes`` - representing the articular surfaces (e.g., ``['prox_art_surf', 'dist_art_surf']``)
   
   (2) ``meshes`` - representing the bones themselves (e.g., ``['prox_mesh', 'dist_mesh']``)
   
   (3) ``fittedShape`` - representing the fitted shape to the proximal articular surfaces (which is a proxy for the target joint spacing; e.g., ``'prox_ftted_sphere'``)

   (4) ``jointName`` - representing the joint (e.g., a joint called ``'myJoint'`` if following [Manafzadeh & Padian, 2018](https://doi.org/10.1098/rspb.2018.0727)). Note that an attribute called *``viable``* in ``jointName`` is no longer required

2. Meshes representing the bones (i.e., ``meshes``) need realtively uniform face areas, otherwise large faces might skew vertex normals towards their direction. It is, therefore, important to extrude edges around large faces prior to closing the hole at edges with otherwise acute angles to circumvent this issue (e.g., if meshes have been cut to reduce polycount) prior to executing the Python scripts.

### Execute the scripts
#### From within Maya

1. Copy and paste the *ROMmapper.py* script into the *Python Script Editor* and execute it
2. Copy and paste the *runROMmapper.py* script into the *Python Script Editor*
3. Adjust the user defined variables in the script and execute it

The optimised translations and the correspondng rotations for each viable pose will be keyed into the translation and rotation attributes of 'jointName' for each viable frame. Only viable poses will be keyed. Maya will become unresponsive until the ROM simulation is done. A progress bar will be updated according to the progress made. The ROM simulations can be cancelled at any time by pressing *esc* without losing any progress.

#### From the command line (terminal) 
Autodesk Maya does not need to be open.
1. Create the following exemplary folder structure and change the directories in the *ROMmapperWrapper.py* script
```
path = '/your/file/path/ROM/python' 
fileDir = '/your/file/path/ROM/maya files'
outDir = '/your/file/path/ROM/results' 
```
2. Copy the following scripts into the python folder according to the *path* specified in the *ROMmapperWrapper.py* script as above
```
ROMmapperBatch.py
ROMmapperWrapper.py
```
3. Adjust the user defined variables in *ROMmapperWrapper.py* and save it
4. Execute the script as follows:

For **Windows** in the command prompt execute the following
```
cd C:\Program Files\Autodesk\Maya<VersionNumber>\bin\
mayapy /your/file/path/ROM/python/ROMmapperWrapper.py
```
For **macOS** in the terminal execute the following
```
cd /Applications/Autodesk/maya<VersionNumber>/Maya.app/Contents/bin/
./mayapy /your/file/path/ROM/python/ROMmapperWrapper.py
```
The optimised translations and the correspondng rotations for each viable pose will be saved as .csv files in the results folder, named according to the respective Maya file (.mb) in the Maya files folder. The ROM simulations cannot be safely aborted and can only be cancelled by closing the command prompt/terminal, however, all progress will be lost.

###### The Python scripts have been written in Python 3 and were tested with Python 3.11 and Autodesk Maya 2025
