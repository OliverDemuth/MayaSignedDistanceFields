# Compute Ligament Length

## How to run the scripts

#### 1. Make sure to have the PyMEL package installed for Maya

For **Windows** in the command promp execute the following 
```
cd C:\Program Files\Autodesk\Maya<VersionNumber>\bin
mayapy -m pip install "pymel>=1.3.,<1.4."
```
For **macOS** in the terminal execute the following
```
export PATH=$PATH:/Applications/Autodesk/maya<VersionNumber>/Maya.app/Contents/bin
sudo ./mayapy -m pip install "pymel>=1.3.,<1.4."
```
For more information see [here](https://knowledge.autodesk.com/support/maya/learn-explore/caas/CloudHelp/cloudhelp/2023/ENU/Maya-Scripting/files/GUID-2AA5EFCE-53B1-46A0-8E43-4CD0B2C72FB4-htm.html)

#### 2. Run first script
Copy and Paste *ligLength.py* script into the *Python Script Editor* and execute it

#### 3. Run second script
Copy and Paste *ligLengthExpression.py* into the *Python Script Editor*, adjust the user defined variables and execute it

###### The PyMEL/Python scripts have been written in Python 3 and were tested in Maya 2023
