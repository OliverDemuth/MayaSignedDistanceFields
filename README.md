# Compute Ligament Length

## How to run the scripts

1. We recommend create your own virtual enviornment to run the experiments using conda (cheat sheet available [here](https://docs.conda.io/projects/conda/en/4.6.0/_downloads/52a95608c49671267e40c689e0bc00ca/conda-cheatsheet.pdf) )
```
conda create  --name ligLengthEnv
conda activate ligLengthEnv
```
2. Clone the project
```
git clone https://github.com/OliverDemuth/LigamentLength.git
cd LigamentLength
```
4. As explained in [PyMEL](https://github.com/LumaPictures/pymel) we need to execute
```
export PATH=$PATH:/Applications/Autodesk/maya202*/Maya.app/Contents/bin (replace the star with the Maya version installed)
curl https://bootstrap.pypa.io/get-pip.py | mayapy
mayapy -m pip install --pre pymel
mayapy -m pip install numpy
```
4. execute the script
```
mayapy main.py
```
