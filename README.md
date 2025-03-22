<p align="center"> <img src="gcas_py.gif"/> </p>

# AeroBenchVVPython Overview
This project is based on the F-16 model and GCAS (ground collision avoidance system) verification benchmark created in AeroBenchVV. On top of GCAS, this project introduces a novel version of AES (automatic ejection system) on the F-16 model. Currently the AES system is only available in VTOL aircrafts like the F-35B, but the AES we are proposing will allow the system to automatically eject the pilot if GCAS is unable to prevent a collision with the ground.

# Citation

For citation purposes, please use: "Verification Challenges in F-16 Ground Collision Avoidance and Other Automated Maneuvers", P. Heidlauf, A. Collins, M. Bolender, S. Bak, 5th International Workshop on Applied Verification for Continuous and Hybrid Systems (ARCH 2018)

# Required Libraries 

The following Python libraries are required (can be installed using `sudo pip install <library>`):

`numpy` - for matrix operations

`scipy` - for simulation / numerical integration (RK45) and trim condition optimization 

`matplotlib` - for animation / plotting (requires `ffmpeg` for .mp4 output or `imagemagick` for .gif)

`slycot` - for control design (not needed for simulation)

`control` - for control design (not needed for simulation)
