<p align="center"> <img src="anim3d.gif"/> </p>

# AeroBenchVVPython Overview
This project is developed on top of the Aerobench framework and is a non-formal verification of a novel AES and GCAS on F-16 platform. For verification, the initial state space is randomly sampled and simulations are run based on those random initial states. 

# Citation

For citation purposes, please use: "Verification Challenges in F-16 Ground Collision Avoidance and Other Automated Maneuvers", P. Heidlauf, A. Collins, M. Bolender, S. Bak, 5th International Workshop on Applied Verification for Continuous and Hybrid Systems (ARCH 2018)

# Required Libraries 

The following Python libraries are required (can be installed using `sudo pip install <library>`):

`numpy` - for matrix operations

`scipy` - for simulation / numerical integration (RK45) and trim condition optimization 

`matplotlib` - for animation / plotting (requires `ffmpeg` for .mp4 output or `imagemagick` for .gif)

`slycot` - for control design (not needed for simulation)

`control` - for control design (not needed for simulation)

## Code Execution
Ensure you care currently within the `aes` (assuming you start off within the `F-16_Model_Verification` folder, run `cd code/aerobench/examples/aes`)

To run a single simulation for specified initial conditons use the follow command (you can change the initial states within `run_AES.py`): <br />
<br />
```python run_AES.py <animation save file name.gif>```

To run verfication through random sampling of initial states use the following command (you can change the number of simulatins `n`, the other initial states, and the file that the results are saved to within `test_verif_AES.py`): <br />
<br />
```python test_verif_AES.py```


### Animation isuses
Use matplotlib version 3.1.1 if you get errors like: 
"art3d.py", line 175, in set_3d_properties
    zs = np.broadcast_to(zs, xs.shape)
AttributeError: 'list' object has no attribute 'shape'

### Release Documentation
Distribution A: Approved for Public Release (88ABW-2020-2188) (changes in this version)
    
Distribution A: Approved for Public Release (88ABW-2017-6379) (v1)
