import math
import sys

from numpy import deg2rad

import matplotlib.pyplot as plt

import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from aerobench.visualize import anim3d, plot

from aerobench.modified_run_f16_sim import modified_run_f16_sim
from aerobench.run_f16_sim import run_f16_sim

from aerobench.util import SafetyLimits, SafetyLimitsVerifier

from aerobench.examples.aes.aes_autopilot import AESAutopilot
from aerobench.examples.gcas.gcas_autopilot import GcasAutopilot

import numpy as np
import pandas as pd

from tqdm import trange

def main():
    'main function'

    ### Initial Conditions ###
    power = 9 # engine power level (0-10)

    # Default alpha & beta
    alpha = deg2rad(2.1215) # Trim Angle of Attack (rad)
    beta = 0                # Side slip angle (rad)

    low_speed = range(300, 800)
    medium_speed = range(800, 1500)
    high_speed = range(1500, 2000)
    low_alt = range(500, 1000)
    medium_alt = range(1000, 30000)
    high_alt = range(30000, 45000)

    n = 100

    test_results = {}
    test_results = pd.DataFrame(test_results)

    combos = {"Test1": (low_speed, low_alt), "Test2": (low_speed, medium_alt), "Test3": (low_speed, high_alt),
              "Test4": (medium_speed, low_alt), "Test5": (medium_speed, medium_alt), "Test6": (medium_speed, high_alt),
              "Test7": (high_speed, low_alt), "Test8": (high_speed, medium_alt), "Test9": (high_speed, high_alt)}

    for test in combos.keys():
        speed = combos[test][0]
        height = combos[test][1]
        
        gcas_results = np.zeros(n)
        aes_results = np.zeros(n)
        
        np.random.seed(16)
        
        print(test)
        for i in trange(n):
            # Initial Attitude
            alt = height[np.random.randint(len(height))]       # altitude (ft)
            vt = speed[np.random.randint(len(speed))]          # initial velocity (ft/sec)
            phi = 0           # Roll angle from wings level (rad)
            theta = (-math.pi/2)*0.7         # Pitch angle from nose level (rad)
            psi = 0.8 * math.pi   # Yaw angle from North (rad)

            # Build Initial Condition Vectors
            # state = [vt, alpha, beta, phi, theta, psi, P, Q, R, pn, pe, h, pow]
            init = [vt, alpha, beta, phi, theta, psi, 0, 0, 0, 0, 0, alt, power]
            tmax = 15 # simulation time

            gcas = GcasAutopilot(init_mode='waiting', stdout=True)
            aes = AESAutopilot(init_mode='waiting', stdout=True)

            gcas.waiting_time = 2.2
            aes.waiting_time = 1
            gcas.waiting_cmd[1] = 2.2 # ps command
            aes.waiting_cmd[1] = 1 # ps command

            # custom gains
            aes.cfg_k_prop = 1.4
            aes.cfg_k_der = 0
            aes.cfg_eps_p = deg2rad(20)
            aes.cfg_eps_phi = deg2rad(15)

            step = 1/30
            
            try:
                res_gcas = run_f16_sim(init, tmax, gcas, step=step, extended_states=True, integrator_str='rk45')
                
                # Determine whether the GCAS system kept the plane in a safe state
                # the entire time.
                safety_limits = SafetyLimits( \
                    altitude=(0, 45000), #ft \
                    Nz=(-2, 9), #G's \
                    v=(300, 2500), # ft/s \
                    alpha=(-10, 45), # deg \
                    betaMaxDeg=30,# deg
                    psMaxAccelDeg=500) # deg/s/s
                
                verifier = SafetyLimitsVerifier(safety_limits, gcas.llc)
                
                verifier.verify(res_gcas)
                
                gcas_results[i] = 1
            except BaseException:  
                pass
            
            if gcas_results[i] == 0:  
                try:
                    res_aes = modified_run_f16_sim(init, tmax, aes, step=step, extended_states=True, integrator_str='rk45')
                    
                    safety_limits = SafetyLimits( \
                    Nz=(-2, 9), #G's \
                    v=(300, 2500), # ft/s \
                    alpha=(-10, 45), # deg \
                    betaMaxDeg=30,# deg
                    psMaxAccelDeg=500) # deg/s/s
                
                    verifier = SafetyLimitsVerifier(safety_limits, aes.llc)
                    
                    verifier.verify(res_aes)
                except BaseException:
                    pass 
                try:
                    safety_limits = SafetyLimits( \
                    altitude=(0, 45000)) #ft 
                
                    verifier = SafetyLimitsVerifier(safety_limits, aes.llc)
                    
                    verifier.verify(res_aes)
                    
                    aes_results[i] = 1
                except BaseException:
                    pass
        
        test_results[test+" GCAS"] = gcas_results
        test_results[test+" AES"] = aes_results
        
    test_results.to_csv("test_results.csv")
    

if __name__ == '__main__':
    main()
