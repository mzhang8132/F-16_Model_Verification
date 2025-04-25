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



def main():
    'main function'

    ### Initial Conditions ###
    power = 9 # engine power level (0-10)

    # Default alpha & beta
    alpha = deg2rad(2.1215) # Trim Angle of Attack (rad)
    beta = 0                # Side slip angle (rad)

    # Initial Attitude
    alt = 10200        # altitude (ft)
    vt = 2040          # initial velocity (ft/sec)
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
    aes.waiting_time = 2.2
    gcas.waiting_cmd[1] = 2.2 # ps command
    aes.waiting_cmd[1] = 2.2 # ps command

    # custom gains
    aes.cfg_k_prop = 1.4
    aes.cfg_k_der = 0
    aes.cfg_eps_p = deg2rad(20)
    aes.cfg_eps_phi = deg2rad(15)

    step = 1/30
    
    try:
        res = run_f16_sim(init, tmax, gcas, step=step, extended_states=True, integrator_str='rk45')
        
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
        
        verifier.verify(res)
    except AssertionError as error:
        print("GCAS failed to recover")
        
        res = modified_run_f16_sim(init, tmax, aes, step=step, extended_states=True, integrator_str='rk45')
        
        try:
            safety_limits = SafetyLimits( \
            Nz=(-2, 9), #G's \
            v=(300, 2500), # ft/s \
            alpha=(-10, 45), # deg \
            betaMaxDeg=30,# deg
            psMaxAccelDeg=500) # deg/s/s
        
            verifier = SafetyLimitsVerifier(safety_limits, aes.llc)
            
            verifier.verify(res)
            
            if len(sys.argv) > 1 and (sys.argv[1].endswith('.mp4') or sys.argv[1].endswith('.gif')):
                filename = sys.argv[1]
                print(f"saving result to '{filename}'")
            else:
                filename = ''
                print("Plotting to the screen. To save a video, pass a command-line argument ending with '.mp4' or '.gif'.")

            plot.plot_attitude(res, figsize=(12, 10))
            plt.savefig('gcas_attitude.png')
            plt.close()
            
            plot.plot_single(res, 'alt', title='Altitude (ft)')
            alt_filename = 'gcas_altitude.png'
            plt.savefig(alt_filename)
            print(f"Made {alt_filename}")
            plt.close()
            
            plot.plot_single(res, 'vt', title='Speed (ft/s)')
            alt_filename = 'gcas_velocity.png'
            plt.savefig(alt_filename)
            print(f"Made {alt_filename}")
            plt.close()
            
            anim3d.make_anim(res, filename, elev=15, azim=-150)
        except AssertionError as error:
            print("AES broke the plane")
            print(error)
            
        try:
            safety_limits = SafetyLimits( \
            altitude=(0, 45000)) #ft 
        
            verifier = SafetyLimitsVerifier(safety_limits, aes.llc)
            
            verifier.verify(res)
            
            print("Successfully eject pilot")
            
            print(f"Simulation Completed in {round(res['runtime'], 3)} seconds")
    
            if len(sys.argv) > 1 and (sys.argv[1].endswith('.mp4') or sys.argv[1].endswith('.gif')):
                filename = sys.argv[1]
                print(f"saving result to '{filename}'")
            else:
                filename = ''
                print("Plotting to the screen. To save a video, pass a command-line argument ending with '.mp4' or '.gif'.")

            plot.plot_attitude(res, figsize=(12, 10))
            plt.savefig('aes_attitude.png')
            plt.close()
            
            plot.plot_single(res, 'alt', title='Altitude (ft)')
            alt_filename = 'aes_altitude.png'
            plt.savefig(alt_filename)
            print(f"Made {alt_filename}")
            plt.close()
            
            plot.plot_single(res, 'vt', title='Speed (ft/s)')
            alt_filename = 'aes_velocity.png'
            plt.savefig(alt_filename)
            print(f"Made {alt_filename}")
            plt.close()
            

            anim3d.make_anim(res, filename, elev=15, azim=-150)
        except AssertionError as error:
            print("AES failed")
            
        
    

if __name__ == '__main__':
    main()
