import math

import numpy as np
from numpy import deg2rad

from aerobench.highlevel.autopilot import Autopilot
from aerobench.util import StateIndex
from aerobench.lowlevel.low_level_controller import LowLevelController



class AESAutopilot(Autopilot):

    def __init__(self, init_mode='standby', gain_str='old', stdout=False):
        assert init_mode in ['standby', 'recover', 'eject']

        self.cfg_eps_phi = deg2rad(10)  # max roll angle tolerance for level
        self.cfg_eps_pitch = deg2rad(10)  # max pitch angle tolerance for level

        self.cfg_nz_des = 5

        self.cfg_alt_bounds = (0, 55000)  # altitude bounds: 0 to 55,000 ft
        self.cfg_vt_bounds = (0, 1012.69)  # airspeed bounds: 0 to 1,012.69 ft/s

        self.stdout = stdout

        llc = LowLevelController(gain_str=gain_str)
        Autopilot.__init__(self, init_mode, llc=llc)

    def log(self, s):
        if self.stdout:
            print(s)


    def advance_discrete_mode(self, t, x_f16):
        premode = self.mode

        ## AES controller state machine
        if self.mode == 'standby':
            if self.make_plane_safe_for_ejection(x_f16):
                self.mode = 'recover'
        elif self.mode == 'recover':
            if self.can_eject(x_f16):
                self.mode = 'eject'
        elif self.mode == 'eject':
            pass  # stay in eject once committed

        changed = premode != self.mode

        if changed:
            self.log(f"AES transition {premode} -> {self.mode} at time {t}")

        return changed



    def make_plane_safe_for_ejection(self, x_f16):
        
        ## TODO somehow make plane level and slow if it isnt meeting constraints?
        ## i think there is something for throttle
        ## i copy pasted pull_nose_level and roll_wings_level from gcas to make the plane flat so that should cover that part?
        
        return
    def can_eject(self, x_f16):
        alt = x_f16[StateIndex.ALT]
        vt = x_f16[StateIndex.VT]
        
        ## plane needs to be level and invariants  cant be broken.
        if not self.is_plane_flat(x_f16):
            return False  
        
        if alt < self.cfg_alt_bounds[0] or alt > self.cfg_alt_bounds[1] or vt < self.cfg_vt_bounds[0] or vt > self.cfg_vt_bounds[1]:
            return False  

        return True

    def is_plane_flat(self, x_f16):
        phi = x_f16[StateIndex.PHI]  # Roll angle
        theta = x_f16[StateIndex.THETA]  # Pitch angle
        p = x_f16[StateIndex.P]  # Roll rate

        ## check if plane is level
        wings_level = abs(phi) < self.cfg_eps_phi
        low_roll_rate = abs(p) < self.cfg_eps_pitch  
        flat_pitch = abs(theta) < self.cfg_eps_pitch  

        return wings_level and low_roll_rate and flat_pitch




    def pull_nose_level(self):
        'get commands in mode PULL'

        rv = np.zeros(4)

        rv[0] = self.cfg_nz_des

        return rv

    def roll_wings_level(self, x_f16):
        'get commands in mode ROLL'

        phi = x_f16[StateIndex.PHI]
        p = x_f16[StateIndex.P]

        rv = np.zeros(4)

        # Determine which angle is "level" (0, 360, 720, etc)
        radsFromWingsLevel = round(phi / (2 * math.pi))

        # PD Control until phi == pi * radsFromWingsLevel
        ps = -(phi - (2 * math.pi) * radsFromWingsLevel) * self.cfg_k_prop - p * self.cfg_k_der

        # Build commands to roll wings level
        rv[1] = ps

        return rv




if __name__ == '__main__':
    print("Run this with a simulation script like run_AES.py")