import numpy as np
from math import sin, cos

class Controller3D():
    """
    This class computes the commanded thrusts (N) to be applied to the quadrotor plant.

    You are to implement the "compute_commands" method.
    """
    def __init__(self, hbparams, pid_gains, dt=0):
        """
        Inputs:
        - hbparams (HBarams dataclass):             model parameter class for the drone
        - pid_gains (dict):                         pid gains

        N.B. pid_gains is a dictionary structure where the keys are 'kp_x', 'kd_z', etc.
        """
        self.params = hbparams

        # set control gains here



    def compute_commands(self, setpoint, state):
        """
        Inputs:
        - setpoint (Waypoint):      the desired control setpoint
        - state (State):            the current state of the system
        Returns:
        - U (np.array):     array of control inputs {u1-u4}

        """
        U = np.array([0.,0.,0.,0.])

        # your code here

        

        return U