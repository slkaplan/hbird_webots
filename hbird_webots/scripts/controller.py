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

        self.kp_x = pid_gains["kp_x"]
        self.kp_y = pid_gains["kp_y"]
        self.kp_z = pid_gains["kp_z"]
        
        self.ki_x = pid_gains["ki_x"]
        self.ki_y = pid_gains["ki_y"]
        self.ki_z = pid_gains["ki_Z"]
        
        self.kd_x = pid_gains["kd_x"]
        self.kd_y = pid_gains["kd_y"]
        self.kd_z = pid_gains["kd_z"]

        # rotational 
        self.kp_phi = pid_gains["kp_phi"]
        self.kp_theta = pid_gains["kp_theta"]
        self.kp_psi = pid_gains["kp_psi"]
        
        self.ki_phi = pid_gains["ki_phi"]
        self.ki_theta = pid_gains["ki_theta"]
        self.ki_psi = pid_gains["ki_psi"]
        
        self.kd_p = pid_gains["kd_p"]
        self.kd_q = pid_gains["kd_q"]
        self.kd_r = pid_gains["kd_r"]


    def compute_commands(self, setpoint, state):
        """
        Inputs:
        - setpoint (Waypoint):      the desired control setpoint
        - state (State):            the current state of the system
        Returns:
        - U (np.array):     array of control inputs {u1-u4}

        """
        U = np.array([0.,0.,0.,0.])

        e = np.array([
            setpoint.position.z - state.position.z,
        ])

        e_dot = np.array([
            0 - state.velocity.z
        ])

        U[0] = self.params.mass * (self.kp_z * e[0] + self.kd_z * e_dot[0] + self.params.g)

        return U
