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
        self.ki_z = pid_gains["ki_z"]
        
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

        # Position controller (cascades into attitude controller)
        e_z = setpoint.position.z - state.position.z
        e_x = setpoint.position.x - state.position.x
        e_y = setpoint.position.y - state.position.y

        e_dot_z = 0 - state.velocity.z
        e_dot_x = 0 - state.velocity.x
        e_dot_y = 0 - state.velocity.y

        z_dotdot_c = self.kp_z * e_z + self.kd_z * e_dot_z
        x_dotdot_c = self.kp_x * e_x + self.kd_x * e_dot_x
        y_dotdot_c = self.kp_y * e_y + self.kd_y * e_dot_y

        s_psi = np.sin(setpoint.heading)
        c_psi = np.cos(setpoint.heading)

        U[0] = self.params.mass * (z_dotdot_c + self.params.g)
        phi_d = (1 / self.params.g) * (x_dotdot_c * s_psi - y_dotdot_c * c_psi)
        theta_d = (1 / self.params.g) * (x_dotdot_c * c_psi - y_dotdot_c * s_psi)
        # phi_d = .25
        # theta_d = .25
        # psi_d = current heading

        # TOOD: compute angular rate error
        e_p_dot = 0 - state.angular_velocity.x
        e_q_dot = 0 - state.angular_velocity.y
        e_r_dot = 0 - state.angular_velocity.z
    

        # Attitude controller
        # FIXME: not sure how phi/theta correspond to roll and pitch;
        # orientation.x, y, and z are yaw, pitch and roll, respectively (see hbird_sim_node.py)
        #i think there's some funky stuff in here with p,q,r
        e_phi = phi_d - state.orientation.x
        e_theta = theta_d - state.orientation.y
        e_psi = state.orientation.z - setpoint.heading  

        # e_dot_phi = 0 - state.angular_velocity.x
        # e_dot_theta = 0 - state.angular_velocity.y
        # e_dot_psi = 0 - state.angular_velocity.z

        U[1] = (self.kd_p * e_p_dot) + (self.kp_phi * e_phi)
        U[2] = (self.kd_q * e_q_dot) + (self.kp_theta * e_theta)
        U[3] = (self.kd_r * e_r_dot) + (self.kp_psi * e_psi)

        # U[1] = self.kp_phi * e_phi + self.kd_p * e_dot_phi
        # U[2] = self.kp_theta * e_theta +  self.kd_q * e_dot_theta
        # U[3] = self.kp_psi * e_psi + self.kd_r * e_dot_psi

        return U
