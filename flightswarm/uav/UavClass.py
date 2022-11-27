from simple_pid import PID
import numpy as np
from uav import utils

class UavClass:
    """
        UAV Class
    """
    def __init__(self, uav_id):
        self.name = 'uav_' + str(uav_id)
        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0
        self.orientation_x = 0
        self.orientation_y = 0
        self.orientation_z = 0
        self.linear_vel_x = 0
        self.linear_vel_y = 0
        self.linear_vel_z = 0
        self.angular_vel_x = 0
        self.angular_vel_y = 0
        self.angular_vel_z = 0
        # PID controller class init
        self.pid_phi = PID(0.0, 0.0, 0.0, setpoint=0.0)
        self.pid_theta = PID(0.0, 0.0, 0.0, setpoint=0.0)
        self.pid_psi = PID(0.0, 0.0, 0.0, setpoint=0.0)
        self.pid_alt = PID(0.5, 0.0, 0.0, setpoint=0.0)

    def update_uav_states(self, states):
        """
            Update uav states
        """
        self.pos_x = states[0]
        self.pos_y = states[1]
        self.pos_z = states[2]
        self.orientation_x = states[3]
        self.orientation_y = states[4]
        self.orientation_z = states[5]
        self.linear_vel_x = states[6]
        self.linear_vel_y = states[7]
        self.linear_vel_z = states[8]
        self.angular_vel_x = states[9]
        self.angular_vel_y = states[10]
        self.angular_vel_z = states[11]

    def step(self):
        """
            Simulation step
        """
        return None