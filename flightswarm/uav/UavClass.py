from ctypes import util
from distutils.log import error
from simple_pid import PID
import numpy as np 
import uav.utils as utils 

class UavClass:
    def __init__(self, id):
        self.name = 'uav_' + str(id)
        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0
        self.orientation_x = 0
        self.orientation_y = 0
        self.orientation_z = 0
        self.linearVel_x = 0
        self.linearVel_y = 0
        self.linearVel_z = 0
        self.angularVel_x = 0
        self.angularVel_y = 0
        self.angularVel_z = 0
        # PID controller class init
        self.pid_phi = PID(0.0, 0.0, 0.0, setpoint=0.0)
        self.pid_theta = PID(0.0, 0.0, 0.0, setpoint=0.0)
        self.pid_psi = PID(0.0, 0.0, 0.0, setpoint=0.0)
        self.pid_alt = PID(0.5, 0.0, 0.0, setpoint=0.0)

        self.motor_cmds = np.zeros([1,4], dtype=np.float32)


    def update_uav_states(self, states):
        self.pos_x = states[0]
        self.pos_y = states[1]
        self.pos_z = states[2]
        self.orientation_x = states[3]
        self.orientation_y = states[4]
        self.orientation_z = states[5]
        self.linearVel_x = states[6]
        self.linearVel_y = states[7]
        self.linearVel_z = states[8]
        self.angularVel_x = states[9]
        self.angularVel_y = states[10]
        self.angularVel_z = states[11]


    def step(self, attitude_ref):
        motor_cmds = self.sas_controller(attitude_ref, 6, 'pid')
        return motor_cmds


    def sas_controller(self, attitude_ref, altitute_ref, controller_type='pid'):
        if controller_type == "pid":
            """
                Attitude Controller
            """   

            # Pid controller
            self.pid_phi.setpoint = attitude_ref[0]
            if self.orientation_x >= 3.13 or self.orientation_x <= -3.13:
                self.orientation_x = 0.0

            phi_cmd = self.pid_phi(-1*self.orientation_x) # Here, orientation multiply with -1. because uav state are taken reverse
            
            self.pid_theta.setpoint = attitude_ref[1]
            if self.orientation_y >= 3.13 or self.orientation_y <= -3.13:
                self.orientation_y = 0.0

            theta_cmd = self.pid_theta(self.orientation_y)

            self.pid_psi.setpoint = attitude_ref[2]
            if self.orientation_z >= 3.13 or self.orientation_z <= -3.13:
                self.orientation_z = 0.0

            psi_cmd = self.pid_psi(self.orientation_z)

            
            self.pid_alt.setpoint = altitute_ref
            alt_cmd = self.pid_alt(self.pos_z)

            # Motor control allocation
            cmds = utils.control_allocation(phi_cmd, theta_cmd, psi_cmd, alt_cmd) 
            
            self.motor_cmds[0,0] = cmds[0]
            self.motor_cmds[0,1] = cmds[1]
            self.motor_cmds[0,2] = cmds[2]
            self.motor_cmds[0,3] = cmds[3]

            return self.motor_cmds

        elif controller_type == "lqr":
            """
                Under Construction
            """
            raise SyntaxError('LQR controller is under construction.')
            return np.ones([1,4], dtype=np.float32) * 0.01

        elif controller_type == "rl":
            """
                Under Construction
            """
            raise SyntaxError('Reinforcement Learning controller is under construction.')
            return np.ones([1,4], dtype=np.float32) * 0.01

        else:
            print('here'),
            raise SyntaxError('Please select controller type or Selected controller is not apropriate.')


    def cas_controller(self, position_ref):
        return np.zeros([1,4], dtype=np.float32)

    def nav_controller(self):
        pass