from simple_pid import PID


class LowLevelCOntrollerClass:
    """
        Low level controller properties and methods
    """
    def __init__(self, states):
        """
            Initialization of Class
        """
        self.kp_att_cont = 0
        self.ki_att_cont = 0
        self.kd_att_cont = 0
        self.kp_vel_cont = 0
        self.ki_vel_cont = 0
        self.kd_vel_cont = 0
        self.kp_pos_cont = 0
        self.ki_pos_cont = 0
        self.kd_pos_cont = 0

        self.pid_att_cont_x = PID(self.kp_att_cont, self.ki_att_cont, self.kd_att_cont, setpoint=0)
        self.pid_att_cont_y = PID(self.kp_att_cont, self.ki_att_cont, self.kd_att_cont, setpoint=0)
        self.pid_att_cont_z = PID(self.kp_att_cont, self.ki_att_cont, self.kd_att_cont, setpoint=0)

        self.pid_vel_cont_x = PID(self.kp_vel_cont, self.ki_vel_cont, self.kd_vel_cont, setpoint=0)
        self.pid_vel_cont_y = PID(self.kp_vel_cont, self.ki_vel_cont, self.kd_vel_cont, setpoint=0)
        self.pid_vel_cont_z = PID(self.kp_vel_cont, self.ki_vel_cont, self.kd_vel_cont, setpoint=0)
        
        self.pid_pos_cont_x = PID(self.kp_pos_cont, self.ki_pos_cont, self.kd_pos_cont, setpoint=0)
        self.pid_pos_cont_y = PID(self.kp_pos_cont, self.ki_pos_cont, self.kd_pos_cont, setpoint=0)
        self.pid_pos_cont_z = PID(self.kp_pos_cont, self.ki_pos_cont, self.kd_pos_cont, setpoint=0)
        
        # Set Current States
        self.cur_lin_pos_x = states[0]
        self.cur_lin_pos_y = states[1]
        self.cur_lin_pos_z = states[2]
        self.cur_ang_pos_x = states[3]
        self.cur_ang_pos_y = states[4]
        self.cur_ang_pos_z = states[5]
        self.cur_lin_vel_x = states[6]
        self.cur_lin_vel_y = states[7]
        self.cur_lin_vel_z = states[8]
        self.cur_ang_vel_x = states[9]
        self.cur_ang_vel_y = states[10]
        self.cur_ang_vel_z = states[11]

    def attitude_controller(self, ref_ang_pos, ref_collect_thrust):
        """
            Low level attitude controller
        """
        self.pid_att_cont_x.setpoint = ref_ang_pos[0]
        self.pid_att_cont_y.setpoint = ref_ang_pos[1]
        self.pid_att_cont_z.setpoint = ref_ang_pos[2]

        cmd_ang_vel_x = self.pid_att_cont_x(self.cur_ang_pos_x)
        cmd_ang_vel_y = self.pid_att_cont_y(self.cur_ang_pos_y)
        cmd_ang_vel_z = self.pid_att_cont_z(self.cur_ang_pos_z)
        cmd_ang_vel = [cmd_ang_vel_x, cmd_ang_vel_y, cmd_ang_vel_z]

        cmd_collect_thrust = ref_collect_thrust

        return cmd_ang_vel, cmd_collect_thrust

    def velocity_controller(self, ref_lin_vel, ref_collect_thrust):
        """
            Low level linear velocity controller
        """
        self.pid_vel_cont_x.setpoint = ref_lin_vel[0]
        self.pid_vel_cont_y.setpoint = ref_lin_vel[1]
        self.pid_vel_cont_z.setpoint = ref_lin_vel[2]

        cmd_ang_pos_x = self.pid_vel_cont_x(self.cur_lin_vel_x)
        cmd_ang_pos_y = self.pid_vel_cont_y(self.cur_lin_vel_y)
        cmd_ang_pos_z = self.pid_vel_cont_z(self.cur_lin_vel_z)
        cmd_ang_pos = [cmd_ang_pos_x, cmd_ang_pos_y, cmd_ang_pos_z]    
        
        cmd_ang_vel, cmd_collect_thrust = self.attitude_controller(cmd_ang_pos, ref_collect_thrust)

         return cmd_ang_vel, cmd_collect_thrust

    def position_controller(self, ref_lin_pos, cur_lin_pos, ref_collect_thrust):
        """
            Low level linear position controller
        """
        return [cmd_ang_vel_x, cmd_ang_vel_y, cmd_ang_vel_z, cmd_collect_thrust]
        