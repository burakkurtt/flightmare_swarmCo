from simple_pid import PID

class LowLevelCOntrollerClass:
    """
        Low level controller properties and methods
    """
    def __init__(self):
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

    def attitude_controller(self, ref_ang_pos, cur_ang_pos, ref_collect_thrust):
        """
            Low level attitude controller
        """
        return [cmd_ang_vel_x, cmd_ang_vel_y, cmd_ang_vel_z, cmd_collect_thrust]

    def velocity_controller(self, ref_lin_vel, cur_ang_pos, ref_collect_thrust):
        """
            Low level linear velocity controller
        """
        return [cmd_ang_vel_x, cmd_ang_vel_y, cmd_ang_vel_z, cmd_collect_thrust]

    def position_controller(self, ref_lin_pos, cur_lin_pos, ref_collect_thrust):
        """
            Low level linear position controller
        """
        return [cmd_ang_vel_x, cmd_ang_vel_y, cmd_ang_vel_z, cmd_collect_thrust]
        