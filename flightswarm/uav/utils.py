def control_allocation(phi_cmd, theta_cmd, psi_cmd, alt_cmd):
    """
        Quadcopter is in X configuration. motor number is in 
        M4    M1
           \/
           /\
        M3    M2
    """
    m1 = -phi_cmd/4 + theta_cmd/4 - psi_cmd/4 + alt_cmd
    m2 = -phi_cmd/4 - theta_cmd/4 + psi_cmd/4 + alt_cmd
    m3 = phi_cmd/4 - theta_cmd/4 - psi_cmd/4 + alt_cmd
    m4 = phi_cmd/4 + theta_cmd/4 + psi_cmd/4 + alt_cmd

    a = 5e-3

    m1 = 0.5
    m2 = 0.5
    m3 = 0.5
    m4 = 0.5

    return [m1, m2, m3, m4]