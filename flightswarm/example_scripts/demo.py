# !/usr/bin/env python3

import os
import argparse
import numpy as np
from ruamel.yaml import YAML, dump, RoundTripDumper
from rpg_baselines.envs import vec_env_wrapper as wrapper
import matplotlib.pyplot as plt
from matplotlib import gridspec

from flightgym import QuadrotorEnv_v1

def parser():
    """
        Parser
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--train', type=int, default=1,
                        help="To train new model or simply test pre-trained model")
    parser.add_argument('--render', type=int, default=0,
                        help="Enable Unity Render")
    parser.add_argument('--save_dir', type=str, default=os.path.dirname(os.path.realpath(__file__)),
                        help="Directory where to save the checkpoints and training metrics")
    parser.add_argument('--seed', type=int, default=0,
                        help="Random seed")
    parser.add_argument('-w', '--weight', type=str, default='./saved/quadrotor_env.zip',
                        help='trained weight path')
    return parser

def main():
    """
        Main Quadcopter Script
    """
    # Define Quadcopter Number
    uav_num = 1

    args = parser().parse_args()
    cfg = YAML().load(open(os.environ["FLIGHTMARE_PATH"] +
                           "/flightlib/configs/vec_env.yaml", 'r', encoding="utf8"))
    if not args.train:
        cfg["env"]["num_envs"] = uav_num
        cfg["env"]["num_threads"] = 1

    if args.render:
        cfg["env"]["render"] = "yes"
    else:
        cfg["env"]["render"] = "no"

    env = wrapper.FlightEnvVec(QuadrotorEnv_v1(
        dump(cfg, Dumper=RoundTripDumper), False))

    if args.render:
        # Connect to Flightmare.
        env.connectUnity()

    # Reset the quadrotor state and get observations.
    uav_states = env.reset()

    # UAV Initialization
    uav_list = []
    for uav_id in range(uav_num):
        uav_list.append(UavClass(uav_id))

    # Plot
    fig = plt.figure(figsize=(9, 6), tight_layout=True)
    gs = gridspec.GridSpec(1, 12)

    fig1 = plt.figure(figsize=(9, 6), tight_layout=True)
    gs1 = gridspec.GridSpec(1, 9)

    fig2 = plt.figure(figsize=(9, 6), tight_layout=True)
    gs2 = gridspec.GridSpec(1, 1)

    ax_euler_x = fig.add_subplot(gs[0, 0:4])
    ax_euler_y = fig.add_subplot(gs[0, 4:8])
    ax_euler_z = fig.add_subplot(gs[0, 8:12])

    m1_command = fig1.add_subplot(gs1[0, 1:3])
    m2_command = fig1.add_subplot(gs1[0, 3:5])
    m3_command = fig1.add_subplot(gs1[0, 5:7])
    m4_command = fig1.add_subplot(gs1[0, 7:9])

    alt_command = fig2.add_subplot(gs2[0, 0])

    dt = 0.1
    sim_time = 0.0
    alt = []
    m1, m2, m3, m4 = [], [], [], []
    a1, a2, a3, a4 = [], [], [], []
    euler_x, euler_y, euler_z = [], [], []
    ref_x, ref_y, ref_z = [], [], []
    while True:
        # Parse Quadcopter States
        for i, uav in enumerate(uav_list):
            uav.update_uav_states(uav_states[i,:])
        
        # Calculate Ref Cmds
        if sim_time < 1:
            attitude_ref = [0.0, 0.0, 0.0]
        elif sim_time >= 1 and sim_time < 30:
            attitude_ref = [0.0*3.14/180, 0*3.14/180, 0*3.14/180]
        else:
            attitude_ref = [0.0, 0.0, 0.0]

        # Calculate & Store Quadcopter Commands
        thrust = np.zeros([uav_num,4], dtype=np.float32)
        for i, uav in enumerate(uav_list):
            thrust[i,:] = uav.step(attitude_ref)

        thrust[0][3] = 0

        action = env.sample_actions()

        t = np.ones([1,4], dtype=np.float32) 
        t[0][0]=t[0][0]*0.0 # omegaX
        t[0][1]=t[0][1]*0.0 # omegaY
        t[0][2]=t[0][2]*0.0 # omegaZ
        t[0][3]=t[0][3]*20.0 # thrust
        # Run Simulation One Step
        uav_states, reward, done, info = env.step(t)

        if uav_states[0, 3] >= 3.13 or uav_states[0, 3] <= -3.13:
            uav_states[0, 3] = 0.0

        if uav_states[0, 4] >= 3.13 or uav_states[0, 4] <= -3.13:
            uav_states[0, 4] = 0.0

        if uav_states[0, 5] >= 3.13 or uav_states[0, 5] <= -3.13:
            uav_states[0, 5] = 0.0

        euler_x.append(uav_states[0, 3]*180/3.14)
        euler_y.append(uav_states[0, 4]*180/3.14)
        euler_z.append(uav_states[0, 5]*180/3.14)
        ref_x.append(attitude_ref[0]*180/3.14)
        ref_y.append(attitude_ref[1]*180/3.14)
        ref_z.append(attitude_ref[2]*180/3.14)

        m1.append(thrust[0][0])
        m2.append(thrust[0][1])
        m3.append(thrust[0][2])
        m4.append(thrust[0][3])

        a1.append(action[0][0])
        a2.append(action[0][1])
        a3.append(action[0][2])
        a4.append(action[0][3])

        alt.append(uav_states[0, 2])

        sim_time = sim_time + dt
        print(np.round(sim_time,1), ' | ', np.round(uav_states[0, 3], 4), ' | ', np.round(uav_states[0, 4], 4), ' | ', np.round(uav_states[0, 5],4), ' | ', np.round(uav_states[0, 2],4), ' | ', np.round(t[0][0],4), ' | ', np.round(t[0][1],4), ' | ', np.round(t[0][2],4), ' | ', np.round(t[0][3],4))

        if sim_time > 100:
            env.disconnectUnity()
            break

    euler_x = np.asarray(euler_x)
    euler_y = np.asarray(euler_y)
    euler_z = np.asarray(euler_z)
    ref_x = np.asarray(ref_x)
    ref_y = np.asarray(ref_y)
    ref_z = np.asarray(ref_z)
    m1 = np.asarray(m1)
    m2 = np.asarray(m2)
    m3 = np.asarray(m3)
    m4 = np.asarray(m4)
    a1 = np.asarray(a1)
    a2 = np.asarray(a2)
    a3 = np.asarray(a3)
    a4 = np.asarray(a4)
    alt = np.asarray(alt)

    t = np.arange(0, euler_x.shape[0])

    ax_euler_x.step(t, euler_x, label="PHI_state")
    ax_euler_x.step(t, ref_x, label="PHI_ref")
    ax_euler_y.step(t, euler_y, label="THETA_state")
    ax_euler_y.step(t, ref_y, label="THETA_ref")
    ax_euler_z.step(t, euler_z, label="PSI_state")
    ax_euler_z.step(t, ref_z, label="PSI_ref")

    # m1_command.step(t, m1, label="M1")
    # m2_command.step(t, m2, label="M2")
    # m3_command.step(t, m3, label="M3")
    # m4_command.step(t, m4, label="M4")
    m1_command.step(t, a1, label="A1")
    m2_command.step(t, a2, label="A2")
    m3_command.step(t, a3, label="A3")
    m4_command.step(t, a4, label="A4")

    alt_command.step(t, alt, label="altitute")

    ax_euler_x.legend()
    ax_euler_y.legend()
    ax_euler_z.legend()
    m1_command.legend()
    m2_command.legend()
    m3_command.legend()
    m4_command.legend()
    alt_command.legend()
    

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
