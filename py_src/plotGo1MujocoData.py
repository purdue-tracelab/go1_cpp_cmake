#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
# from mpl_toolkits.mplot3d import Axes3D

def plot_state_data(csv_file):

    # Load data
    df = pd.read_csv(csv_file)
    data_length = len(df)
    time = np.arange(0, data_length * 0.002, 0.002)

    root_pos = df[['root_pos_x', 'root_pos_y', 'root_pos_z']].values
    root_pos_d = df[['root_pos_d_x', 'root_pos_d_y', 'root_pos_d_z']].values
    root_pos_est = df[['root_pos_est_x', 'root_pos_est_y', 'root_pos_est_z']].values

    root_rpy = df[['root_rpy_x', 'root_rpy_y', 'root_rpy_z']].values
    root_rpy_d = df[['root_rpy_d_x', 'root_rpy_d_y', 'root_rpy_d_z']].values
    root_rpy_est = df[['root_rpy_est_x', 'root_rpy_est_y', 'root_rpy_est_z']].values

    root_lin_vel = df[['root_lin_vel_x', 'root_lin_vel_y', 'root_lin_vel_z']].values
    root_lin_vel_d = df[['root_lin_vel_d_x', 'root_lin_vel_d_y', 'root_lin_vel_d_z']].values
    root_lin_vel_est = df[['root_lin_vel_est_x', 'root_lin_vel_est_y', 'root_lin_vel_est_z']].values

    root_lin_acc_est = df[['root_lin_acc_est_x', 'root_lin_acc_est_y', 'root_lin_acc_est_z']].values

    root_ang_vel = df[['root_ang_vel_x', 'root_ang_vel_y', 'root_ang_vel_z']].values
    root_ang_vel_d = df[['root_ang_vel_d_x', 'root_ang_vel_d_y', 'root_ang_vel_d_z']].values

    foot_pos_FR = df[['foot_pos_FR_x', 'foot_pos_FR_y', 'foot_pos_FR_z']].values
    foot_pos_FL = df[['foot_pos_FL_x', 'foot_pos_FL_y', 'foot_pos_FL_z']].values
    foot_pos_RR = df[['foot_pos_RR_x', 'foot_pos_RR_y', 'foot_pos_RR_z']].values
    foot_pos_RL = df[['foot_pos_RL_x', 'foot_pos_RL_y', 'foot_pos_RL_z']].values

    foot_pos_abs_FR = df[['foot_pos_abs_FR_x', 'foot_pos_abs_FR_y', 'foot_pos_abs_FR_z']].values
    foot_pos_abs_FL = df[['foot_pos_abs_FL_x', 'foot_pos_abs_FL_y', 'foot_pos_abs_FL_z']].values
    foot_pos_abs_RR = df[['foot_pos_abs_RR_x', 'foot_pos_abs_RR_y', 'foot_pos_abs_RR_z']].values
    foot_pos_abs_RL = df[['foot_pos_abs_RL_x', 'foot_pos_abs_RL_y', 'foot_pos_abs_RL_z']].values

    foot_pos_liftoff_FR = df[['foot_pos_liftoff_FR_x', 'foot_pos_liftoff_FR_y', 'foot_pos_liftoff_FR_z']].values
    foot_pos_liftoff_FL = df[['foot_pos_liftoff_FL_x', 'foot_pos_liftoff_FL_y', 'foot_pos_liftoff_FL_z']].values
    foot_pos_liftoff_RR = df[['foot_pos_liftoff_RR_x', 'foot_pos_liftoff_RR_y', 'foot_pos_liftoff_RR_z']].values
    foot_pos_liftoff_RL = df[['foot_pos_liftoff_RL_x', 'foot_pos_liftoff_RL_y', 'foot_pos_liftoff_RL_z']].values

    foot_pos_d_FR = df[['foot_pos_d_FR_x', 'foot_pos_d_FR_y', 'foot_pos_d_FR_z']].values
    foot_pos_d_FL = df[['foot_pos_d_FL_x', 'foot_pos_d_FL_y', 'foot_pos_d_FL_z']].values
    foot_pos_d_RR = df[['foot_pos_d_RR_x', 'foot_pos_d_RR_y', 'foot_pos_d_RR_z']].values
    foot_pos_d_RL = df[['foot_pos_d_RL_x', 'foot_pos_d_RL_y', 'foot_pos_d_RL_z']].values

    foot_vel_d_FR = df[['foot_vel_d_FR_x', 'foot_vel_d_FR_y', 'foot_vel_d_FR_z']].values
    foot_vel_d_FL = df[['foot_vel_d_FL_x', 'foot_vel_d_FL_y', 'foot_vel_d_FL_z']].values
    foot_vel_d_RR = df[['foot_vel_d_RR_x', 'foot_vel_d_RR_y', 'foot_vel_d_RR_z']].values
    foot_vel_d_RL = df[['foot_vel_d_RL_x', 'foot_vel_d_RL_y', 'foot_vel_d_RL_z']].values

    foot_forces_grf_FR = df[['foot_forces_grf_FR_x', 'foot_forces_grf_FR_y', 'foot_forces_grf_FR_z']].values
    foot_forces_grf_FL = df[['foot_forces_grf_FL_x', 'foot_forces_grf_FL_y', 'foot_forces_grf_FL_z']].values
    foot_forces_grf_RR = df[['foot_forces_grf_RR_x', 'foot_forces_grf_RR_y', 'foot_forces_grf_RR_z']].values
    foot_forces_grf_RL = df[['foot_forces_grf_RL_x', 'foot_forces_grf_RL_y', 'foot_forces_grf_RL_z']].values

    foot_forces_swing_FR = df[['foot_forces_swing_FR_x', 'foot_forces_swing_FR_y', 'foot_forces_swing_FR_z']].values
    foot_forces_swing_FL = df[['foot_forces_swing_FL_x', 'foot_forces_swing_FL_y', 'foot_forces_swing_FL_z']].values
    foot_forces_swing_RR = df[['foot_forces_swing_RR_x', 'foot_forces_swing_RR_y', 'foot_forces_swing_RR_z']].values
    foot_forces_swing_RL = df[['foot_forces_swing_RL_x', 'foot_forces_swing_RL_y', 'foot_forces_swing_RL_z']].values

    joint_torques_FR = df[['joint_torques_FR_hip', 'joint_torques_FR_thigh', 'joint_torques_FR_calf']].values
    joint_torques_FL = df[['joint_torques_FL_hip', 'joint_torques_FL_thigh', 'joint_torques_FL_calf']].values
    joint_torques_RR = df[['joint_torques_RR_hip', 'joint_torques_RR_thigh', 'joint_torques_RR_calf']].values
    joint_torques_RL = df[['joint_torques_RL_hip', 'joint_torques_RL_thigh', 'joint_torques_RL_calf']].values

    swing_phase = df['swing_phase'].values

    # Plot data

    ###############
    ## Root pose ##
    ###############

    root_pose_plot = plt.figure(1, figsize=(16, 9))

    plt.subplot(3, 2, 1)
    plt.plot(time, root_pos[:, 0], label="Actual x pos", color='r')
    plt.plot(time, root_pos_d[:, 0], label="Desired x pos", color='b')
    # plt.plot(time, root_pos_est[:, 0], label="Estimated x pos", color='g')
    plt.title("Root position x")
    plt.xlabel("Time (s)")
    plt.ylabel("X position (m)")
    plt.legend()

    plt.subplot(3, 2, 2)
    plt.plot(time, root_pos[:, 1], label="Actual y pos", color='r')
    plt.plot(time, root_pos_d[:, 1], label="Desired y pos", color='b')
    # plt.plot(time, root_pos_est[:, 1], label="Estimated y pos", color='g')
    plt.title("Root position y")
    plt.xlabel("Time (s)")
    plt.ylabel("Y position (m)")
    plt.legend()

    plt.subplot(3, 2, 3)
    plt.plot(time, root_pos[:, 2], label="Actual z pos", color='r')
    plt.plot(time, root_pos_d[:, 2], label="Desired z pos", color='b')
    # plt.plot(time, root_pos_est[:, 2], label="Estimated z pos", color='g')
    plt.title("Root position z")
    plt.xlabel("Time (s)")
    plt.ylabel("Z position (m)")
    plt.legend()

    plt.subplot(3, 2, 4)
    plt.plot(time, root_rpy[:, 0], label="Actual roll", color='r')
    plt.plot(time, root_rpy_d[:, 0], label="Desired roll", color='b')
    # plt.plot(time, root_rpy_est[:, 0], label="Estimated roll", color='g')
    plt.title("Root roll")
    plt.xlabel("Time (s)")
    plt.ylabel("Roll angle (rad)")
    plt.legend()

    plt.subplot(3, 2, 5)
    plt.plot(time, root_rpy[:, 1], label="Actual pitch", color='r')
    plt.plot(time, root_rpy_d[:, 1], label="Desired pitch", color='b')
    # plt.plot(time, root_rpy_est[:, 1], label="Estimated pitch", color='g')
    plt.title("Root pitch")
    plt.xlabel("Time (s)")
    plt.ylabel("Pitch angle (rad)")
    plt.legend()

    plt.subplot(3, 2, 6)
    plt.plot(time, root_rpy[:, 2], label="Actual yaw", color='r')
    plt.plot(time, root_rpy_d[:, 2], label="Desired yaw", color='b')
    # plt.plot(time, root_rpy_est[:, 2], label="Estimated yaw", color='g')
    plt.title("Root yaw")
    plt.xlabel("Time (s)")
    plt.ylabel("Yaw angle (rad)")
    plt.legend()

    plt.tight_layout()
    plt.savefig("data/root_pose.png")

    ##############
    ## Root vel ##
    ##############

    root_vel_plot = plt.figure(2, figsize=(16, 9))

    plt.subplot(3, 2, 1)
    plt.plot(time, root_lin_vel[:, 0], label="Actual x vel", color='r')
    plt.plot(time, root_lin_vel_d[:, 0], label="Desired x vel", color='b')
    # plt.plot(time, root_lin_vel_est[:, 0], label="Estimated x vel", color='g')
    plt.title("Root linear velocity x")
    plt.xlabel("Time (s)")
    plt.ylabel("X linear velocity (m/s)")
    plt.legend()

    plt.subplot(3, 2, 2)
    plt.plot(time, root_lin_vel[:, 1], label="Actual y vel", color='r')
    plt.plot(time, root_lin_vel_d[:, 1], label="Desired y vel", color='b')
    # plt.plot(time, root_lin_vel_est[:, 1], label="Estimated y vel", color='g')
    plt.title("Root linear velocity y")
    plt.xlabel("Time (s)")
    plt.ylabel("Y linear velocity (m/s)")
    plt.legend()

    plt.subplot(3, 2, 3)
    plt.plot(time, root_lin_vel[:, 2], label="Actual z vel", color='r')
    plt.plot(time, root_lin_vel_d[:, 2], label="Desired z vel", color='b')
    # plt.plot(time, root_lin_vel_est[:, 2], label="Estimated z vel", color='g')
    plt.title("Root linear velocity z")
    plt.xlabel("Time (s)")
    plt.ylabel("Z linear velocity (m/s)")
    plt.legend()

    plt.subplot(3, 2, 4)
    plt.plot(time, root_ang_vel[:, 0], label="Actual roll rate", color='r')
    plt.plot(time, root_ang_vel_d[:, 0], label="Desired roll rate", color='b')
    plt.title("Root angular velocity x")
    plt.xlabel("Time (s)")
    plt.ylabel("X angular velocity (rad/s)")
    plt.legend()

    plt.subplot(3, 2, 5)
    plt.plot(time, root_ang_vel[:, 1], label="Actual pitch rate", color='r')
    plt.plot(time, root_ang_vel_d[:, 1], label="Desired pitch rate", color='b')
    plt.title("Root angular velocity y")
    plt.xlabel("Time (s)")
    plt.ylabel("Y angular velocity (rad/s)")
    plt.legend()

    plt.subplot(3, 2, 6)
    plt.plot(time, root_ang_vel[:, 2], label="Actual yaw rate", color='r')
    plt.plot(time, root_ang_vel_d[:, 2], label="Desired yaw rate", color='b')
    plt.title("Root angular velocity z")
    plt.xlabel("Time (s)")
    plt.ylabel("Z angular velocity (rad/s)")
    plt.legend()

    plt.tight_layout()
    plt.savefig("data/root_vel.png")

    #################
    ## Foot forces ##
    #################

    root_force_plot = plt.figure(3, figsize=(16, 9))

    plt.subplot(3, 2, 1)
    plt.plot(time, foot_forces_grf_FR[:, 0], label="GRF FR x", color='r')
    plt.plot(time, foot_forces_grf_FL[:, 0], label="GRF FL x", color='b')
    plt.plot(time, foot_forces_grf_RR[:, 0], label="GRF RR x", color='g')
    plt.plot(time, foot_forces_grf_RL[:, 0], label="GRF RL x", color='orange')
    plt.title("Foot GRF x")
    plt.xlabel("Time (s)")
    plt.ylabel("X GRF (N)")
    plt.legend()

    plt.subplot(3, 2, 2)
    plt.plot(time, foot_forces_grf_FR[:, 1], label="GRF FR y", color='r')
    plt.plot(time, foot_forces_grf_FL[:, 1], label="GRF FL y", color='b')
    plt.plot(time, foot_forces_grf_RR[:, 1], label="GRF RR y", color='g')
    plt.plot(time, foot_forces_grf_RL[:, 1], label="GRF RL y", color='orange')
    plt.title("Foot GRF y")
    plt.xlabel("Time (s)")
    plt.ylabel("Y GRF (N)")
    plt.legend()

    plt.subplot(3, 2, 3)
    plt.plot(time, foot_forces_grf_FR[:, 2], label="GRF FR z", color='r')
    plt.plot(time, foot_forces_grf_FL[:, 2], label="GRF FL z", color='b')
    plt.plot(time, foot_forces_grf_RR[:, 2], label="GRF RR z", color='g')
    plt.plot(time, foot_forces_grf_RL[:, 2], label="GRF RL z", color='orange')
    plt.title("Foot GRF z")
    plt.xlabel("Time (s)")
    plt.ylabel("Z GRF (N)")
    plt.legend()

    plt.subplot(3, 2, 4)
    plt.plot(time, foot_forces_swing_FR[:, 0], label="swing FR x", color='r')
    plt.plot(time, foot_forces_swing_FL[:, 0], label="swing FL x", color='b')
    plt.plot(time, foot_forces_swing_RR[:, 0], label="swing RR x", color='g')
    plt.plot(time, foot_forces_swing_RL[:, 0], label="swing RL x", color='orange')
    plt.title("Foot swing forces x")
    plt.xlabel("Time (s)")
    plt.ylabel("X swing forces (N)")
    plt.legend()

    plt.subplot(3, 2, 5)
    plt.plot(time, foot_forces_swing_FR[:, 1], label="swing FR y", color='r')
    plt.plot(time, foot_forces_swing_FL[:, 1], label="swing FL y", color='b')
    plt.plot(time, foot_forces_swing_RR[:, 1], label="swing RR y", color='g')
    plt.plot(time, foot_forces_swing_RL[:, 1], label="swing RL y", color='orange')
    plt.title("Foot swing forces y")
    plt.xlabel("Time (s)")
    plt.ylabel("Y swing forces (N)")
    plt.legend()

    plt.subplot(3, 2, 6)
    plt.plot(time, foot_forces_swing_FR[:, 2], label="swing FR z", color='r')
    plt.plot(time, foot_forces_swing_FL[:, 2], label="swing FL z", color='b')
    plt.plot(time, foot_forces_swing_RR[:, 2], label="swing RR z", color='g')
    plt.plot(time, foot_forces_swing_RL[:, 2], label="swing RL z", color='orange')
    plt.title("Foot swing forces z")
    plt.xlabel("Time (s)")
    plt.ylabel("Z swing forces (N)")
    plt.legend()

    plt.tight_layout()
    plt.savefig("data/foot_forces.png")

    ###################
    ## Joint torques ##
    ###################

    root_torque_plot = plt.figure(4, figsize=(16, 9))

    plt.subplot(3, 1, 1)
    plt.plot(time, joint_torques_FR[:, 0], label="FR hip", color='r')
    plt.plot(time, joint_torques_FL[:, 0], label="FL hip", color='b')
    plt.plot(time, joint_torques_RR[:, 0], label="RR hip", color='g')
    plt.plot(time, joint_torques_RL[:, 0], label="RL hip", color='orange')
    plt.title("Hip joint torques")
    plt.xlabel("Time (s)")
    plt.ylabel("Torque (N*m)")
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(time, joint_torques_FR[:, 1], label="FR thigh", color='r')
    plt.plot(time, joint_torques_FL[:, 1], label="FL thigh", color='b')
    plt.plot(time, joint_torques_RR[:, 1], label="RR thigh", color='g')
    plt.plot(time, joint_torques_RL[:, 1], label="RL thigh", color='orange')
    plt.title("Thigh joint torques")
    plt.xlabel("Time (s)")
    plt.ylabel("Torque (N*m)")
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(time, joint_torques_FR[:, 2], label="FR calf", color='r')
    plt.plot(time, joint_torques_FL[:, 2], label="FL calf", color='b')
    plt.plot(time, joint_torques_RR[:, 2], label="RR calf", color='g')
    plt.plot(time, joint_torques_RL[:, 2], label="RL calf", color='orange')
    plt.title("Calf joint torques")
    plt.xlabel("Time (s)")
    plt.ylabel("Torque (N*m)")
    plt.legend()

    plt.tight_layout()
    plt.savefig("data/joint_torques.png")

    ####################
    ## Foot positions ##
    ####################

    foot_pos_plot = plt.figure(5, figsize=(16, 9))

    plt.subplot(4, 3, 1)
    plt.plot(time, foot_pos_abs_FR[:, 0], label="Actual FR pos x", color='r')
    plt.plot(time, foot_pos_d_FR[:, 0], label="Desired FR pos x", color='b')
    plt.title("FR foot x pos (abs frame)")
    plt.xlabel("Time (s)")
    plt.ylabel("Foot pos x (m)")
    plt.legend()

    plt.subplot(4, 3, 2)
    plt.plot(time, foot_pos_abs_FR[:, 1], label="Actual FR pos y", color='r')
    plt.plot(time, foot_pos_d_FR[:, 1], label="Desired FR pos y", color='b')
    plt.title("FR foot y pos (abs frame)")
    plt.xlabel("Time (s)")
    plt.ylabel("Foot pos y (m)")
    plt.legend()

    plt.subplot(4, 3, 3)
    plt.plot(time, foot_pos_abs_FR[:, 2], label="Actual FR pos z", color='r')
    plt.plot(time, foot_pos_d_FR[:, 2], label="Desired FR pos z", color='b')
    plt.title("FR foot z pos (abs frame)")
    plt.xlabel("Time (s)")
    plt.ylabel("Foot pos z (m)")
    plt.legend()

    plt.subplot(4, 3, 4)
    plt.plot(time, foot_pos_abs_FL[:, 0], label="Actual FL pos x", color='r')
    plt.plot(time, foot_pos_d_FL[:, 0], label="Desired FR pos x", color='b')
    plt.title("FL foot x pos (abs frame)")
    plt.xlabel("Time (s)")
    plt.ylabel("Foot pos x (m)")
    plt.legend()

    plt.subplot(4, 3, 5)
    plt.plot(time, foot_pos_abs_FL[:, 1], label="Actual FL pos y", color='r')
    plt.plot(time, foot_pos_d_FL[:, 1], label="Desired FL pos y", color='b')
    plt.title("FL foot y pos (abs frame)")
    plt.xlabel("Time (s)")
    plt.ylabel("Foot pos y (m)")
    plt.legend()

    plt.subplot(4, 3, 6)
    plt.plot(time, foot_pos_abs_FL[:, 2], label="Actual FL pos z", color='r')
    plt.plot(time, foot_pos_d_FL[:, 2], label="Desired FL pos z", color='b')
    plt.title("FL foot z pos (abs frame)")
    plt.xlabel("Time (s)")
    plt.ylabel("Foot pos z (m)")
    plt.legend()

    plt.subplot(4, 3, 7)
    plt.plot(time, foot_pos_abs_RR[:, 0], label="Actual RR pos x", color='r')
    plt.plot(time, foot_pos_d_RR[:, 0], label="Desired RR pos x", color='b')
    plt.title("RR foot x pos (abs frame)")
    plt.xlabel("Time (s)")
    plt.ylabel("Foot pos x (m)")
    plt.legend()

    plt.subplot(4, 3, 8)
    plt.plot(time, foot_pos_abs_RR[:, 1], label="Actual RR pos y", color='r')
    plt.plot(time, foot_pos_d_RR[:, 1], label="Desired RR pos y", color='b')
    plt.title("RR foot y pos (abs frame)")
    plt.xlabel("Time (s)")
    plt.ylabel("Foot pos y (m)")
    plt.legend()

    plt.subplot(4, 3, 9)
    plt.plot(time, foot_pos_abs_RR[:, 2], label="Actual RR pos z", color='r')
    plt.plot(time, foot_pos_d_RR[:, 2], label="Desired RR pos z", color='b')
    plt.title("RR foot z pos (abs frame)")
    plt.xlabel("Time (s)")
    plt.ylabel("Foot pos z (m)")
    plt.legend()

    plt.subplot(4, 3, 10)
    plt.plot(time, foot_pos_abs_RL[:, 0], label="Actual RL pos x", color='r')
    plt.plot(time, foot_pos_d_RL[:, 0], label="Desired RL pos x", color='b')
    plt.title("RL foot x pos (abs frame)")
    plt.xlabel("Time (s)")
    plt.ylabel("Foot pos x (m)")
    plt.legend()

    plt.subplot(4, 3, 11)
    plt.plot(time, foot_pos_abs_RL[:, 1], label="Actual RL pos y", color='r')
    plt.plot(time, foot_pos_d_RL[:, 1], label="Desired RL pos y", color='b')
    plt.title("RL foot y pos (abs frame)")
    plt.xlabel("Time (s)")
    plt.ylabel("Foot pos y (m)")
    plt.legend()

    plt.subplot(4, 3, 12)
    plt.plot(time, foot_pos_abs_RL[:, 2], label="Actual RL pos z", color='r')
    plt.plot(time, foot_pos_d_RL[:, 2], label="Desired RL pos z", color='b')
    plt.title("RL foot z pos (abs frame)")
    plt.xlabel("Time (s)")
    plt.ylabel("Foot pos z (m)")
    plt.legend()

    plt.tight_layout()
    plt.savefig("data/2d_foot_pos.png")

    # plt.close(root_pose_plot)
    # plt.close(root_vel_plot)
    # plt.close(foot_pos_plot)
    # plt.close(root_force_plot)
    # plt.close(root_torque_plot)
    plt.show()

csv_file = "data/go1_mujoco_data.csv"
plot_state_data(csv_file)