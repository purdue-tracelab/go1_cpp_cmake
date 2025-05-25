#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
# from mpl_toolkits.mplot3d import Axes3D

def contiguous_segments(time, mask):
    """
    Given a 1D time array and a boolean mask of the same length,
    returns a list of (start, end) time tuples for each contiguous True block.
    """
    segments = []
    start = None
    for t, m in zip(time, mask):
        if m and start is None:
            start = t
        elif not m and start is not None:
            segments.append((start, t))
            start = None
    # close out final segment
    if start is not None:
        segments.append((start, time[-1]))
    return segments

def shade_by_phase(ax, time, phase, threshold=99,
                           low_color='lightcoral', high_color='lightblue',
                           alpha=0.3):
    low_mask  = phase <= threshold
    high_mask = phase >  threshold

    # shade all low‐phase segments in pale red
    for t0, t1 in contiguous_segments(time, low_mask):
        ax.axvspan(t0, t1, facecolor=low_color, alpha=alpha, zorder=0)

    # shade all high‐phase segments in pale blue
    for t0, t1 in contiguous_segments(time, high_mask):
        ax.axvspan(t0, t1, facecolor=high_color, alpha=alpha, zorder=0)

def plot_state_data(csv_file, state_est_select=2):
    # Load data
    df = pd.read_csv(csv_file, on_bad_lines='skip', engine='python', encoding='utf-8', encoding_errors='ignore')
    data_length = len(df)
    time = np.arange(0, data_length * 0.002, 0.002)

    # Extract data
    foot_pos_meas = df[['z_k1', 'z_k2', 'z_k3', 'z_k4', 'z_k5', 'z_k6', 'z_k7', 'z_k8', 'z_k9', 'z_k10', 'z_k11', 'z_k12']].values
    foot_pos_pred = df[['Hx_k1', 'Hx_k2', 'Hx_k3', 'Hx_k4', 'Hx_k5', 'Hx_k6', 'Hx_k7', 'Hx_k8', 'Hx_k9', 'Hx_k10', 'Hx_k11', 'Hx_k12']].values

    foot_vel_meas = df[['z_k13', 'z_k14', 'z_k15', 'z_k16', 'z_k17', 'z_k18', 'z_k19', 'z_k20', 'z_k21', 'z_k22', 'z_k23', 'z_k24']].values
    foot_vel_pred = df[['Hx_k13', 'Hx_k14', 'Hx_k15', 'Hx_k16', 'Hx_k17', 'Hx_k18', 'Hx_k19', 'Hx_k20', 'Hx_k21', 'Hx_k22', 'Hx_k23', 'Hx_k24']].values

    foot_pos_z_meas = df[['z_k25', 'z_k26', 'z_k27', 'z_k28']].values
    foot_pos_z_pred = df[['Hx_k25', 'Hx_k26', 'Hx_k27', 'Hx_k28']].values

    pos_res_post_fit = df[['y_k1', 'y_k2', 'y_k3', 'y_k4', 'y_k5', 'y_k6', 'y_k7', 'y_k8', 'y_k9', 'y_k10', 'y_k11', 'y_k12']].values
    vel_res_post_fit = df[['y_k13', 'y_k14', 'y_k15', 'y_k16', 'y_k17', 'y_k18', 'y_k19', 'y_k20', 'y_k21', 'y_k22', 'y_k23', 'y_k24']].values
    foot_pos_z_res_post_fit = df[['y_k25', 'y_k26', 'y_k27', 'y_k28']].values

    swing_phase = df['swing_phase'].values

    # Plot data

    ##################
    ## FR pos + vel ##
    ##################

    FR_pos_vel_plot = plt.figure(1, figsize=(16, 9))

    plt.subplot(3, 2, 1)
    shade_by_phase(plt.subplot(3, 2, 1), time, swing_phase)
    plt.plot(time, foot_pos_meas[:, 0], label="Measured", color='b')
    plt.plot(time, foot_pos_pred[:, 0], label="Predicted", color='g')
    plt.plot(time, pos_res_post_fit[:, 0], label="Post Fit", color='r')
    plt.title("FR x position")
    plt.xlabel("Time (s)")
    plt.ylabel("X position (m)")
    plt.xlim(0, time[-1])
    plt.legend()

    plt.subplot(3, 2, 2)
    shade_by_phase(plt.subplot(3, 2, 2), time, swing_phase)
    plt.plot(time, foot_vel_meas[:, 0], label="Measured", color='b')
    plt.plot(time, foot_vel_pred[:, 0], label="Predicted", color='g')
    plt.plot(time, vel_res_post_fit[:, 0], label="Post Fit", color='r')
    plt.title("FR x velocity")
    plt.xlabel("Time (s)")
    plt.ylabel("X vel (m/s)")
    plt.xlim(0, time[-1])
    plt.legend()

    plt.subplot(3, 2, 3)
    shade_by_phase(plt.subplot(3, 2, 3), time, swing_phase)
    plt.plot(time, foot_pos_meas[:, 1], label="Measured", color='b')
    plt.plot(time, foot_pos_pred[:, 1], label="Predicted", color='g')
    plt.plot(time, pos_res_post_fit[:, 1], label="Post Fit", color='r')
    plt.title("FR y position")
    plt.xlabel("Time (s)")
    plt.ylabel("Y position (m)")
    plt.xlim(0, time[-1])
    plt.legend()

    plt.subplot(3, 2, 4)
    shade_by_phase(plt.subplot(3, 2, 4), time, swing_phase)
    plt.plot(time, foot_vel_meas[:, 1], label="Measured", color='b')
    plt.plot(time, foot_vel_pred[:, 1], label="Predicted", color='g')
    plt.plot(time, vel_res_post_fit[:, 1], label="Post Fit", color='r')
    plt.title("FR y velocity")
    plt.xlabel("Time (s)")
    plt.ylabel("Y vel (m/s)")
    plt.xlim(0, time[-1])
    plt.legend()

    plt.subplot(3, 2, 5)
    shade_by_phase(plt.subplot(3, 2, 5), time, swing_phase)
    plt.plot(time, foot_pos_meas[:, 2], label="Measured", color='b')
    plt.plot(time, foot_pos_pred[:, 2], label="Predicted", color='g')
    plt.plot(time, pos_res_post_fit[:, 2], label="Post Fit", color='r')
    plt.title("FR z position")
    plt.xlabel("Time (s)")
    plt.ylabel("Z position (m)")
    plt.xlim(0, time[-1])
    plt.legend()

    plt.subplot(3, 2, 6)
    shade_by_phase(plt.subplot(3, 2, 6), time, swing_phase)
    plt.plot(time, foot_vel_meas[:, 2], label="Measured", color='b')
    plt.plot(time, foot_vel_pred[:, 2], label="Predicted", color='g')
    plt.plot(time, vel_res_post_fit[:, 2], label="Post Fit", color='r')
    plt.title("FR z velocity")
    plt.xlabel("Time (s)")
    plt.ylabel("Z vel (m/s)")
    plt.xlim(0, time[-1])
    plt.legend()

    plt.tight_layout()
    plt.savefig("data/estimator_tuning/FR_pos_vel.png")

    ##################
    ## FL pos + vel ##
    ##################

    FL_pos_vel_plot = plt.figure(2, figsize=(16, 9))

    plt.subplot(3, 2, 1)
    shade_by_phase(plt.subplot(3, 2, 1), time, swing_phase)
    plt.plot(time, foot_pos_meas[:, 3], label="Measured", color='b')
    plt.plot(time, foot_pos_pred[:, 3], label="Predicted", color='g')
    plt.plot(time, pos_res_post_fit[:, 3], label="Post Fit", color='r')
    plt.title("FL x position")
    plt.xlabel("Time (s)")
    plt.ylabel("X position (m)")
    plt.xlim(0, time[-1])
    plt.legend()

    plt.subplot(3, 2, 2)
    shade_by_phase(plt.subplot(3, 2, 2), time, swing_phase)
    plt.plot(time, foot_vel_meas[:, 3], label="Measured", color='b')
    plt.plot(time, foot_vel_pred[:, 3], label="Predicted", color='g')
    plt.plot(time, vel_res_post_fit[:, 3], label="Post Fit", color='r')
    plt.title("FL x velocity")
    plt.xlabel("Time (s)")
    plt.ylabel("X vel (m/s)")
    plt.xlim(0, time[-1])
    plt.legend()

    plt.subplot(3, 2, 3)
    shade_by_phase(plt.subplot(3, 2, 3), time, swing_phase)
    plt.plot(time, foot_pos_meas[:, 4], label="Measured", color='b')
    plt.plot(time, foot_pos_pred[:, 4], label="Predicted", color='g')
    plt.plot(time, pos_res_post_fit[:, 4], label="Post Fit", color='r')
    plt.title("FL y position")
    plt.xlabel("Time (s)")
    plt.ylabel("Y position (m)")
    plt.xlim(0, time[-1])
    plt.legend()

    plt.subplot(3, 2, 4)
    shade_by_phase(plt.subplot(3, 2, 4), time, swing_phase)
    plt.plot(time, foot_vel_meas[:, 4], label="Measured", color='b')
    plt.plot(time, foot_vel_pred[:, 4], label="Predicted", color='g')
    plt.plot(time, vel_res_post_fit[:, 4], label="Post Fit", color='r')
    plt.title("FL y velocity")
    plt.xlabel("Time (s)")
    plt.ylabel("Y vel (m/s)")
    plt.xlim(0, time[-1])
    plt.legend()

    plt.subplot(3, 2, 5)
    shade_by_phase(plt.subplot(3, 2, 5), time, swing_phase)
    plt.plot(time, foot_pos_meas[:, 5], label="Measured", color='b')
    plt.plot(time, foot_pos_pred[:, 5], label="Predicted", color='g')
    plt.plot(time, pos_res_post_fit[:, 5], label="Post Fit", color='r')
    plt.title("FL z position")
    plt.xlabel("Time (s)")
    plt.ylabel("Z position (m)")
    plt.xlim(0, time[-1])
    plt.legend()

    plt.subplot(3, 2, 6)
    shade_by_phase(plt.subplot(3, 2, 6), time, swing_phase)
    plt.plot(time, foot_vel_meas[:, 5], label="Measured", color='b')
    plt.plot(time, foot_vel_pred[:, 5], label="Predicted", color='g')
    plt.plot(time, vel_res_post_fit[:, 5], label="Post Fit", color='r')
    plt.title("FL z velocity")
    plt.xlabel("Time (s)")
    plt.ylabel("Z vel (m/s)")
    plt.xlim(0, time[-1])
    plt.legend()

    plt.tight_layout()
    plt.savefig("data/estimator_tuning/FL_pos_vel.png")

    ##################
    ## RR pos + vel ##
    ##################

    RR_pos_vel_plot = plt.figure(3, figsize=(16, 9))

    plt.subplot(3, 2, 1)
    shade_by_phase(plt.subplot(3, 2, 1), time, swing_phase)
    plt.plot(time, foot_pos_meas[:, 6], label="Measured", color='b')
    plt.plot(time, foot_pos_pred[:, 6], label="Predicted", color='g')
    plt.plot(time, pos_res_post_fit[:, 6], label="Post Fit", color='r')
    plt.title("RR x position")
    plt.xlabel("Time (s)")
    plt.ylabel("X position (m)")
    plt.xlim(0, time[-1])
    plt.legend()

    plt.subplot(3, 2, 2)
    shade_by_phase(plt.subplot(3, 2, 2), time, swing_phase)
    plt.plot(time, foot_vel_meas[:, 6], label="Measured", color='b')
    plt.plot(time, foot_vel_pred[:, 6], label="Predicted", color='g')
    plt.plot(time, vel_res_post_fit[:, 6], label="Post Fit", color='r')
    plt.title("RR x velocity")
    plt.xlabel("Time (s)")
    plt.ylabel("X vel (m/s)")
    plt.xlim(0, time[-1])
    plt.legend()

    plt.subplot(3, 2, 3)
    shade_by_phase(plt.subplot(3, 2, 3), time, swing_phase)
    plt.plot(time, foot_pos_meas[:, 7], label="Measured", color='b')
    plt.plot(time, foot_pos_pred[:, 7], label="Predicted", color='g')
    plt.plot(time, pos_res_post_fit[:, 7], label="Post Fit", color='r')
    plt.title("RR y position")
    plt.xlabel("Time (s)")
    plt.ylabel("Y position (m)")
    plt.xlim(0, time[-1])
    plt.legend()

    plt.subplot(3, 2, 4)
    shade_by_phase(plt.subplot(3, 2, 4), time, swing_phase)
    plt.plot(time, foot_vel_meas[:, 7], label="Measured", color='b')
    plt.plot(time, foot_vel_pred[:, 7], label="Predicted", color='g')
    plt.plot(time, vel_res_post_fit[:, 7], label="Post Fit", color='r')
    plt.title("RR y velocity")
    plt.xlabel("Time (s)")
    plt.ylabel("Y vel (m/s)")
    plt.xlim(0, time[-1])
    plt.legend()

    plt.subplot(3, 2, 5)
    shade_by_phase(plt.subplot(3, 2, 5), time, swing_phase)
    plt.plot(time, foot_pos_meas[:, 8], label="Measured", color='b')
    plt.plot(time, foot_pos_pred[:, 8], label="Predicted", color='g')
    plt.plot(time, pos_res_post_fit[:, 8], label="Post Fit", color='r')
    plt.title("RR z position")
    plt.xlabel("Time (s)")
    plt.ylabel("Z position (m)")
    plt.xlim(0, time[-1])
    plt.legend()

    plt.subplot(3, 2, 6)
    shade_by_phase(plt.subplot(3, 2, 6), time, swing_phase)
    plt.plot(time, foot_vel_meas[:, 8], label="Measured", color='b')
    plt.plot(time, foot_vel_pred[:, 8], label="Predicted", color='g')
    plt.plot(time, vel_res_post_fit[:, 8], label="Post Fit", color='r')
    plt.title("RR z velocity")
    plt.xlabel("Time (s)")
    plt.ylabel("Z vel (m/s)")
    plt.xlim(0, time[-1])
    plt.legend()

    plt.tight_layout()
    plt.savefig("data/estimator_tuning/RR_pos_vel.png")

    ##################
    ## RL pos + vel ##
    ##################

    RL_pos_vel_plot = plt.figure(4, figsize=(16, 9))

    plt.subplot(3, 2, 1)
    shade_by_phase(plt.subplot(3, 2, 1), time, swing_phase)
    plt.plot(time, foot_pos_meas[:, 9], label="Measured", color='b')
    plt.plot(time, foot_pos_pred[:, 9], label="Predicted", color='g')
    plt.plot(time, pos_res_post_fit[:, 9], label="Post Fit", color='r')
    plt.title("RL x position")
    plt.xlabel("Time (s)")
    plt.ylabel("X position (m)")
    plt.xlim(0, time[-1])
    plt.legend()

    plt.subplot(3, 2, 2)
    shade_by_phase(plt.subplot(3, 2, 2), time, swing_phase)
    plt.plot(time, foot_vel_meas[:, 9], label="Measured", color='b')
    plt.plot(time, foot_vel_pred[:, 9], label="Predicted", color='g')
    plt.plot(time, vel_res_post_fit[:, 9], label="Post Fit", color='r')
    plt.title("RL x velocity")
    plt.xlabel("Time (s)")
    plt.ylabel("X vel (m/s)")
    plt.xlim(0, time[-1])
    plt.legend()

    plt.subplot(3, 2, 3)
    shade_by_phase(plt.subplot(3, 2, 3), time, swing_phase)
    plt.plot(time, foot_pos_meas[:, 10], label="Measured", color='b')
    plt.plot(time, foot_pos_pred[:, 10], label="Predicted", color='g')
    plt.plot(time, pos_res_post_fit[:, 10], label="Post Fit", color='r')
    plt.title("RL y position")
    plt.xlabel("Time (s)")
    plt.ylabel("Y position (m)")
    plt.xlim(0, time[-1])
    plt.legend()

    plt.subplot(3, 2, 4)
    shade_by_phase(plt.subplot(3, 2, 4), time, swing_phase)
    plt.plot(time, foot_vel_meas[:, 10], label="Measured", color='b')
    plt.plot(time, foot_vel_pred[:, 10], label="Predicted", color='g')
    plt.plot(time, vel_res_post_fit[:, 10], label="Post Fit", color='r')
    plt.title("RL y velocity")
    plt.xlabel("Time (s)")
    plt.ylabel("Y vel (m/s)")
    plt.xlim(0, time[-1])
    plt.legend()

    plt.subplot(3, 2, 5)
    shade_by_phase(plt.subplot(3, 2, 5), time, swing_phase)
    plt.plot(time, foot_pos_meas[:, 11], label="Measured", color='b')
    plt.plot(time, foot_pos_pred[:, 11], label="Predicted", color='g')
    plt.plot(time, pos_res_post_fit[:, 11], label="Post Fit", color='r')
    plt.title("RL z position")
    plt.xlabel("Time (s)")
    plt.ylabel("Z position (m)")
    plt.xlim(0, time[-1])
    plt.legend()

    plt.subplot(3, 2, 6)
    shade_by_phase(plt.subplot(3, 2, 6), time, swing_phase)
    plt.plot(time, foot_vel_meas[:, 11], label="Measured", color='b')
    plt.plot(time, foot_vel_pred[:, 11], label="Predicted", color='g')
    plt.plot(time, vel_res_post_fit[:, 11], label="Post Fit", color='r')
    plt.title("RL z velocity")
    plt.xlabel("Time (s)")
    plt.ylabel("Z vel (m/s)")
    plt.xlim(0, time[-1])
    plt.legend()

    plt.tight_layout()
    plt.savefig("data/estimator_tuning/RL_pos_vel.png")

    ####################
    ## Foot z heights ##
    ####################

    foot_z_plot = plt.figure(5, figsize=(16, 9))

    plt.subplot(2, 2, 1)
    shade_by_phase(plt.subplot(2, 2, 1), time, swing_phase)
    plt.plot(time, foot_pos_z_meas[:, 0], label="Measured", color='b')
    plt.plot(time, foot_pos_z_pred[:, 0], label="Predicted", color='g')
    plt.plot(time, foot_pos_z_res_post_fit[:, 0], label="Post Fit", color='r')
    plt.title("FR z position")
    plt.xlabel("Time (s)")
    plt.ylabel("Z position (m)")
    plt.xlim(0, time[-1])
    plt.legend()

    plt.subplot(2, 2, 2)
    shade_by_phase(plt.subplot(2, 2, 2), time, swing_phase)
    plt.plot(time, foot_pos_z_meas[:, 1], label="Measured", color='b')
    plt.plot(time, foot_pos_z_pred[:, 1], label="Predicted", color='g')
    plt.plot(time, foot_pos_z_res_post_fit[:, 1], label="Post Fit", color='r')
    plt.title("FL z position")
    plt.xlabel("Time (s)")
    plt.ylabel("Z position (m)")
    plt.xlim(0, time[-1])
    plt.legend()

    plt.subplot(2, 2, 3)
    shade_by_phase(plt.subplot(2, 2, 3), time, swing_phase)
    plt.plot(time, foot_pos_z_meas[:, 2], label="Measured", color='b')
    plt.plot(time, foot_pos_z_pred[:, 2], label="Predicted", color='g')
    plt.plot(time, foot_pos_z_res_post_fit[:, 2], label="Post Fit", color='r')
    plt.title("RR z position")
    plt.xlabel("Time (s)")
    plt.ylabel("Z position (m)")
    plt.xlim(0, time[-1])
    plt.legend()

    plt.subplot(2, 2, 4)
    shade_by_phase(plt.subplot(2, 2, 4), time, swing_phase)
    plt.plot(time, foot_pos_z_meas[:, 3], label="Measured", color='b')
    plt.plot(time, foot_pos_z_pred[:, 3], label="Predicted", color='g')
    plt.plot(time, foot_pos_z_res_post_fit[:, 3], label="Post Fit", color='r')
    plt.title("RL z position")
    plt.xlabel("Time (s)")
    plt.ylabel("Z position (m)")
    plt.xlim(0, time[-1])
    plt.legend()

    plt.tight_layout()
    plt.savefig("data/estimator_tuning/foot_z_pos.png")

    plt.show()

csv_file = "data/go1_estimator_data.csv"
plot_state_data(csv_file)