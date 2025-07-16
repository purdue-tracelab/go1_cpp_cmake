#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
# from mpl_toolkits.mplot3d import Axes3D

def plot_calc_time(csv_file):
    # Read the CSV file into a DataFrame
    df = pd.read_csv(csv_file)
    data_length = len(df)
    time = np.arange(0, data_length * 0.002, 0.002)

    # Extract the relevant columns
    state_update_times = df['state_update_time']
    state_estimation_times = df['estimation_time']
    mpc_calc_times = df['MPC_calc_time']

    # Plot data
    calc_time_plot = plt.figure(figsize=(10, 6))

    plt.subplot(2, 2, 1)
    plt.plot(time, state_update_times, label='Footstep Planning + State Update Time', color='blue')
    plt.xlabel('Simulation time (s)')
    plt.ylabel('Calculation time (ms)')
    plt.legend()

    plt.subplot(2, 2, 2)
    plt.plot(time, state_estimation_times, label='State Estimation Time', color='green')
    plt.xlabel('Simulation time (s)')
    plt.ylabel('Calculation time (ms)')
    plt.legend()

    plt.subplot(2, 2, (3, 4))
    plt.plot(time, mpc_calc_times, label='MPC Calculation Time', color='red')
    plt.xlabel('Simulation time (s)')
    plt.ylabel('Calculation time (ms)')
    plt.legend()

    # Show the plot
    plt.tight_layout()
    plt.savefig("data/calc_times/sim_calc_times.png")
    plt.show()

csv_file = 'data/go1_mujoco_calc_time.csv'
plot_calc_time(csv_file)