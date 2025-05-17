/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <ncurses.h>

#include "go1_cpp_cmake/go1DataInterface.h"

using namespace std;
using namespace UNITREE_LEGGED_SDK;

// Global objects for hardware interfacing
LowState lowState = {0};
UDP udp(LOWLEVEL, 8090, "192.168.123.10", 8007);
LowCmd lowCmd = {0};
go1State tester_state;
auto data_src = std::make_unique<hardwareDataReader>(lowState, udp);
auto command_sender = std::make_unique<hardwareCommandSender>(lowState, udp, lowCmd);

// Data logging functions
std::ostringstream hardware_datastream;
AsyncLogger data_log("../data/go1_hardware_data.csv", hardware_datastream.str());
std::ostringstream hardware_data_row;

///////////////////////////////
// Data collection functions //
///////////////////////////////

// Function to write Eigen::Vector in CSV format
template <typename VectorType>
void write_vector(const Eigen::MatrixBase<VectorType> &vec, std::ostream &os) {
    for (size_t i = 0; i < vec.size(); ++i) {
        os << vec(i);
        if (i < vec.size() - 1) {
            os << ",";  // Add comma between elements
        }
    }
}

// Functions to store data in CSV file
void storeData(const go1State &state, std::ostream &os) {
    // Position
    write_vector(state.root_pos_est, os); os << ",";
    write_vector(state.root_pos_d, os); os << ",";

    // Linear velocity
    write_vector(state.root_lin_vel_est, os); os << ",";
    write_vector(state.root_lin_vel_d, os); os << ",";

    // Linear acceleration
    write_vector(state.root_lin_acc_est, os); os << ",";
    write_vector(state.root_lin_acc_meas, os); os << ",";

    // RPY orientation
    write_vector(state.root_rpy_est, os); os << ",";
    write_vector(state.root_rpy_d, os); os << ",";

    // Angular velocity
    write_vector(state.root_ang_vel_est, os); os << ",";
    write_vector(state.root_ang_vel_meas, os); os << ",";
    write_vector(state.root_ang_vel_d, os); os << ",";    

    // Joint states
    write_vector(state.joint_pos, os); os << ",";
    write_vector(state.joint_pos_d, os); os << ",";
    write_vector(state.joint_vel, os); os << ",";
    write_vector(state.joint_vel_d, os); os << ",";
 
    // Foot positions and velocities
    write_vector(state.foot_pos_world_rot, os); os << ",";
    write_vector(state.foot_pos_abs, os); os << ",";
    write_vector(state.foot_pos_liftoff, os); os << ",";
    write_vector(state.foot_pos_d, os); os << ",";
    write_vector(state.foot_vel_d, os); os << ",";

    // Foot forces
    {
        Eigen::Map<const Eigen::VectorXd> flat_grf(
            state.foot_forces_grf.data(),
            state.foot_forces_grf.size()
        );
        write_vector(flat_grf, os);
    }

    os << ',';
    
    {
        Eigen::Map<const Eigen::VectorXd> flat_swing(
            state.foot_forces_swing.data(),
            state.foot_forces_swing.size()
        );
        write_vector(flat_swing, os);
    }

    os << ',';
    
    // Joint torques
    {
        Eigen::Map<const Eigen::VectorXd> flat_torque(
            state.joint_torques.data(),
            state.joint_torques.size()
        );
        write_vector(flat_torque, os);
    }

    os << ',';

    // Foot contacts
    os << state.swing_phase << "," 
        << (state.contacts[0] ? 1 : 0) << "," 
        << (state.contacts[1] ? 1 : 0) << "," 
        << (state.contacts[2] ? 1 : 0) << "," 
        << (state.contacts[3] ? 1 : 0) << ",";
    write_vector(state.est_contacts, os); os << "\n";
    
}

void writeCSVHeader(std::ostream &os) {    
    os << "root_pos_est_x,root_pos_est_y,root_pos_est_z,root_pos_d_x,root_pos_d_y,root_pos_d_z,"
            "root_lin_vel_est_x,root_lin_vel_est_y,root_lin_vel_est_z,root_lin_vel_d_x,root_lin_vel_d_y,root_lin_vel_d_z,"
            "root_lin_acc_est_x,root_lin_acc_est_y,root_lin_acc_est_z,root_lin_acc_meas_x,root_lin_acc_meas_y,root_lin_acc_meas_z,"
            "root_rpy_est_x,root_rpy_est_y,root_rpy_est_z,root_rpy_d_x,root_rpy_d_y,root_rpy_d_z,"
            "root_ang_vel_est_x,root_ang_vel_est_y,root_ang_vel_est_z,root_ang_vel_meas_x,root_ang_vel_meas_y,root_ang_vel_meas_z,root_ang_vel_d_x,root_ang_vel_d_y,root_ang_vel_d_z,"
            "FR_0,FR_1,FR_2,FL_0,FL_1,FL_2,RR_0,RR_1,RR_2,RL_0,RL_1,RL_2,"
            "FR_0_des,FR_1_des,FR_2_des,FL_0_des,FL_1_des,FL_2_des,RR_0_des,RR_1_des,RR_2_des,RL_0_des,RL_1_des,RL_2_des,"
            "FR_0_dot,FR_1_dot,FR_2_dot,FL_0_dot,FL_1_dot,FL_2_dot,RR_0_dot,RR_1_dot,RR_2_dot,RL_0_dot,RL_1_dot,RL_2_dot,"
            "FR_0_dot_des,FR_1_dot_des,FR_2_dot_des,FL_0_dot_des,FL_1_dot_des,FL_2_dot_des,RR_0_dot_des,RR_1_dot_des,RR_2_dot_des,RL_0_dot_des,RL_1_dot_des,RL_2_dot_des,"
            "foot_pos_FR_x,foot_pos_FR_y,foot_pos_FR_z,foot_pos_FL_x,foot_pos_FL_y,foot_pos_FL_z,"
            "foot_pos_RR_x,foot_pos_RR_y,foot_pos_RR_z,foot_pos_RL_x,foot_pos_RL_y,foot_pos_RL_z,"
            "foot_pos_abs_FR_x,foot_pos_abs_FR_y,foot_pos_abs_FR_z,foot_pos_abs_FL_x,foot_pos_abs_FL_y,foot_pos_abs_FL_z,"
            "foot_pos_abs_RR_x,foot_pos_abs_RR_y,foot_pos_abs_RR_z,foot_pos_abs_RL_x,foot_pos_abs_RL_y,foot_pos_abs_RL_z,"
            "foot_pos_liftoff_FR_x,foot_pos_liftoff_FR_y,foot_pos_liftoff_FR_z,foot_pos_liftoff_FL_x,foot_pos_liftoff_FL_y,foot_pos_liftoff_FL_z,"
            "foot_pos_liftoff_RR_x,foot_pos_liftoff_RR_y,foot_pos_liftoff_RR_z,foot_pos_liftoff_RL_x,foot_pos_liftoff_RL_y,foot_pos_liftoff_RL_z,"
            "foot_pos_d_FR_x,foot_pos_d_FR_y,foot_pos_d_FR_z,foot_pos_d_FL_x,foot_pos_d_FL_y,foot_pos_d_FL_z,"
            "foot_pos_d_RR_x,foot_pos_d_RR_y,foot_pos_d_RR_z,foot_pos_d_RL_x,foot_pos_d_RL_y,foot_pos_d_RL_z,"
            "foot_vel_d_FR_x,foot_vel_d_FR_y,foot_vel_d_FR_z,foot_vel_d_FL_x,foot_vel_d_FL_y,foot_vel_d_FL_z,"
            "foot_vel_d_RR_x,foot_vel_d_RR_y,foot_vel_d_RR_z,foot_vel_d_RL_x,foot_vel_d_RL_y,foot_vel_d_RL_z,"
            "foot_forces_grf_FR_x,foot_forces_grf_FR_y,foot_forces_grf_FR_z,foot_forces_grf_FL_x,foot_forces_grf_FL_y,foot_forces_grf_FL_z,"
            "foot_forces_grf_RR_x,foot_forces_grf_RR_y,foot_forces_grf_RR_z,foot_forces_grf_RL_x,foot_forces_grf_RL_y,foot_forces_grf_RL_z,"
            "foot_forces_swing_FR_x,foot_forces_swing_FR_y,foot_forces_swing_FR_z,foot_forces_swing_FL_x,foot_forces_swing_FL_y,foot_forces_swing_FL_z,"
            "foot_forces_swing_RR_x,foot_forces_swing_RR_y,foot_forces_swing_RR_z,foot_forces_swing_RL_x,foot_forces_swing_RL_y,foot_forces_swing_RL_z,"
            "joint_torques_FR_hip,joint_torques_FR_thigh,joint_torques_FR_calf,joint_torques_FL_hip,joint_torques_FL_thigh,joint_torques_FL_calf,"
            "joint_torques_RR_hip,joint_torques_RR_thigh,joint_torques_RR_calf,joint_torques_RL_hip,joint_torques_RL_thigh,joint_torques_RL_calf,"
            "swing_phase,contact_FR,contact_FL,contact_RR,contact_RL,"
            "FR_contact_meas,FL_contact_meas,RR_contact_meas,RL_contact_meas\n";

}

////////////////////////////
// Main control functions //
////////////////////////////

// Keyboard control variable
bool keyboardInit = false;
bool startup = true;

void keyboardControl() {
/*
    Keyboard terminal commands for sending desired velocities to
    the physical Go1 robot. Can't continuously rotate, and the
    swing leg control is iffy atm.
*/
    if (!keyboardInit) {
        initscr();
        timeout(0);
        nodelay(stdscr, TRUE);
        noecho();
        cbreak();
        keyboardInit = true;
    }

    int ch = getch();

    if (ch != ERR) {
        switch (ch) {
            case 'p':
                startup = !startup;
                break;
        }
    }
    move(0, 0);
    printw("squat: %i, squat_prog: %f\n", startup, tester_state.squat_prog);
    refresh();
}

void runStartupPDHardware() {
    data_src->pullSensorData(tester_state);
    
    if (startup) tester_state.computeStartupPD();
    else tester_state.computeShutdownPD();

    command_sender->setCommand(tester_state);
}

void receiveAndSend() {
    udp.Recv();
    udp.Send();
}

void recordLowLevel() {
    hardware_data_row.str("");
    hardware_data_row.clear();
    storeData(tester_state, hardware_data_row);
    data_log.logLine(hardware_data_row.str());
}

int main(void) {
    writeCSVHeader(hardware_datastream);
    data_log.logLine(hardware_datastream.str());

    std::cout << "Communication level is set to LOW-level." << std::endl
            << "WARNING: Make sure the robot is hung up." << std::endl
            << "NOTE: The robot also needs to be set to LOW-level mode, otherwise it will make strange noises and this example will not run successfully! " << std::endl
            << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    LoopFunc loop_stand("squat_loop", DT_CTRL, boost::bind(&runStartupPDHardware));
    LoopFunc loop_keyInput("key_input_loop", DT_CTRL, 4, boost::bind(&keyboardControl));
    LoopFunc loop_recvSend("udp_recvSend", DT_CTRL, 3, boost::bind(&receiveAndSend));
    LoopFunc loop_record("recording_loop", DT_CTRL, boost::bind(&recordLowLevel));

    loop_recvSend.start();

    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // helps with initial lag of loading data into go1State properly
    
    loop_keyInput.start();
    loop_stand.start();
    loop_record.start();

    while (1) {
        sleep(10);
    };

    // Cleanup
    loop_record.shutdown();
    loop_stand.shutdown();
    loop_keyInput.shutdown();
    loop_recvSend.shutdown();

    return 0;
}
