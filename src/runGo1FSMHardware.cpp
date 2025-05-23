/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <ncurses.h>
#include <stdio.h>
#include <stdint.h>

#include "go1_cpp_cmake/go1FSM.h"
#include "go1_cpp_cmake/go1StateEstimator.h"

using namespace std;
using namespace UNITREE_LEGGED_SDK;

// Global objects for hardware interfacing
LowState lowState = {0};
UDP udp(LOWLEVEL, 8090, "192.168.123.10", 8007);
LowCmd lowCmd = {0};

///////////////////////////////
// Data collection functions //
///////////////////////////////

// Data logging objects
ostringstream hardware_datastream;
SyncLogger data_log("../data/go1_hardware_data.csv", hardware_datastream.str());
ostringstream hardware_data_row;

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
            "root_ang_vel_est_x,root_ang_vel_est_y,root_ang_vel_est_z,root_ang_vel_meas_x,root_ang_vel_meas_y,root_ang_vel_meas_z,root_ang_vel_d_x,root_ang_vel_d_y,root_ang_vel_d_z,"            "FR_0,FR_1,FR_2,FL_0,FL_1,FL_2,RR_0,RR_1,RR_2,RL_0,RL_1,RL_2,"
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

// External controller variables
bool running = false;
bool keyboardInit = false;

void keyboardControl(go1FSM &fsm) {
/*
    Keyboard terminal commands for sending desired velocities to
    the physical Go1 robot. Can't continuously rotate, and the
    swing leg control is iffy atm.
*/
    // Initialize keyboard input w/ ncurses
    if (!keyboardInit) {
        initscr();
        timeout(0);
        nodelay(stdscr, TRUE);
        noecho();
        cbreak();
        keyboardInit = true;
    }

    static bool hold_forward = false;
    static bool hold_backward = false;
    static bool hold_left = false;
    static bool hold_right = false;
    static bool hold_yawL = false;
    static bool hold_yawR = false;

    int ch;
    while ((ch = getch()) != ERR) {
        switch(ch) {

            ///////////////////////
            // Movement commands //
            ///////////////////////

            case 'w': 
                if (fsm.getState().walking_mode)
                    hold_forward = true; 
                    break;

            case 's': 
                if (fsm.getState().walking_mode)
                    hold_backward = true; 
                    break;

            case 'a': 
                if (fsm.getState().walking_mode)
                    hold_left = true; 
                    break;

            case 'd': 
                if (fsm.getState().walking_mode)
                    hold_right = true; 
                    break;

            case 'q': 
                if (fsm.getState().walking_mode)
                    hold_yawL = true; 
                    break;

            case 'e': 
                if (fsm.getState().walking_mode)
                    hold_yawR = true; 
                    break;

            case 'x': // stop linear movement
                hold_forward = hold_backward = false;
                hold_left = hold_right = false;
                break;

            case 'z': // stop yaw movement
                hold_yawL = hold_yawR = false;
                break;

            ///////////////////////////////////
            // Finite state machine commands //
            ///////////////////////////////////

            case 'p':
                fsm.requestWalk(true);
                hold_forward = hold_backward = false;
                hold_left = hold_right = false;
                hold_yawL = hold_yawR = false;
                break;

            case 'o':
                fsm.requestWalk(false);
                hold_forward = hold_backward = false;
                hold_left = hold_right = false;
                hold_yawL = hold_yawR = false;
                break;
            
            case 'i':
                fsm.requestShutdown();
                hold_forward = hold_backward = false;
                hold_left = hold_right = false;
                hold_yawL = hold_yawR = false;
                break;

            case 'u':
                fsm.requestStartup();
                hold_forward = hold_backward = false;
                hold_left = hold_right = false;
                hold_yawL = hold_yawR = false;
                break;

            case ' ':
                running = false;
                mvprintw(0, 0, "Killing controller...");
                refresh();
                endwin();
                std::cout << "Controller killed, exiting..." << std::endl;
                break;

            default:
                break;
        }
    }

    Eigen::Vector3d v_cmd = Eigen::Vector3d::Zero();
    double yaw_cmd = 0;

    if (hold_forward) v_cmd.x() += 0.2;
    if (hold_backward) v_cmd.x() -= 0.2;
    if (hold_left) v_cmd.y() += 0.2;
    if (hold_right) v_cmd.y() -= 0.2;
    if (hold_yawL) yaw_cmd += 1.0;
    if (hold_yawR) yaw_cmd -= 1.0;

    double yaw = fsm.getState().root_rpy_est(2);
    Eigen::Matrix3d rootRotZ = rotZ(yaw);
    Eigen::Vector3d v_world = rootRotZ * v_cmd;

    fsm.setDesiredVel(v_world, yaw_cmd);
    fsm.setDesiredPos();

    // Debug output
    mvprintw(0, 0,
        "v: [%.2f, %.2f], yaw_rate: %.2f, FSM state: %s",
        v_cmd.x(), v_cmd.y(), yaw_cmd, fsm.go1FiniteState2Str()
        );
    refresh();
}

void receiveAndSend() {
    udp.Recv();
    udp.GetRecv(lowState);
    udp.SetSend(lowCmd);
    udp.Send();
}

void recordLowLevel(const go1State &state) {
    hardware_data_row.str("");
    hardware_data_row.clear();
    storeData(state, hardware_data_row);
    data_log.logLine(hardware_data_row.str());
}

int main(void) {
    // Prep data collection
    writeCSVHeader(hardware_datastream);
    data_log.logLine(hardware_datastream.str());

    // Initialize data interface + estimator
    auto data_src = make_unique<hardwareDataReader>(lowState, udp);
    auto command_sender = make_unique<hardwareCommandSender>(lowState, udp, lowCmd);
    auto estimator = makeEstimator();

    // Instantiate FSM at DT_CTRL for overall loop rate, DT_MPC_CTRL for MPC loop rate
    go1FSM fsm(DT_CTRL, DT_MPC_CTRL, move(data_src), move(estimator), move(command_sender));

    std::cout << "Hardware control is initialized..." << std::endl;
    running = true;

    // Main loop: construct LoopFunc objects and run until killed by keyboard input [spacebar]
    LoopFunc loop_keyInput("key_input_loop", DT_CTRL, 4, boost::bind(&keyboardControl, boost::ref(fsm)));
    LoopFunc loop_record("recording_loop", DT_CTRL, 5, boost::bind(&recordLowLevel, boost::ref(fsm.getState())));
    LoopFunc loop_ctrl("fsm_ctrl_loop", DT_CTRL, 6, boost::bind(&go1FSM::step, &fsm));
    
    // construct and start UDP send and receive loop first to collect first valid packages
    LoopFunc loop_recvSend("udp_recvSend", DT_CTRL, 3, boost::bind(&receiveAndSend));
    loop_recvSend.start();

    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // helps with initial lag

    fsm.collectInitialState();
    fsm.step();
    storeData(fsm.getState(), hardware_data_row);
    data_log.logLine(hardware_data_row.str());
    
    loop_ctrl.start();
    loop_keyInput.start();
    loop_record.start();

    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    };

    // Cleanup
    loop_record.shutdown();
    loop_recvSend.shutdown();
    loop_ctrl.shutdown();
    loop_keyInput.shutdown();

    endwin();
    std::cout << "Hardware code cleaned up!" << std::endl;

    return 0;
}
