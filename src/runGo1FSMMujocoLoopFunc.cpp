/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
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

/////////////////////////////
// MuJoCo simulation setup //
/////////////////////////////

// Global variables for MuJoCo simulation
static mjModel* mujoco_model = nullptr;
static mjData* mujoco_data = nullptr;
static mjvCamera cam;
static mjvOption opt;
static mjvScene scn;
static mjrContext con;
static GLFWwindow* window = nullptr;
int window_width = 1600, window_height = 900;  // Default window size

// Mouse state variables
bool button_left = false, button_middle = false, button_right = false;
double lastx = 0, lasty = 0;

// GLFW window resize callback
void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    window_width = width;
    window_height = height;
    // mjr_setBuffer(mjFB_WINDOW, &con);

    glViewport(0, 0, width, height);
    mjr_setBuffer(mjFB_WINDOW, &con);
}

// GLFW mouse button callback
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
}

// GLFW scroll callback (Zoom)
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    cam.distance *= (1.0 - 0.05 * yoffset);
}

// GLFW cursor position callback (Rotate/Pan)
void cursor_position_callback(GLFWwindow* window, double xpos, double ypos) {
    if (!button_left && !button_middle && !button_right)
        return;

    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    if (button_left) {
        cam.azimuth += dx * 0.5;
        cam.elevation -= dy * 0.5;
    }
    if (button_right) {
        cam.distance *= (1.0 + dy * 0.01);
    }
    if (button_middle) {
        cam.lookat[0] += dx * 0.01;
        cam.lookat[1] -= dy * 0.01;
    }
}

// GLFW window close callback
void window_close_callback(GLFWwindow* window) {
    glfwSetWindowShouldClose(window, GLFW_TRUE);
}

// Function to initialize visualization
void initVisualization() {
    if (!glfwInit()) {
        std::cerr << "Could not initialize GLFW!" << std::endl;
        exit(1);
    }

    window = glfwCreateWindow(window_width, window_height, "MuJoCo Simulation", NULL, NULL);
    if (!window) {
        std::cerr << "Could not create GLFW window!" << std::endl;
        glfwTerminate();
        exit(1);
    }

    glfwMakeContextCurrent(window);
    glfwSetWindowCloseCallback(window, window_close_callback);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetCursorPosCallback(window, cursor_position_callback);

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_makeScene(mujoco_model, &scn, 2000);

    mjr_defaultContext(&con);
    mjr_makeContext(mujoco_model, &con, mjFONTSCALE_150);

    glfwGetFramebufferSize(window, &window_width, &window_height);
    glViewport(0, 0, window_width, window_height);
    mjr_setBuffer(mjFB_WINDOW, &con);
}

// Function to render the MuJoCo scene
void renderScene() {
    int fb_w, fb_h;
    glfwGetFramebufferSize(window, &fb_w, &fb_h);
    glViewport(0, 0, fb_w, fb_h);
    mjr_setBuffer(mjFB_WINDOW, &con);
    mjv_updateScene(mujoco_model, mujoco_data, &opt, nullptr, &cam, mjCAT_ALL, &scn);
    mjrRect viewport = {0, 0, fb_w, fb_h};
    mjr_render(viewport, &scn, &con);
    glfwSwapBuffers(window);
    glfwPollEvents();
}

// Function to set initial camera position
void adjustCamera() {
    cam.distance = 2.0;
    cam.azimuth = 235;
    cam.elevation = -20;
}

///////////////////////////////
// Data collection functions //
///////////////////////////////

// Data logging objects
std::ostringstream mujoco_datastream;
AsyncLogger data_log("../data/go1_mujoco_data.csv", mujoco_datastream.str());
std::ostringstream mujoco_data_row;

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
    write_vector(state.root_pos, os); os << ",";
    write_vector(state.root_pos_d, os); os << ",";

    // Linear velocity
    write_vector(state.root_lin_vel, os); os << ",";
    write_vector(state.root_lin_vel_d, os); os << ",";

    // RPY orientation
    write_vector(state.root_rpy, os); os << ",";
    write_vector(state.root_rpy_d, os); os << ",";

    // Angular velocity
    write_vector(state.root_ang_vel, os); os << ",";
    write_vector(state.root_ang_vel_d, os); os << ",";
 
    // Foot positions and velocities
    write_vector(state.foot_pos_world_rot, os); os << ",";
    write_vector(state.foot_pos_abs, os); os << ",";
    write_vector(state.foot_pos_liftoff, os); os << ",";
    write_vector(state.foot_pos_d, os); os << ",";
    write_vector(state.foot_vel_d, os); os << ",";

    // Foot forces and joint torques
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
    
    {
        Eigen::Map<const Eigen::VectorXd> flat_torque(
            state.joint_torques.data(),
            state.joint_torques.size()
        );
        write_vector(flat_torque, os);
    }

    os << ',';

    // Foot contact states
    os << state.swing_phase << "," 
        << (state.contacts[0] ? 1 : 0) << "," 
        << (state.contacts[1] ? 1 : 0) << "," 
        << (state.contacts[2] ? 1 : 0) << "," 
        << (state.contacts[3] ? 1 : 0) << ",";

    // Estimated states
    write_vector(state.root_pos_est, os); os << ",";
    write_vector(state.root_lin_vel_est, os); os << ",";
    write_vector(state.root_lin_acc_est, os); os << ",";
    write_vector(state.root_rpy_est, os); os << ",";
    
    // Sensor measurements
    write_vector(state.root_lin_acc_meas, os); os << ",";
    write_vector(state.root_ang_vel_meas, os); os << ",";
    write_vector(state.est_contacts, os); os << ",";
    write_vector(state.joint_pos, os); os << "\n";
}

void storeCalcTimeData(double update_time, double est_time, double MPC_calc_time, std::ostream &os) {
    // Store calculation times
    {
        os << update_time << "," 
        << est_time << "," 
        << MPC_calc_time << "\n";
    }
}

void writeCSVHeader(std::ostream &os) {    
    os << "root_pos_x,root_pos_y,root_pos_z,root_pos_d_x,root_pos_d_y,root_pos_d_z,"
            "root_lin_vel_x,root_lin_vel_y,root_lin_vel_z,root_lin_vel_d_x,root_lin_vel_d_y,root_lin_vel_d_z,"
            "root_rpy_x,root_rpy_y,root_rpy_z,root_rpy_d_x,root_rpy_d_y,root_rpy_d_z,"
            "root_ang_vel_x,root_ang_vel_y,root_ang_vel_z,root_ang_vel_d_x,root_ang_vel_d_y,root_ang_vel_d_z,"
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
            "root_pos_est_x,root_pos_est_y,root_pos_est_z,"
            "root_lin_vel_est_x,root_lin_vel_est_y,root_lin_vel_est_z,"
            "root_lin_acc_est_x,root_lin_acc_est_y,root_lin_acc_est_z,"
            "root_rpy_est_x,root_rpy_est_y,root_rpy_est_z,"
            "root_lin_acc_meas_x,root_lin_acc_meas_y,root_lin_acc_meas_z,"
            "root_ang_vel_meas_x,root_ang_vel_meas_y,root_ang_vel_meas_z,"
            "FR_contact_meas,FL_contact_meas,RR_contact_meas,RL_contact_meas,"
            "FR_0,FR_1,FR_2,FL_0,FL_1,FL_2,RR_0,RR_1,RR_2,RL_0,RL_1,RL_2\n";

}

void writeCalcTimeCSVHeader(std::ostream &os) {
    os << "state_update_time,estimation_time,MPC_calc_time\n";
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

void recordLowLevel(const go1State &state) {
    mujoco_data_row.str("");
    mujoco_data_row.clear();
    storeData(state, mujoco_data_row);
    data_log.logLine(mujoco_data_row.str());
}

int main(void) {
    // Load MuJoCo model
    char error[1000] = "";
    std::filesystem::path rel("../models/go1_MATLAB.xml");
    std::string path = std::filesystem::absolute(rel);
    mujoco_model = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
    if (!mujoco_model) {
        std::cerr << "Model load error: " << error << std::endl;
        return -1;
    }
    mujoco_data = mj_makeData(mujoco_model);

    if (mujoco_model->nkey > 0) {
        int keyframe_id = mj_name2id(mujoco_model, mjOBJ_KEY, "prone");
        if (keyframe_id == -1) {
            std::cerr << "Keyframe 'prone' not found!" << std::endl;
            return -1;
        }
        mj_resetDataKeyframe(mujoco_model, mujoco_data, keyframe_id);
    } else {
        mj_resetData(mujoco_model, mujoco_data);
    }

    initVisualization();
    adjustCamera();

    // Prep data collection
    writeCSVHeader(mujoco_datastream);
    data_log.logLine(mujoco_datastream.str());

    // Initialize data interface + estimator
    auto data_src = make_unique<mujocoDataReader>(mujoco_model, mujoco_data, "trunk", 
                                                "lin_acc_sensor", "ang_vel_sensor", 
                                                "touch_FR", "touch_FL", 
                                                "touch_RR", "touch_RL");
    auto command_sender = make_unique<mujocoCommandSender>(mujoco_model, mujoco_data);
    auto estimator = makeEstimator();
    
    // Instantiate FSM at DT_CTRL for overall loop rate, DT_MPC_CTRL for MPC loop rate
    go1FSM fsm(DT_CTRL, DT_MPC_CTRL, std::move(data_src), std::move(estimator), std::move(command_sender));
    fsm.collectInitialState();
    fsm.step();
    storeData(fsm.getState(), mujoco_data_row);
    data_log.logLine(mujoco_data_row.str());

    std::cout << "MuJoCo simulation has initialized..." << std::endl;
    running = true;

    // Main loop: construct LoopFunc objects and run until GLFW window is closed or controller is killed
    LoopFunc loop_keyInput("key_input_loop", DT_CTRL, 4, boost::bind(&keyboardControl, boost::ref(fsm)));
    LoopFunc loop_record("recording_loop", DT_CTRL, boost::bind(&recordLowLevel, boost::ref(fsm.getState())));
    LoopFunc loop_simulate("simulation_loop", 
                            mujoco_model->opt.timestep, 
                            7, 
                            [&](){
                                mj_step(mujoco_model, mujoco_data);
                                fsm.step();
                            });
    const auto render_interval = std::chrono::milliseconds(16); // ~60 FPS
    auto last_render_time = std::chrono::steady_clock::now();
    
    loop_simulate.start();
    loop_keyInput.start();
    loop_record.start();

    while (running && !glfwWindowShouldClose(window)) {
        auto now = std::chrono::steady_clock::now();
        if (now - last_render_time >= render_interval) {
            renderScene();
            last_render_time = now;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    };

    // Cleanup
    endwin();

    glfwDestroyWindow(window);
    glfwTerminate();
    mj_deleteData(mujoco_data);
    mj_deleteModel(mujoco_model);
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    std::cout << "MuJoCo simulation complete!" << std::endl;

    return 0;
}
