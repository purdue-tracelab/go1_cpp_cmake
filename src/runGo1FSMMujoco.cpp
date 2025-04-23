#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <filesystem>
#include <ncurses.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <cstring>
#include <chrono>

#include "go1_cpp_cmake/go1FSM.h"
#include "go1_cpp_cmake/go1StateEstimator.h"

// Global variables
static mjModel* model = nullptr;
static mjData* mujocoFSMData = nullptr;
static mjvCamera cam;
static mjvOption opt;
static mjvScene scn;
static mjrContext con;
static GLFWwindow* window = nullptr;
int window_width = 1600, window_height = 900;

/////////////////////////////
// MuJoCo simulation setup //
/////////////////////////////

// Mouse state
bool button_left = false, button_middle = false, button_right = false;
double lastx = 0, lasty = 0;

// GLFW callbacks
void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    window_width = width;
    window_height = height;
    glViewport(0, 0, width, height);
    mjr_setBuffer(mjFB_WINDOW, &con);
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    button_left   = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right  = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    cam.distance *= (1.0 - 0.05 * yoffset);
}

void cursor_position_callback(GLFWwindow* window, double xpos, double ypos) {
    if (!button_left && !button_middle && !button_right) return;
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

void window_close_callback(GLFWwindow* window) {
    glfwSetWindowShouldClose(window, GLFW_TRUE);
}

// Visualization setup
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
    mjv_makeScene(model, &scn, 2000);
    mjr_defaultContext(&con);
    mjr_makeContext(model, &con, mjFONTSCALE_150);

    glfwGetFramebufferSize(window, &window_width, &window_height);
    glViewport(0, 0, window_width, window_height);
    mjr_setBuffer(mjFB_WINDOW, &con);
}

void renderScene() {
    int fb_w, fb_h;
    glfwGetFramebufferSize(window, &fb_w, &fb_h);
    glViewport(0, 0, fb_w, fb_h);
    mjr_setBuffer(mjFB_WINDOW, &con);
    mjv_updateScene(model, mujocoFSMData, &opt, nullptr, &cam, mjCAT_ALL, &scn);
    mjrRect viewport = {0, 0, fb_w, fb_h};
    mjr_render(viewport, &scn, &con);
    glfwSwapBuffers(window);
    glfwPollEvents();
}

void adjustCamera() {
    cam.distance = 2.0;
    cam.azimuth = 235;
    cam.elevation = -20;
}

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
void storeData(const go1State &state, std::ostream &os, const Eigen::Vector3d &lin_acc_meas, const Eigen::Vector3d &ang_vel_meas) {
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
    write_vector(state.foot_pos, os); os << ",";
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
    write_vector(lin_acc_meas, os); os << ",";
    write_vector(ang_vel_meas, os); os << "\n";
    
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
            "root_ang_vel_meas_x,root_ang_vel_meas_y,root_ang_vel_meas_z\n";

}

void writeCalcTimeCSVHeader(std::ostream &os) {
    os << "state_update_time,estimation_time,MPC_calc_time\n";
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        auto fsm = static_cast<go1FSM*>(glfwGetWindowUserPointer(window));
        if (!fsm) return;

        switch (key) {
            case GLFW_KEY_I:
                fsm->requestShutdown();
                break;
            case GLFW_KEY_P:
                fsm->requestWalk(true);
                break;
            case GLFW_KEY_O:
                fsm->requestWalk(false);
                break;
        }
    }
}

// Function to start the state estimator thread
void startEstimatorThread(go1State& state, go1StateEstimator& estimator, std::mutex& mtx, std::atomic<bool>& running, std::thread& thread_out) {
    thread_out = std::thread([&]() {
        const auto dt = std::chrono::milliseconds(2); // 500 Hz
        auto next_tick = std::chrono::steady_clock::now();

        while (running.load()) {
            Eigen::Vector3d accel, gyro;
            {
                std::lock_guard<std::mutex> lk(mtx);
                accel = state.root_lin_acc_meas;
                gyro = state.root_ang_vel_meas;
            }

            {
                std::lock_guard<std::mutex> lk(mtx);
                estimator.estimateState(state, accel, gyro);
            }

            next_tick += dt;
            auto now = std::chrono::steady_clock::now();

            if (next_tick > now)    
                std::this_thread::sleep_for(next_tick - now);
        }
    });
}

int main(int argc, char** argv) {
    std::cout << "Initializing MuJoCo..." << std::endl;

    // Load MuJoCo model
    char error[1000] = "";
    std::filesystem::path relative_model_path("../models/go1_MATLAB.xml");
    std::string model_path = std::filesystem::absolute(relative_model_path);

    mjVFS vfs;
    mj_defaultVFS(&vfs);

    if (mj_addFileVFS(&vfs, model_path.c_str(), model_path.c_str()) != 0) {
        std::cerr << "Failed to add XML to VFS: " << model_path << std::endl;
        mj_deleteVFS(&vfs);
        return -1;
    }

    model = mj_loadXML(model_path.c_str(), &vfs, error, 1000);
    mj_deleteVFS(&vfs);

    if (!model) {
        std::cerr << "Failed to load MuJoCo model: " << error << std::endl;
        return -1;
    }

    mujocoFSMData = mj_makeData(model);

    if (model->nkey > 0) {
        int keyframe_id = mj_name2id(model, mjOBJ_KEY, "prone");
        if (keyframe_id == -1) {
            std::cerr << "Keyframe 'prone' not found!" << std::endl;
            return -1;
        }
        mj_resetDataKeyframe(model, mujocoFSMData, keyframe_id);
    } else {
        mj_resetData(model, mujocoFSMData);
    }

    initVisualization();
    adjustCamera();

    int base_id = mj_name2id(model, mjOBJ_BODY, "trunk");
    int sensor_id1 = mj_name2id(model, mjOBJ_SENSOR, "lin_acc_sensor");
    int sensor_id2 = mj_name2id(model, mjOBJ_SENSOR, "ang_vel_sensor");
    int sensor_adr1 = model->sensor_adr[sensor_id1];
    int sensor_adr2 = model->sensor_adr[sensor_id2];

    // Create go1State & go1FSM objects & associated thread
    auto state = std::make_unique<go1State>();
    auto fsm   = std::make_unique<go1FSM>(*state);
    auto estimator = makeEstimator();

    std::mutex est_mutex;
    std::atomic<bool> est_running{true};
    std::thread est_thread;

    std::ostringstream mujoco_data, calc_time_data;
    writeCSVHeader(mujoco_data);
    // writeCalcTimeCSVHeader(calc_time_data);

    AsyncLogger data_log("../data/go1_mujoco_data.csv", mujoco_data.str());
    std::ostringstream mujoco_data_row;

    glfwSetWindowUserPointer(window, fsm.get());
    glfwSetKeyCallback(window, key_callback);

    {
        std::lock_guard<std::mutex> lk(est_mutex);
        state->updateStateFromMujoco(mujocoFSMData->qpos, mujocoFSMData->qvel, Eigen::Map<const Eigen::Vector3d>(mujocoFSMData->cacc + 3*base_id));
        state->root_lin_acc_meas = Eigen::Map<const Eigen::Vector3d>(mujocoFSMData->sensordata + sensor_adr1);
        state->root_ang_vel_meas = Eigen::Map<const Eigen::Vector3d>(mujocoFSMData->sensordata + sensor_adr2);
        storeData(*state, mujoco_data_row, state->root_lin_acc_meas, state->root_ang_vel_meas);
        data_log.logLine(mujoco_data_row.str());
    }

    estimator->collectInitialState(*state);
    startEstimatorThread(*state, *estimator, est_mutex, est_running, est_thread);
    fsm->start();

    const double timestep = model->opt.timestep;
    const auto sim_dt = std::chrono::duration<double>(timestep);
    const auto render_dt = std::chrono::milliseconds(16);
    auto next_render_time = std::chrono::steady_clock::now();

    while (!glfwWindowShouldClose(window)) {
        mj_step(model, mujocoFSMData);
        {
            std::lock_guard<std::mutex> lk(est_mutex);
            state->updateStateFromMujoco(mujocoFSMData->qpos, mujocoFSMData->qvel, Eigen::Map<const Eigen::Vector3d>(mujocoFSMData->cacc + 3*base_id));
            state->root_lin_acc_meas = Eigen::Map<const Eigen::Vector3d>(mujocoFSMData->sensordata + sensor_adr1);
            state->root_ang_vel_meas = Eigen::Map<const Eigen::Vector3d>(mujocoFSMData->sensordata + sensor_adr2);
        }

        switch (fsm->getFiniteState()) {
            case go1FiniteState::Startup:
                state->computeStartupPDMujoco(mujocoFSMData->qpos, mujocoFSMData->qvel);
                break;
            case go1FiniteState::Shutdown:
                state->computeShutdownPDMujoco(mujocoFSMData->qpos, mujocoFSMData->qvel);
                break;
            default:
                break;
        }

        if (fsm->getFiniteState() == go1FiniteState::Locomotion)
            state->convertForcesToTorquesMujoco(mujocoFSMData->qpos);
        for (int i = 0; i < model->nu; ++i)
            mujocoFSMData->ctrl[i] = state->joint_torques(i%3, i/3);

        {   
            mujoco_data_row.str("");
            mujoco_data_row.clear();
            storeData(*state, mujoco_data_row, state->root_lin_acc_meas, state->root_ang_vel_meas);
            data_log.logLine(mujoco_data_row.str());
        }
        
        {
            auto now = std::chrono::steady_clock::now();
            if (now >= next_render_time) {
                // std::cout << "FSM state (number): " << static_cast<int>(fsm->getFiniteState()) << std::endl;
                renderScene();
                next_render_time += render_dt;
            }
        }

        if (fsm->getFiniteState() == go1FiniteState::Shutdown && state->isShutdownComplete()) break;

        std::this_thread::sleep_for(sim_dt);
    }

    est_running = false;
    est_thread.join();

    fsm->stop();
    mj_deleteData(mujocoFSMData);
    mj_deleteModel(model);
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
