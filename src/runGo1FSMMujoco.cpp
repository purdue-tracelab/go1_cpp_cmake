#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <filesystem>
#include <ncurses.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <cstring>

#include "go1_cpp_cmake/go1FSM.h"
#include "go1_cpp_cmake/go1StateEstimator.h"

// Global variables for MuJoCo
static mjModel* mujoco_model = nullptr;
static mjData* mujoco_data = nullptr;
static mjvCamera cam;
static mjvOption opt;
static mjvScene scn;
static mjrContext con;
static GLFWwindow* window = nullptr;
int window_width = 1600, window_height = 900;  // Default window size

/////////////////////////////
// MuJoCo simulation setup //
/////////////////////////////

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

///////////////////
// Main function //
///////////////////

int main() {
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

    std::ostringstream mujoco_datastream;
    writeCSVHeader(mujoco_datastream);

    AsyncLogger data_log("../data/go1_mujoco_data.csv", mujoco_datastream.str());
    std::ostringstream mujoco_data_row;

    auto data_src = std::make_unique<mujocoDataReader>(
                      mujoco_model, mujoco_data,
                      "trunk",
                      "lin_acc_sensor",
                      "ang_vel_sensor",
                      "touch_FR",
                      "touch_FL",
                      "touch_RR",
                      "touch_RL");
    auto estimator = makeEstimator();
    auto command_sender = std::make_unique<mujocoCommandSender>(mujoco_model, mujoco_data);

    // Instantiate FSM at 500 Hz state loop, 50 Hz MPC
    go1FSM fsm(DT_CTRL, DT_MPC_CTRL, std::move(data_src), std::move(estimator), std::move(command_sender));
    fsm.collectInitialState();
    fsm.step();
    storeData(fsm.getState(), mujoco_data_row);
    data_log.logLine(mujoco_data_row.str());

    // Hook key callbacks
    glfwSetWindowUserPointer(window, &fsm);
    glfwSetKeyCallback(window, [](GLFWwindow* w, int key, int sc, int action, int mods) {
        if (action != GLFW_PRESS) return;
        auto f = static_cast<go1FSM*>(glfwGetWindowUserPointer(w));
        if (!f) return;
        switch (key) {
            case GLFW_KEY_P: f->requestWalk(true);  break;
            case GLFW_KEY_O: f->requestWalk(false); break;
            case GLFW_KEY_I: f->requestShutdown();  break;
            case GLFW_KEY_U: f->requestStartup();   break;
        }
    });

    // Main loop: step simulation and FSM, then render
    const std::chrono::milliseconds loop_time(static_cast<long>(1000 * mujoco_model->opt.timestep)); // 500 Hz loop time
    bool running = true;
    const auto render_interval = std::chrono::milliseconds(16); // ~60 FPS
    auto last_render_time = std::chrono::steady_clock::now();

    while (!glfwWindowShouldClose(window)) {
        auto loop_start = std::chrono::high_resolution_clock::now();
        mj_step(mujoco_model, mujoco_data);
        fsm.step();

        {
            mujoco_data_row.str("");
            mujoco_data_row.clear();
            storeData(fsm.getState(), mujoco_data_row);
            data_log.logLine(mujoco_data_row.str());
        }

        auto now = std::chrono::steady_clock::now();
        if (now - last_render_time >= render_interval) {
            renderScene();
            last_render_time = now;
            std::cout << "FSM state: " << fsm.go1FiniteState2Str() << std::endl;
        }

        auto loop_end = std::chrono::high_resolution_clock::now();
        auto loop_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end - loop_start);
        auto remaining_time = loop_time - loop_elapsed;
        if (remaining_time > std::chrono::milliseconds(0)) {
            std::this_thread::sleep_for(remaining_time);
        }
    }

    // Cleanup
    glfwDestroyWindow(window);
    glfwTerminate();
    mj_deleteData(mujoco_data);
    mj_deleteModel(mujoco_model);
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    return 0;
}