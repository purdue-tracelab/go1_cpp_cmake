#include <rclcpp/rclcpp.hpp>
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

#include "go1_cpp_cmake/go1StanceMPC.h"

// Global variables for MuJoCo
mjModel* model = nullptr;
mjData* data = nullptr;
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;
GLFWwindow* window = nullptr;
int window_width = 1270, window_height = 720;  // Default window size

// Global variables for Go1 Robot
go1State mujoco_go1_state;
go1MPC mujoco_go1_stance_proctor;
Eigen::Matrix<double, 12, 1> joint_torques_stacked;

// Synchronization primitives & variables
std::mutex stateMutex;
std::atomic<bool> runSimulation{true};
std::chrono::duration<double> simulationTimeStep(DT_CTRL); // 500 Hz, dt = 0.002 s or 2 ms
std::chrono::duration<double> mpcTimeStep(DT_MPC_CTRL); // 50 Hz, dt = 0.02 s or 20 ms

// Mouse state variables
bool button_left = false, button_middle = false, button_right = false;
double lastx = 0, lasty = 0;

// GLFW window resize callback
void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    window_width = width;
    window_height = height;
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
    runSimulation = false;
}

// Function to initialize visualization
void initVisualization() {
    if (!glfwInit()) {
        RCLCPP_ERROR(rclcpp::get_logger("mujoco_sim"), "Could not initialize GLFW!");
        exit(1);
    }

    window = glfwCreateWindow(window_width, window_height, "MuJoCo Simulation", NULL, NULL);
    if (!window) {
        RCLCPP_ERROR(rclcpp::get_logger("mujoco_sim"), "Could not create GLFW window!");
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
}

// Function to render the MuJoCo scene
void renderScene() {
    mjv_updateScene(model, data, &opt, nullptr, &cam, mjCAT_ALL, &scn);
    mjrRect viewport = {0, 0, window_width, window_height};
    mjr_render(viewport, &scn, &con);
    glfwSwapBuffers(window);
    glfwPollEvents();
}

// Function to set initial camera position
void adjustCamera() {
    cam.distance = 3.5;
    cam.azimuth = 235;
    cam.elevation = -20;
}

void simulationThread() {
    // Buffers for joint states
    std::vector<mjtNum> qpos_buffer(model->nq);
    std::vector<mjtNum> qvel_buffer(model->nv);

    while (runSimulation && !glfwWindowShouldClose(window)) {
        auto start = std::chrono::steady_clock::now();

        std::memcpy(qpos_buffer.data(), data->qpos, model->nq * sizeof(mjtNum));
        std::memcpy(qvel_buffer.data(), data->qvel, model->nv * sizeof(mjtNum));
        
        { // Lock state before applying torques
            std::lock_guard<std::mutex> lock(stateMutex);
            for (int j = 0; j < model->nu; j++) {
                data->ctrl[j] = joint_torques_stacked(j, 0);
            }
        }

        // Step the simulation
        mj_step(model, data);

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        if (elapsed < simulationTimeStep) {
            std::this_thread::sleep_for(simulationTimeStep - elapsed);
        }
    }
}

void stateUpdateThread() { // 500 Hz
    // std::chrono::milliseconds updateInterval(2);
    std::vector<mjtNum> qpos_buffer(model->nq);
    std::vector<mjtNum> qvel_buffer(model->nv);
    int base_id = mj_name2id(model, mjOBJ_BODY, "trunk");

    while (runSimulation) {
        auto start = std::chrono::steady_clock::now();
        { // Copy simulation data to buffers
            std::lock_guard<std::mutex> lock(stateMutex);
            std::memcpy(qpos_buffer.data(), data->qpos, model->nq * sizeof(mjtNum));
            std::memcpy(qvel_buffer.data(), data->qvel, model->nv * sizeof(mjtNum));
        }
        
        Eigen::Vector3d lin_acc = Eigen::Map<const Eigen::Vector3d>(data->cacc + 3 * base_id);
        
        { // update go1State object & compute torques
            std::lock_guard<std::mutex> lock(stateMutex);
            mujoco_go1_state.updateStateFromMujoco(qpos_buffer.data(), qvel_buffer.data(), lin_acc);
            mujoco_go1_state.convertForcesToTorquesMujoco(qpos_buffer.data());
            joint_torques_stacked << mujoco_go1_state.joint_torques.block<3, 1>(0, 0), // FR
                                    mujoco_go1_state.joint_torques.block<3, 1>(0, 1), // FL
                                    mujoco_go1_state.joint_torques.block<3, 1>(0, 2), // RR
                                    mujoco_go1_state.joint_torques.block<3, 1>(0, 3); // RL
        }
        
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        if (elapsed < simulationTimeStep) {
            std::this_thread::sleep_for(simulationTimeStep - elapsed);
        }
        // std::this_thread::sleep_for(updateInterval);
    }
}

void mpcThread() { // 50 Hz
    // std::chrono::milliseconds mpcInterval(20);

    while (runSimulation) {
        auto start = std::chrono::steady_clock::now();
        { // Calculate stance forces using go1MPC object
            std::lock_guard<std::mutex> lock(stateMutex);
            mujoco_go1_stance_proctor.solveMPCForState(mujoco_go1_state);
        }
        
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        if (elapsed < mpcTimeStep) {
            std::this_thread::sleep_for(mpcTimeStep - elapsed);
        }
        // std::this_thread::sleep_for(mpcInterval);
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("threaded_mujoco_sim");

    RCLCPP_INFO(node->get_logger(), "Initializing MuJoCo ...");

    mujoco_go1_state.resetState();
    joint_torques_stacked.setZero();

    // Load MuJoCo model
    char error[1000] = "";
    std::filesystem::path relative_model_path("../models/go1_MATLAB.xml");
    std::string model_path = std::filesystem::absolute(relative_model_path);

    mjVFS vfs;
    mj_defaultVFS(&vfs);

    if (mj_addFileVFS(&vfs, model_path.c_str(), model_path.c_str()) != 0) {
        RCLCPP_ERROR(node->get_logger(), "Failed to add XML to VFS: %s", model_path.c_str());
        mj_deleteVFS(&vfs);
        rclcpp::shutdown();
        return -1;
    }

    model = mj_loadXML(model_path.c_str(), &vfs, error, 1000);
    mj_deleteVFS(&vfs);

    if (!model) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load MuJoCo model: %s", error);
        rclcpp::shutdown();
        return -1;
    }

    data = mj_makeData(model);
    
    if (model->nkey > 0) {
        int keyframe_id = mj_name2id(model, mjOBJ_KEY, "standing");
        if (keyframe_id == -1) {
            RCLCPP_ERROR(node->get_logger(), "Keyframe 'standing' not found!");
            rclcpp::shutdown();
            return -1;
        }
        mj_resetDataKeyframe(model, data, keyframe_id);
    } else {
        mj_resetData(model, data);
    }

    initVisualization();
    adjustCamera();

    mujoco_go1_state.root_pos_d << 0, 0, 0.27;
    mujoco_go1_state.walking_mode = true;

    // Launch threads
    std::thread simThread(simulationThread);
    std::thread stateThread(stateUpdateThread);
    std::thread mpcCtrlThread(mpcThread);

    std::thread rosSpinThread([&]() { rclcpp::spin(node); });

    double lastRenderSimTime = -1.0;
    double renderInterval = 1.0 / 60.0;  // ~60 Hz rendering

    while (!glfwWindowShouldClose(window)) {
        // Check if window is getting closed
        glfwPollEvents();

        // Render the simulation
        if (data->time - lastRenderSimTime >= renderInterval) {
            renderScene();
            lastRenderSimTime = data->time;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // why this?
    }

    // Shutdown sequence
    runSimulation = false;

    if (simThread.joinable()) simThread.join();
    if (stateThread.joinable()) stateThread.join();
    if (mpcCtrlThread.joinable()) mpcCtrlThread.join();
    if (rosSpinThread.joinable()) rosSpinThread.join();

    mj_deleteData(data);
    mj_deleteModel(model);
    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    glfwDestroyWindow(window);
    glfwTerminate();
    rclcpp::shutdown();

    RCLCPP_INFO(node->get_logger(), "MuJoCo Threaded Simulation Complete!");
    return 0;
}