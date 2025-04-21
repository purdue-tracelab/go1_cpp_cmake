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

// Global variables
static mjModel* model = nullptr;
static mjData* mujocoFSMData = nullptr;
static mjvCamera cam;
static mjvOption opt;
static mjvScene scn;
static mjrContext con;
static GLFWwindow* window = nullptr;
int window_width = 1600, window_height = 900;

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

int main(int argc, char** argv) {
    std::cout << "Initializing MuJoCo..." << std::endl;

    // Load MuJoCo model
    char error[1000] = "";
    std::filesystem::path relative_model_path("src/go1_cpp_cmake/models/go1_platform.xml");
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

    auto state = std::make_unique<go1State>();
    auto fsm = std::make_unique<go1FSM>(*state);
    fsm->start();

    glfwSetWindowUserPointer(window, fsm.get());
    glfwSetKeyCallback(window, key_callback);

    const double timestep = model->opt.timestep;
    const auto sim_dt = std::chrono::duration<double>(timestep);
    const auto render_dt = std::chrono::milliseconds(16);
    auto next_render_time = std::chrono::steady_clock::now();

    while (!glfwWindowShouldClose(window)) {
        mj_step(model, mujocoFSMData);
        int base_id = mj_name2id(model, mjOBJ_BODY, "trunk");
        Eigen::Vector3d lin_acc = Eigen::Map<const Eigen::Vector3d>(mujocoFSMData->cacc + 3 * base_id);
        state->updateStateFromMujoco(mujocoFSMData->qpos, mujocoFSMData->qvel, lin_acc);

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

        auto now = std::chrono::steady_clock::now();
        if (now >= next_render_time) {
            std::cout << "FSM state (number): " << static_cast<int>(fsm->getFiniteState()) << std::endl;
            renderScene();
            next_render_time += render_dt;
        }

        if (fsm->getFiniteState() == go1FiniteState::Shutdown && state->isShutdownComplete())
            break;

        std::this_thread::sleep_for(sim_dt);
    }

    fsm->stop();
    mj_deleteData(mujocoFSMData);
    mj_deleteModel(model);
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
