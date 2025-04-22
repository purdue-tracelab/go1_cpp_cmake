#include "unitree_legged_sdk/unitree_legged_sdk.h"
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
   
        state->updateStateFromHardware(UNITREE_LEGGED_SDK::LowState& state);

        switch (fsm->getFiniteState()) {
            case go1FiniteState::Startup:
                state->computeStartupPDHardware(UNITREE_LEGGED_SDK::LowState& state);
                break;
            case go1FiniteState::Shutdown:
                state->computeShutdownPDHardware(UNITREE_LEGGED_SDK::LowState& state);
                break;
            default:
                break;
        }

        if (fsm->getFiniteState() == go1FiniteState::Locomotion)
            state->convertForcesToTorquesHardware(UNITREE_LEGGED_SDK::LowState& state);
        for (int i = 0; i < model->nu; ++i)
            cmd.motorCmd[i].q = PosStopF;
            cmd.motorCmd[i].dq = VelStopF;
            cmd.motorCmd[i].Kp = 0;
            cmd.motorCmd[i].Kd = 0;
            cmd.motorCmd[i].tau = state->joint_torrques(i%3,i/3);
        

        if (fsm->getFiniteState() == go1FiniteState::Shutdown && state->isShutdownComplete())
            break;

        std::this_thread::sleep_for(sim_dt);
    }

    fsm->stop();
    return 0;
}