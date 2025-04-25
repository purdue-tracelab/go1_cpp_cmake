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

UNITREE_LEGGED_SDK::LowState lowState;
Eigen::Matrix<double, 12, 1> joint_pos;
// joint_pos.setZero();
UNITREE_LEGGED_SDK::LowCmd cmd = {0};

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

    const auto sim_dt = std::chrono::duration<double>(DT_CTRL);
    const auto render_dt = std::chrono::milliseconds(16);
    auto next_render_time = std::chrono::steady_clock::now();

    while (1) {
   
        state->updateStateFromHardware(lowState); // Question for Annalise: why is this error here? Look at the name and type of the input
                                        //was state-> updateStateFromHardware(UNITREE_LEGGED_SDK::LowState& state);
                                        //change for all below
        switch (fsm->getFiniteState()) {
            case go1FiniteState::Startup:
                state->computeStartupPDHardware(lowState);
                break;
            case go1FiniteState::Shutdown:
                state->computeShutdownPDHardware(lowState);
                break;
            default:
                break;
        }

        if (fsm->getFiniteState() == go1FiniteState::Locomotion)
            {
            for(int i = 0; i < 11; ++i)
            joint_pos[i] = lowState.motorState[i].q;
            }
            state->convertForcesToTorquesHardware(joint_pos); 
        for (int i = 0; i < 3*NUM_LEG; ++i)
            {
            cmd.motorCmd[i].q = UNITREE_LEGGED_SDK::PosStopF;
            cmd.motorCmd[i].dq = UNITREE_LEGGED_SDK::VelStopF;
            cmd.motorCmd[i].Kp = 0;
            cmd.motorCmd[i].Kd = 0;
            cmd.motorCmd[i].tau = state->joint_torques(i%3,i/3);
            }

        if (fsm->getFiniteState() == go1FiniteState::Shutdown && state->isShutdownComplete())
            break;

        std::this_thread::sleep_for(sim_dt);
    }

    fsm->stop();
    return 0;
}