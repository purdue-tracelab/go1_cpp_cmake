#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <filesystem>
#include <ncurses.h>

#include "go1_cpp_cmake/go1StanceMPC.h"
#include "go1_cpp_cmake/go1StateEstimator.h"
#include "go1_cpp_cmake/go1DataInterface.h"

// Global variables for MuJoCo
mjModel* mujoco_model = nullptr;
mjData* mujoco_data = nullptr;
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;
GLFWwindow* window = nullptr;
int window_width = 2560, window_height = 1440;  // Default window size

// Global variables for Go1 Robot
go1State mujoco_go1_state;
std::unique_ptr<go1StateEstimator> mujoco_go1_estimator;
go1MPC mujoco_go1_MPC;
Eigen::Matrix<double, 12, 1> joint_torques_stacked;
float pitch_ctrl;
float ground_truth_height_des;
float go1_pos_shift = 0.5; // Shift the Go1 position in the x-direction

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
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetCursorPosCallback(window, cursor_position_callback);

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_makeScene(mujoco_model, &scn, 2000);

    mjr_defaultContext(&con);
    mjr_makeContext(mujoco_model, &con, mjFONTSCALE_150);
}

// Function to render the MuJoCo scene
void renderScene() {
    mjv_updateScene(mujoco_model, mujoco_data, &opt, nullptr, &cam, mjCAT_ALL, &scn);
    mjrRect viewport = {0, 0, window_width, window_height};
    mjr_render(viewport, &scn, &con);
    glfwSwapBuffers(window);
    glfwPollEvents();
}

// Function to set initial camera position
void adjustCamera() {
    cam.distance = 5.0;
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

int main(int argc, char** argv) {
    std::cout << "Initializing MuJoCo..." << std::endl;

    // Ask user for test case
    int test_case;
    std::cout << "Which test case should we run? (0 = flat ground, 1-5 = pitch + sway)" << std::endl;
    std::cin >> test_case;

    mujoco_go1_state.resetState();

    // Load MuJoCo model
    char error[1000] = "";
    std::filesystem::path relative_model_path("../models/go1_platform.xml");
    std::string model_path = std::filesystem::absolute(relative_model_path);

    mjVFS vfs;
    mj_defaultVFS(&vfs);

    if (mj_addFileVFS(&vfs, model_path.c_str(), model_path.c_str()) != 0) {
        std::cerr << "Failed to add XML to VFS: " << model_path.c_str() << std::endl;
        mj_deleteVFS(&vfs);
        return -1;
    }

    mujoco_model = mj_loadXML(model_path.c_str(), &vfs, error, 1000);
    mj_deleteVFS(&vfs);

    if (!mujoco_model) {
        std::cerr << "Failed to load MuJoCo model: " << error << std::endl;
        return -1;
    }

    mujoco_data = mj_makeData(mujoco_model);
    
    if (mujoco_model->nkey > 0) {
        int keyframe_id = mj_name2id(mujoco_model, mjOBJ_KEY, "fast_start");
        if (keyframe_id == -1) {
            std::cerr << "Keyframe 'fast_start' not found!" << std::endl;
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
    std::ostringstream mujoco_calcstream;
    writeCalcTimeCSVHeader(mujoco_calcstream);

    SyncLogger mujocoDataLog("../data/go1_mujoco_data.csv", mujoco_datastream.str());
    std::ostringstream mujoco_datastream_row;
    SyncLogger calcDataLog("../data/go1_mujoco_calc_time.csv", mujoco_calcstream.str());
    std::ostringstream mujoco_calcstream_row;

    const std::chrono::milliseconds loop_time(static_cast<long>(1000 * mujoco_model->opt.timestep)); // 500 Hz loop time
    bool running = true;
    double lastMPCUpdateTime = -1.0; // calculate MPC solution at the beginning
    double mpcInterval = DT_MPC_CTRL;
    double lastRenderSimTime = -1.0; // render at the beginning
    double renderInterval = 1.0/60.0;

    // Desired initial state
    if (USE_EST_FOR_CONTROL) {
        // mujoco_go1_state.root_pos_d << 0, 0, WALK_HEIGHT;
        mujoco_go1_state.root_pos_d << go1_pos_shift, 0, WALK_HEIGHT; // shifted forward
    } else {
        // mujoco_go1_state.root_pos_d << 0, 0, 1.0 + WALK_HEIGHT;
        mujoco_go1_state.root_pos_d << go1_pos_shift, 0, 1.0 + WALK_HEIGHT; // shifted forward
    }

    // Walking flag
    mujoco_go1_state.walking_mode = true;

    // Initialize the state and estimator from MuJoCo data
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
    data_src->pullSensorData(mujoco_go1_state);
    estimator->collectInitialState(mujoco_go1_state);

    if (STATE_EST_SELECT == 1) {
        for (int i = 0; i < NUM_LEG; i++) {
            if (USE_EST_FOR_CONTROL) {
                estimator->setFootHeightResidual(1.02);
            } else {
                estimator->setFootHeightResidual(0.02);
            }
        }
    }

    // Main simulation loop
    while (running && !glfwWindowShouldClose(window)) {
        auto loop_start = std::chrono::high_resolution_clock::now();
        std::cout << "Simulation time: " << mujoco_data->time << std::endl;

        ///////////////////////
        // TREADMILL CONTROL //
        ///////////////////////

        switch (test_case) {
            case 0:
                mujoco_data->ctrl[12] = 0;  // Sway oscillation in meters
                mujoco_data->ctrl[13] = 0;  // Pitch oscillation in radians
                break;

            case 1:
                mujoco_data->ctrl[12] = 0;
                mujoco_data->ctrl[13] = (4*M_PI/180) * (sin(3.0 * mujoco_data->time) + sin(mujoco_data->time * sqrt(0.5 * mujoco_data->time + 1)));
                pitch_ctrl = (4*M_PI/180) * (sin(3.0 * mujoco_data->time) + sin(mujoco_data->time * sqrt(0.5 * mujoco_data->time + 1))); // Store pitch control for later use
                break;

            case 2:
                mujoco_data->ctrl[12] = 0;
                mujoco_data->ctrl[13] = (4*M_PI/180) * (sin(6.0 * mujoco_data->time) + sin(0.1 * pow(mujoco_data->time, 2)));
                pitch_ctrl = (4*M_PI/180) * (sin(6.0 * mujoco_data->time) + sin(0.1 * pow(mujoco_data->time, 2)));
                break;

            case 3:
                mujoco_data->ctrl[12] = 0;
                mujoco_data->ctrl[13] = (0.2*M_PI/180) * pow(mujoco_data->time, 2) * sin(sqrt(100 * mujoco_data->time + 1)) * exp(mujoco_data->time / -10);
                pitch_ctrl = (0.2*M_PI/180) * pow(mujoco_data->time, 2) * sin(sqrt(100 * mujoco_data->time + 1)) * exp(mujoco_data->time / -10);
                break;

            case 4:
                if (mujoco_data->time <= 83) {
                    mujoco_data->ctrl[12] = 0;
                } else if (mujoco_data->time <= 122) {
                    mujoco_data->ctrl[12] = 40/1000 * sin(M_PI * mujoco_data->time);
                } else if (mujoco_data->time <= 160) {
                    mujoco_data->ctrl[12] = 65/1000 * sin(M_PI * mujoco_data->time);
                } else {
                    mujoco_data->ctrl[12] = 0;
                }
                mujoco_data->ctrl[13] = (4*M_PI/180) * (sin(3.0 * mujoco_data->time) + sin(mujoco_data->time * sqrt(0.5 * mujoco_data->time + 1)));
                pitch_ctrl = (4*M_PI/180) * (sin(3.0 * mujoco_data->time) + sin(mujoco_data->time * sqrt(0.5 * mujoco_data->time + 1)));
                break;

            case 5:
                mujoco_data->ctrl[12] = 0;
                mujoco_data->ctrl[13] = (2.5*M_PI/180) * (sin(3.0 * mujoco_data->time) + sin(mujoco_data->time * sqrt(0.5 * mujoco_data->time + 1)));
                pitch_ctrl = (2.5*M_PI/180) * (sin(3.0 * mujoco_data->time) + sin(mujoco_data->time * sqrt(0.5 * mujoco_data->time + 1)));
                break;

            default:
                std::cerr << "There are only 5 test cases" << std::endl;
                return 0;
        }

        /////////////////
        // GO1 CONTROL //
        /////////////////

        // Desired height (only for ground truth control)
        if (!USE_EST_FOR_CONTROL) {
            ground_truth_height_des = 1.0 + WALK_HEIGHT - go1_pos_shift * sin(pitch_ctrl);
            mujoco_go1_state.root_pos_d << go1_pos_shift, 0, ground_truth_height_des; // 1.25 m forward
        }

        // Update Go1 state from MuJoCo & perform swing PD force control (500 Hz)
        auto start1 = std::chrono::high_resolution_clock::now();

        data_src->pullSensorData(mujoco_go1_state);

        auto start2 = std::chrono::high_resolution_clock::now();
        estimator->estimateState(mujoco_go1_state);
        auto end2  = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed2 = end2 - start2;
        double duration_2 = elapsed2.count();

        mujoco_go1_state.updateLocomotionPlan();

        auto end1  = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed1 = end1 - start1;
        double duration_1 = elapsed1.count();

        double duration_3 = 0.0;

        // Update MPC controller (50 Hz)
        if (mujoco_data->time - lastMPCUpdateTime >= mpcInterval) {
            auto start3 = std::chrono::high_resolution_clock::now();
            
            mujoco_go1_MPC.solveMPCForState(mujoco_go1_state);
            
            auto end3 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> elapsed3 = end3 - start3;
            duration_3 = elapsed3.count();
            lastMPCUpdateTime = mujoco_data->time;
        }

        // Convert foot forces to joint torques
        mujoco_go1_state.convertForcesToTorques();
        joint_torques_stacked << mujoco_go1_state.joint_torques.col(0), // FR
                                mujoco_go1_state.joint_torques.col(1), // FL
                                mujoco_go1_state.joint_torques.col(2), // RR
                                mujoco_go1_state.joint_torques.col(3); // RL

        command_sender->setCommand(mujoco_go1_state);

        // Render at ~60 Hz
        if (mujoco_data->time - lastRenderSimTime >= renderInterval) {
            renderScene();
            lastRenderSimTime = mujoco_data->time;
        }

        // Store data in CSV files
        mujoco_datastream_row.str("");
        mujoco_datastream_row.clear();
        storeData(mujoco_go1_state, mujoco_datastream_row);
        mujocoDataLog.logLine(mujoco_datastream_row.str());

        mujoco_calcstream_row.str("");
        mujoco_calcstream_row.clear();
        storeCalcTimeData(duration_1, duration_2, duration_3, mujoco_calcstream_row);
        calcDataLog.logLine(mujoco_calcstream_row.str());

        mj_step(mujoco_model, mujoco_data);

        auto loop_end = std::chrono::high_resolution_clock::now();
        auto loop_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end - loop_start);
        auto remaining_time = loop_time - loop_elapsed;
        if (remaining_time > std::chrono::milliseconds(0)) {
            std::this_thread::sleep_for(remaining_time);
        }
    }

    mj_deleteData(mujoco_data);
    mj_deleteModel(mujoco_model);
    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    glfwDestroyWindow(window);
    glfwTerminate();
    // endwin(); // End ncurses mode if using velocity commands

    std::cout << "MuJoCo Simulation Complete!" << std::endl;
    return 0;
}

