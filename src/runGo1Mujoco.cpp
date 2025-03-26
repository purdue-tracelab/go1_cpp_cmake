#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <thread>

// #include "go1_cpp_humble/go1GRFMPC.h"
#include "go1_cpp_cmake/go1StanceMPC.h"
#include "go1_cpp_cmake/go1KalmanFilter.h"

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
go1KalmanFilter mujoco_go1_estimator;
// grfMPC mujoco_go1_grf_proctor;
Eigen::Matrix<double, 12, 1> joint_torques_stacked;

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

//// currently retired grfMPC wrapper function to find MPC solution
// void findMPCSolution() {
//     mujoco_go1_grf_proctor.reset();
//     mujoco_go1_grf_proctor.calculateA(mujoco_go1_state.root_rpy);
//     mujoco_go1_grf_proctor.calculateB(mujoco_go1_state.go1_lumped_inertia, mujoco_go1_state.root_rpy, mujoco_go1_state.foot_pos);
//     mujoco_go1_grf_proctor.calculateMPCStates(mujoco_go1_state);
//     mujoco_go1_grf_proctor.calculateQPMats(mujoco_go1_state);
//     mujoco_go1_grf_proctor.solveMPC(mujoco_go1_state);
//
//     // std::cout << "MPC solution found" << std::endl;
//
// }

// Function to write Eigen::Vector in CSV format
template <typename VectorType>
void write_vector(const Eigen::MatrixBase<VectorType> &vec, std::ofstream &file) {
    for (size_t i = 0; i < vec.size(); ++i) {
        file << vec(i);
        if (i < vec.size() - 1) {
            file << ",";  // Add comma between elements
        }
    }
}

// Functions to store data in CSV file
void storeData(const go1State &state, std::ofstream &file) {
    if (!file.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
        return;
    }

    // Position
    write_vector(state.root_pos, file);
    file << ",";
    write_vector(state.root_pos_d, file);
    file << ",";

    // Linear velocity
    write_vector(state.root_lin_vel, file);  // root_lin_vel (e.g., linear velocity vector)
    file << ",";
    write_vector(state.root_lin_vel_d, file);  // root_lin_vel_d (desired velocity)
    file << ",";

    // RPY orientation
    write_vector(state.root_rpy, file);  // root_rpy (e.g., roll/pitch/yaw orientation)
    file << ",";
    write_vector(state.root_rpy_d, file);  // root_rpy_d (desired orientation)
    file << ",";

    // Angular velocity
    write_vector(state.root_ang_vel, file);  // root_ang_vel (angular velocity)
    file << ",";
    write_vector(state.root_ang_vel_d, file);  // root_ang_vel_d (desired angular velocity)
    file << ",";
 
    // Foot positions and velocities
    write_vector(state.foot_pos, file);  // foot positions
    file << ",";
    write_vector(state.foot_pos_abs, file);  // absolute foot positions
    file << ",";
    write_vector(state.foot_pos_liftoff, file);  // foot liftoff positions
    file << ",";
    write_vector(state.foot_pos_d, file);  // foot desired positions
    file << ",";
    write_vector(state.foot_vel_d, file);  // foot velocities
    file << ",";

    // Foot forces and joint torques
    Eigen::Matrix<double, 3, NUM_LEG> fut_grf = state.foot_forces_grf;
    Eigen::VectorXd flat_fut_grf = Eigen::Map<Eigen::VectorXd>(fut_grf.data(), fut_grf.size());  // flatten foot forces (GRF)
    write_vector(flat_fut_grf, file);  // foot forces (GRF)
    file << ",";

    Eigen::Matrix<double, 3, NUM_LEG> fut_swing = state.foot_forces_swing;
    Eigen::VectorXd flat_fut_swing = Eigen::Map<Eigen::VectorXd>(fut_swing.data(), fut_swing.size());  // flatten foot forces (swing)
    write_vector(flat_fut_swing, file);  // foot forces (swing)
    file << ",";

    Eigen::Matrix<double, 3, NUM_LEG> joint_torques = state.joint_torques;  // flatten joint torques
    Eigen::VectorXd flat_joint_torques = Eigen::Map<Eigen::VectorXd>(joint_torques.data(), joint_torques.size());  // flatten joint torques
    write_vector(flat_joint_torques, file);  // joint torques
    file << ",";

    // Foot contact states
    file << state.swing_phase << "," 
        << (state.contacts[0] ? 1 : 0) << "," 
        << (state.contacts[1] ? 1 : 0) << "," 
        << (state.contacts[2] ? 1 : 0) << "," 
        << (state.contacts[3] ? 1 : 0) << ",";

    // Estimated states
    write_vector(state.root_pos_est, file);
    file << ",";
    write_vector(state.root_lin_vel_est, file);
    file << ",";
    write_vector(state.root_lin_acc_est, file);
    file << ",";
    write_vector(state.root_rpy_est, file);
    file << "\n";
    
}

void writeCSVHeader(std::ofstream &file) {
    if (!file.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
        return;
    }
    
    file << "root_pos_x,root_pos_y,root_pos_z,root_pos_d_x,root_pos_d_y,root_pos_d_z,";
    file << "root_lin_vel_x,root_lin_vel_y,root_lin_vel_z,root_lin_vel_d_x,root_lin_vel_d_y,root_lin_vel_d_z,";
    file << "root_rpy_x,root_rpy_y,root_rpy_z,root_rpy_d_x,root_rpy_d_y,root_rpy_d_z,";
    file << "root_ang_vel_x,root_ang_vel_y,root_ang_vel_z,root_ang_vel_d_x,root_ang_vel_d_y,root_ang_vel_d_z,";
    file << "foot_pos_FR_x,foot_pos_FR_y,foot_pos_FR_z,foot_pos_FL_x,foot_pos_FL_y,foot_pos_FL_z,";
    file << "foot_pos_RR_x,foot_pos_RR_y,foot_pos_RR_z,foot_pos_RL_x,foot_pos_RL_y,foot_pos_RL_z,";
    file << "foot_pos_abs_FR_x,foot_pos_abs_FR_y,foot_pos_abs_FR_z,foot_pos_abs_FL_x,foot_pos_abs_FL_y,foot_pos_abs_FL_z,";
    file << "foot_pos_abs_RR_x,foot_pos_abs_RR_y,foot_pos_abs_RR_z,foot_pos_abs_RL_x,foot_pos_abs_RL_y,foot_pos_abs_RL_z,";
    file << "foot_pos_liftoff_FR_x,foot_pos_liftoff_FR_y,foot_pos_liftoff_FR_z,foot_pos_liftoff_FL_x,foot_pos_liftoff_FL_y,foot_pos_liftoff_FL_z,";
    file << "foot_pos_liftoff_RR_x,foot_pos_liftoff_RR_y,foot_pos_liftoff_RR_z,foot_pos_liftoff_RL_x,foot_pos_liftoff_RL_y,foot_pos_liftoff_RL_z,";
    file << "foot_pos_d_FR_x,foot_pos_d_FR_y,foot_pos_d_FR_z,foot_pos_d_FL_x,foot_pos_d_FL_y,foot_pos_d_FL_z,";
    file << "foot_pos_d_RR_x,foot_pos_d_RR_y,foot_pos_d_RR_z,foot_pos_d_RL_x,foot_pos_d_RL_y,foot_pos_d_RL_z,";
    file << "foot_vel_d_FR_x,foot_vel_d_FR_y,foot_vel_d_FR_z,foot_vel_d_FL_x,foot_vel_d_FL_y,foot_vel_d_FL_z,";
    file << "foot_vel_d_RR_x,foot_vel_d_RR_y,foot_vel_d_RR_z,foot_vel_d_RL_x,foot_vel_d_RL_y,foot_vel_d_RL_z,";
    file << "foot_forces_grf_FR_x,foot_forces_grf_FR_y,foot_forces_grf_FR_z,foot_forces_grf_FL_x,foot_forces_grf_FL_y,foot_forces_grf_FL_z,";
    file << "foot_forces_grf_RR_x,foot_forces_grf_RR_y,foot_forces_grf_RR_z,foot_forces_grf_RL_x,foot_forces_grf_RL_y,foot_forces_grf_RL_z,";
    file << "foot_forces_swing_FR_x,foot_forces_swing_FR_y,foot_forces_swing_FR_z,foot_forces_swing_FL_x,foot_forces_swing_FL_y,foot_forces_swing_FL_z,";
    file << "foot_forces_swing_RR_x,foot_forces_swing_RR_y,foot_forces_swing_RR_z,foot_forces_swing_RL_x,foot_forces_swing_RL_y,foot_forces_swing_RL_z,";
    file << "joint_torques_FR_hip,joint_torques_FR_thigh,joint_torques_FR_calf,joint_torques_FL_hip,joint_torques_FL_thigh,joint_torques_FL_calf,";
    file << "joint_torques_RR_hip,joint_torques_RR_thigh,joint_torques_RR_calf,joint_torques_RL_hip,joint_torques_RL_thigh,joint_torques_RL_calf,";
    file << "swing_phase,contact_FR,contact_FL,contact_RR,contact_RL,";
    file << "root_pos_est_x,root_pos_est_y,root_pos_est_z,";
    file << "root_lin_vel_est_x,root_lin_vel_est_y,root_lin_vel_est_z,";
    file << "root_lin_acc_est_x,root_lin_acc_est_y,root_lin_acc_est_z,";
    file << "root_rpy_est_x,root_rpy_est_y,root_rpy_est_z\n";

}

int main(void) {
    std::cout << "Initializing MuJoCo..." << std::endl;

    mujoco_go1_state.resetState();
    mujoco_go1_estimator.initFilter("accelerometer");
    // mujoco_go1_grf_proctor.reset(); // under maintenance
    joint_torques_stacked.setZero();

    // Load MuJoCo model
    char error[1000] = "";
    std::string model_path = "/home/memento/go1_cpp_cmake/models/go1.xml"; // remember to make this relative in the end

    mjVFS vfs;
    mj_defaultVFS(&vfs);

    if (mj_addFileVFS(&vfs, model_path.c_str(), model_path.c_str()) != 0) {
        std::cerr << "Failed to add XML to VFS: " << model_path.c_str() << std::endl;
        mj_deleteVFS(&vfs);
        return -1;
    }

    model = mj_loadXML(model_path.c_str(), &vfs, error, 1000);
    mj_deleteVFS(&vfs);

    if (!model) {
        std::cerr << "Failed to load MuJoCo model: " << error << std::endl;
        return -1;
    }

    data = mj_makeData(model);
    
    if (model->nkey > 0) {
        int keyframe_id = mj_name2id(model, mjOBJ_KEY, "standing");
        if (keyframe_id == -1) {
            std::cerr << "Keyframe 'standing' not found!" << std::endl;
            return -1;
        }
        mj_resetDataKeyframe(model, data, keyframe_id);
    } else {
        mj_resetData(model, data);
    }

    initVisualization();
    adjustCamera();

    const std::chrono::milliseconds loop_time(static_cast<long>(1000 * model->opt.timestep)); // 500 Hz loop time
    bool running = true;
    double lastMPCUpdateTime = -1.0; // calculate MPC solution at the beginning
    double mpcInterval = DT_MPC_CTRL;
    double lastRenderSimTime = -1.0; // render at the beginning
    double renderInterval = 1.0/60.0;

    mjtNum qpos_buffer[model->nq];
    mjtNum qvel_buffer[model->nv];
    int base_id = mj_name2id(model, mjOBJ_BODY, "trunk");
    int sensor_id = mj_name2id(model, mjOBJ_SENSOR, "lin_acc_sensor");
    int sensor_dim = model->sensor_dim[sensor_id];
    int sensor_adr = model->sensor_adr[sensor_id];

    // Desired initial state
    mujoco_go1_state.root_pos_d << 0, 0, 0.27;
    
    // Walking flag
    mujoco_go1_state.walking_mode = true;

    // Initialize the state from MuJoCo
    std::memcpy(qpos_buffer, data->qpos, model->nq * sizeof(mjtNum));
    std::memcpy(qvel_buffer, data->qvel, model->nv * sizeof(mjtNum));
    Eigen::Vector3d lin_acc = Eigen::Map<const Eigen::Vector3d>(data->cacc + 3 * base_id);
    Eigen::Vector3d lin_acc_meas = Eigen::Map<const Eigen::Vector3d>(data->sensordata + sensor_adr);
    
    mujoco_go1_state.updateStateFromMujoco(qpos_buffer, qvel_buffer, lin_acc, true);
    mujoco_go1_estimator.simpleKalmanFilterMujoco(mujoco_go1_state, lin_acc_meas);

    // Write header only once
    std::string filename = "/home/memento/go1_cpp_cmake/data/go1_mujoco_data.csv"; // remember to make this relative in the end
    std::cout << "Writing data to: " << filename << std::endl;
    std::ofstream file(filename, std::ios::trunc);
    writeCSVHeader(file);
    file.close();

    // Reopen the file in append mode
    file.open(filename, std::ios::app);

    if (!file.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
        return -1;
    }

    // Main simulation loop
    while (running && !glfwWindowShouldClose(window)) {
        std::cout << "Simulation time: " << data->time << std::endl;

        // // Desired states (walk in x-direction)
        // mujoco_go1_state.root_lin_vel_d << 0.5, 0, 0;
        // mujoco_go1_state.root_pos_d << 0.5*data->time, 0, WALK_HEIGHT;

        // // Desired states (walk in y-direction)
        // mujoco_go1_state.root_lin_vel_d << 0, 0.1, 0;
        // mujoco_go1_state.root_pos_d << 0, 0.1*data->time, WALK_HEIGHT;

        // // Desired states (walk in xy-direction)
        // mujoco_go1_state.root_lin_vel_d << 0.2, 0.2, 0;
        // mujoco_go1_state.root_pos_d << 0.2*data->time, 0.2*data->time, WALK_HEIGHT;

        // // Desired states (dynamic standing)
        // mujoco_go1_state.root_pos_d << 0, 0, 0.27 + 0.2*sin(2*data->time);
        // mujoco_go1_state.root_lin_vel_d << 0, 0, 0.2*cos(2*data->time);
        // mujoco_go1_state.root_rpy_d << 0, 0, 0.3*sin(data->time);
        // mujoco_go1_state.root_ang_vel_d << 0, 0, 0.3*cos(data->time);

        // // Desired states (dynamic walking)
        // mujoco_go1_state.root_pos_d << 0.5*data->time, 0.1*sin(data->time), 0.27 + 0.1*cos(data->time);
        // mujoco_go1_state.root_lin_vel_d << 0.5, 0.1*cos(data->time), -0.1*sin(data->time);
        // mujoco_go1_state.root_rpy_d << 0, 0, 0.3*sin(data->time);
        // mujoco_go1_state.root_ang_vel_d << 0, 0, 0.3*cos(data->time);

        // Extract joint position, joint velocity, and base linear acceleration from MuJoCo
        std::memcpy(qpos_buffer, data->qpos, model->nq * sizeof(mjtNum));
        std::memcpy(qvel_buffer, data->qvel, model->nv * sizeof(mjtNum));
        Eigen::Vector3d lin_acc = Eigen::Map<const Eigen::Vector3d>(data->cacc + 3 * base_id);
        Eigen::Vector3d lin_acc_meas = Eigen::Map<const Eigen::Vector3d>(data->sensordata + sensor_adr);

        // Update Go1 state from MuJoCo & perform swing PD force control
        mujoco_go1_state.updateStateFromMujoco(qpos_buffer, qvel_buffer, lin_acc);
        mujoco_go1_estimator.simpleKalmanFilterMujoco(mujoco_go1_state, lin_acc_meas);

        // Update MPC controller (40 Hz)
        if (data->time - lastMPCUpdateTime >= mpcInterval) {
            // findMPCSolution();
            go1StanceMPC(mujoco_go1_state);
            lastMPCUpdateTime = data->time;
        }

        // Convert foot forces to joint torques
        mujoco_go1_state.convertForcesToTorques(qpos_buffer);
        joint_torques_stacked << mujoco_go1_state.joint_torques.block<3, 1>(0, 0), // FR
                                mujoco_go1_state.joint_torques.block<3, 1>(0, 1), // FL
                                mujoco_go1_state.joint_torques.block<3, 1>(0, 2), // RR
                                mujoco_go1_state.joint_torques.block<3, 1>(0, 3); // RL

        for (int j = 0; j < model->nu; j++) {
            data->ctrl[j] = joint_torques_stacked(j, 0);
        }

        // Render at ~60 Hz
        if (data->time - lastRenderSimTime >= renderInterval) {
            renderScene();
            lastRenderSimTime = data->time;
        }

        // Store data in CSV file
        storeData(mujoco_go1_state, file);
        mj_step(model, data);
        std::this_thread::sleep_for(loop_time);
    }

    file.close();

    mj_deleteData(data);
    mj_deleteModel(model);
    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    glfwDestroyWindow(window);
    glfwTerminate();

    std::cout << "MuJoCo Simulation Complete!" << std::endl;
    return 0;
}
