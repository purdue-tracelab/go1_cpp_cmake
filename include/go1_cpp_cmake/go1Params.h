#ifndef GO1_PARAMS_H
#define GO1_PARAMS_H

/////////////////////////
// Physical parameters //
/////////////////////////

constexpr int NUM_LEG = 4;
constexpr double BASE_2_HIP_JOINT_X = 0.1881;
constexpr double BASE_2_HIP_JOINT_Y = 0.04675;

// Link lengths
constexpr double HIP_LINK_LENGTH = 0.08;
constexpr double THIGH_LINK_LENGTH = 0.213;
constexpr double CALF_LINK_LENGTH = 0.213;

// Torque limits
constexpr double TORQUE_MAX_HIP = 23.7;
constexpr double TORQUE_MAX_THIGH = 23.7;
constexpr double TORQUE_MAX_CALF = 35.55;

////////////////////////
// Startup parameters //
////////////////////////

// Desired joint positions in startup
constexpr double THIGH_RAD_STAND = 0.9;
constexpr double CALF_RAD_STAND = -1.8;

// Desired startup/shutdown joint PD gains
constexpr double SQUAT_JOINT_KP = 60;
constexpr double SQUAT_JOINT_KD = 3;

////////////////////////////////
// General control parameters //
////////////////////////////////

constexpr double DT_CTRL = 0.002; // General control frequency (500 Hz)
constexpr int STATE_EST_SELECT = 2; // O: naive KF, 1: kinematic KF, 2: two-stage KF, 3: extended KF
constexpr bool USE_EST_FOR_CONTROL = true; // false: no, use ground truth info, true: yes, use estimated info (ALWAYS USE ESTIMATE FOR HARDWARE)
constexpr double MUJOCO_CONTACT_THRESH = 3.0;
constexpr int UNITREE_SDK_CONTACT_THRESH = 100;

/////////////////////////////
// Swing leg PD parameters //
/////////////////////////////

constexpr int PLANNER_SELECT = 0; // 0: Raibert Heuristic, 1: Raibert Heuristic with Capture Point, 2: HT-LIP
constexpr int SWING_PD_SELECT = 1; // 0: Cartesian PD, 1: Joint PD
constexpr double SWING_TRAJ_SELECT = 0; // 0: Bezier, 1: Sinusoidal (sinusoidal broken atm)
constexpr int SWING_PHASE_MAX = 199; // swap between 0-99 (0.2 s gait cycle) and 0-199 (0.4 s gait cycle) for DT_CTRL = 0.002

// Cartesian space swing PD gains
constexpr double SWING_KP_CART = 3000;
constexpr double SWING_KD_CART = 40;

// Joint space swing PD gains
constexpr double SWING_KP_JOINT = 60;
constexpr double SWING_KD_JOINT = 3;

// Swing leg Bezier curve parameters
constexpr double WALK_HEIGHT = 0.27;
constexpr double STEP_HEIGHT = 0.13;
constexpr double FOOT_DELTA_X_LIMIT = 0.20;
constexpr double FOOT_DELTA_Y_LIMIT = 0.10;

////////////////////
// MPC parameters //
////////////////////

// MPC calculation timings
constexpr int MPC_HORIZON = 7; // with EIGEN_STACK_ALLOCATION_LIMIT, max = 7; without, max = 9
constexpr double DT_MPC_CTRL = 0.02; // MPC control frequency (50 Hz)
constexpr double DT_MPC = DT_CTRL * (1 + SWING_PHASE_MAX) / MPC_HORIZON; // MPC horizon time step (ctrl dt * # gait phases / # horizons), should see whole gait

// GRF bounds
constexpr double MU = 0.6;
constexpr double FZ_MIN = 0.0;
constexpr double FZ_MAX = 500.0;

// Helpful MPC matrix dimensions
constexpr int FRIC_PYR = 6;
constexpr int MPC_REF_DIM = 13 * MPC_HORIZON;
constexpr int MPC_INPUT_DIM = 3 * NUM_LEG * MPC_HORIZON;

///////////////////////////////////////////////
// Inertia sets (only use one set at a time) //
///////////////////////////////////////////////

// // MuJoCo go1.xml model w/o calf
// constexpr double ROBOT_MASS = 12.743448;
// constexpr double GO1_LUMPED_INERTIA_XX = 0.0718165;
// constexpr double GO1_LUMPED_INERTIA_XY = 0.0009962;
// constexpr double GO1_LUMPED_INERTIA_XZ = 0.0089032;
// constexpr double GO1_LUMPED_INERTIA_YY = 0.329118;
// constexpr double GO1_LUMPED_INERTIA_YZ = -0.000753;
// constexpr double GO1_LUMPED_INERTIA_ZZ = 0.372901;

// // MuJoCo go1_MATLAB.xml model w/o calf
// constexpr double ROBOT_MASS = 11.7914;
// constexpr double GO1_LUMPED_INERTIA_XX = 0.122975;
// constexpr double GO1_LUMPED_INERTIA_XY = 0.000659457;
// constexpr double GO1_LUMPED_INERTIA_XZ = 0.00902957;
// constexpr double GO1_LUMPED_INERTIA_YY = 0.302195;
// constexpr double GO1_LUMPED_INERTIA_YZ = -0.000964625;
// constexpr double GO1_LUMPED_INERTIA_ZZ = 0.286839;

// // Mujoco go1_MATLAB.xml model w/o thigh + calf
// constexpr double ROBOT_MASS = 11.7914;
// constexpr double GO1_LUMPED_INERTIA_XX = 0.0794534;
// constexpr double GO1_LUMPED_INERTIA_XY = -0.000271265;
// constexpr double GO1_LUMPED_INERTIA_XZ = 0.0003517;
// constexpr double GO1_LUMPED_INERTIA_YY = 0.150899;
// constexpr double GO1_LUMPED_INERTIA_YZ = -0.000226049;
// constexpr double GO1_LUMPED_INERTIA_ZZ = 0.109766;

// // MATLAB magnus_go1.urdf
// constexpr double ROBOT_MASS = 11.8;
// constexpr double GO1_LUMPED_INERTIA_XX = 0.1192;
// constexpr double GO1_LUMPED_INERTIA_XY = 0.0007;
// constexpr double GO1_LUMPED_INERTIA_XZ = -0.0008;
// constexpr double GO1_LUMPED_INERTIA_YY = 0.3328;
// constexpr double GO1_LUMPED_INERTIA_YZ = 0.0154;
// constexpr double GO1_LUMPED_INERTIA_ZZ = 0.3820;

// // MATLAB w/o calf links
// constexpr double ROBOT_MASS = 11.8;
// constexpr double GO1_LUMPED_INERTIA_XX = 0.0782;
// constexpr double GO1_LUMPED_INERTIA_XY = 0.0004;
// constexpr double GO1_LUMPED_INERTIA_XZ = -0.0011;
// constexpr double GO1_LUMPED_INERTIA_YY = 0.2951;
// constexpr double GO1_LUMPED_INERTIA_YZ = 0.0089;
// constexpr double GO1_LUMPED_INERTIA_ZZ = 0.3399;

// // Muqun's hardware code
// constexpr double ROBOT_MASS = 12.8;
// constexpr double GO1_LUMPED_INERTIA_XX = 0.0792;
// constexpr double GO1_LUMPED_INERTIA_XY = 0.0;
// constexpr double GO1_LUMPED_INERTIA_XZ = 0.0;
// constexpr double GO1_LUMPED_INERTIA_YY = 0.2085;
// constexpr double GO1_LUMPED_INERTIA_YZ = 0.0;
// constexpr double GO1_LUMPED_INERTIA_ZZ = 0.2265;

// Muqun and Leo's
constexpr double ROBOT_MASS = 12.03;
constexpr double GO1_LUMPED_INERTIA_XX = 0.0481;
constexpr double GO1_LUMPED_INERTIA_XY = -0.0002;
constexpr double GO1_LUMPED_INERTIA_XZ = -0.0003;
constexpr double GO1_LUMPED_INERTIA_YY = 0.2919;
constexpr double GO1_LUMPED_INERTIA_YZ = -0.0000;
constexpr double GO1_LUMPED_INERTIA_ZZ = 0.2887;

/////////////////////
// MPC weight sets //
/////////////////////

// current hardware default
constexpr double q_weight_1 = 25;
constexpr double q_weight_2 = 25;
constexpr double q_weight_3 = 25;
constexpr double q_weight_4 = 5;
constexpr double q_weight_5 = 5;
constexpr double q_weight_6 = 50;
constexpr double q_weight_7 = 0;
constexpr double q_weight_8 = 0;
constexpr double q_weight_9 = 0.1;
constexpr double q_weight_10 = 0.5;
constexpr double q_weight_11 = 0.5;
constexpr double q_weight_12 = 0.5;
constexpr double r_weight_val = 3e-5;

// // MIT
// constexpr double q_weight_1 = 1;
// constexpr double q_weight_2 = 1;
// constexpr double q_weight_3 = 1;
// constexpr double q_weight_4 = 5;
// constexpr double q_weight_5 = 5;
// constexpr double q_weight_6 = 50;
// constexpr double q_weight_7 = 0.1;
// constexpr double q_weight_8 = 0.1;
// constexpr double q_weight_9 = 1;
// constexpr double q_weight_10 = 1;
// constexpr double q_weight_11 = 1;
// constexpr double q_weight_12 = 1;
// constexpr double r_weight_val = 1e-6;

// // current simulation default
// constexpr double q_weight_1 = 500;
// constexpr double q_weight_2 = 500;
// constexpr double q_weight_3 = 500;
// constexpr double q_weight_4 = 1000;
// constexpr double q_weight_5 = 1000;
// constexpr double q_weight_6 = 1000;
// constexpr double q_weight_7 = 0.1;
// constexpr double q_weight_8 = 0.1;
// constexpr double q_weight_9 = 1;
// constexpr double q_weight_10 = 10;
// constexpr double q_weight_11 = 10;
// constexpr double q_weight_12 = 10;
// constexpr double r_weight_val = 1e-6;

// // MATLAB
// constexpr double q_weight_1 = 100;
// constexpr double q_weight_2 = 100;
// constexpr double q_weight_3 = 100;
// constexpr double q_weight_4 = 50;
// constexpr double q_weight_5 = 50;
// constexpr double q_weight_6 = 50;
// constexpr double q_weight_7 = 0.1;
// constexpr double q_weight_8 = 0.1;
// constexpr double q_weight_9 = 1;
// constexpr double q_weight_10 = 1;
// constexpr double q_weight_11 = 1;
// constexpr double q_weight_12 = 1;
// constexpr double r_weight_val = 1e-6;

// // Muqun and Leo's simulation
// constexpr double q_weight_1 = 2000;
// constexpr double q_weight_2 = 2000;
// constexpr double q_weight_3 = 2000;
// constexpr double q_weight_4 = 500;
// constexpr double q_weight_5 = 2000;
// constexpr double q_weight_6 = 2000;
// constexpr double q_weight_7 = 120;
// constexpr double q_weight_8 = 50;
// constexpr double q_weight_9 = 50;
// constexpr double q_weight_10 = 10;
// constexpr double q_weight_11 = 10;
// constexpr double q_weight_12 = 50;
// constexpr double r_weight_val = 1e-3;

// // Muqun and Leo's hardware
// constexpr double q_weight_1 = 15;
// constexpr double q_weight_2 = 12;
// constexpr double q_weight_3 = 10;
// constexpr double q_weight_4 = 1.5;
// constexpr double q_weight_5 = 1.5;
// constexpr double q_weight_6 = 35;
// constexpr double q_weight_7 = 0;
// constexpr double q_weight_8 = 0;
// constexpr double q_weight_9 = 0.3;
// constexpr double q_weight_10 = 0.2;
// constexpr double q_weight_11 = 0.2;
// constexpr double q_weight_12 = 0.2;
// constexpr double r_weight_val = 4e-5;

#endif //GO1_PARAMS_H