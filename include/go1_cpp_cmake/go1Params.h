#ifndef GO1_PARAMS_H
#define GO1_PARAMS_H

/////////////////////////
// Physical parameters //
/////////////////////////

constexpr int NUM_LEG = 4;
constexpr double DELTA_X_HIP = 0.1881;
constexpr double DELTA_Y_HIP = 0.04675;
constexpr double DELTA_Y_HIP_JOINT = 0.08;
constexpr double TORQUE_MAX_HIP = 23.7;
constexpr double TORQUE_MAX_THIGH = 23.7;
constexpr double TORQUE_MAX_CALF = 35.55;

/////////////////////////////
// Swing leg PD parameters //
/////////////////////////////

constexpr int PLANNER_SELECT = 0; // 0: Raibert Heuristic with/without Capture Point, 1: HT-LIP
constexpr double SWING_KP = 2000;
constexpr double SWING_KD = 20;
constexpr double WALK_HEIGHT = 0.27; // check all places where this is called, consider removing it
constexpr double STEP_HEIGHT = 0.1;
constexpr double FOOT_DELTA_X_LIMIT = 0.20;
constexpr double FOOT_DELTA_Y_LIMIT = 0.10;
constexpr int SWING_PHASE_MAX = 199; // swap between 0-99 (0.2 s gait cycle) and 0-199 (0.4 s gait cycle)

////////////////////
// MPC parameters //
////////////////////

constexpr double DT_CTRL = 0.002; // General control frequency (500 Hz)
constexpr int MPC_HORIZON = 7; // limited by EIGEN_STACK_ALLOCATION_LIMIT, explore how to increase?
constexpr double DT_MPC_CTRL = 0.02; // MPC control frequency (50 Hz)
constexpr double DT_MPC = DT_CTRL*(1 + SWING_PHASE_MAX)/MPC_HORIZON; // MPC horizon time step (ctrl dt * # gait phases / # horizons), should see whole gait
constexpr double MU = 0.6;
constexpr double FZ_MIN = 0.0;
constexpr double FZ_MAX = 500.0;
constexpr int FRIC_PYR = 6;
constexpr int MPC_REF_DIM = 13 * MPC_HORIZON;
constexpr int MPC_INPUT_DIM = 3 * NUM_LEG * MPC_HORIZON;
constexpr double ALEPH = 0.1; // ranges from 0 to 1, 0.1 is slow, 0.8 is fast (use slow for standing)
constexpr double BETTA = 0.1; // ranges from 0 to 1, 0.1 is slow, 0.8 is fast (use slow for standing)

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

// MATLAB
constexpr double ROBOT_MASS = 11.8;
constexpr double GO1_LUMPED_INERTIA_XX = 0.1192;
constexpr double GO1_LUMPED_INERTIA_XY = 0.0007;
constexpr double GO1_LUMPED_INERTIA_XZ = -0.0008;
constexpr double GO1_LUMPED_INERTIA_YY = 0.3328;
constexpr double GO1_LUMPED_INERTIA_YZ = 0.0154;
constexpr double GO1_LUMPED_INERTIA_ZZ = 0.3820;

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

// // Muqun and Leo's
// constexpr double ROBOT_MASS = 12.03;
// constexpr double GO1_LUMPED_INERTIA_XX = 0.0481;
// constexpr double GO1_LUMPED_INERTIA_XY = -0.0002;
// constexpr double GO1_LUMPED_INERTIA_XZ = -0.0003;
// constexpr double GO1_LUMPED_INERTIA_YY = 0.2919;
// constexpr double GO1_LUMPED_INERTIA_YZ = -0.0000;
// constexpr double GO1_LUMPED_INERTIA_ZZ = 0.2887;

/////////////////////
// MPC weight sets //
/////////////////////

// // Experimental
// constexpr double q_weight_1 = 8000;
// constexpr double q_weight_2 = 8000;
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

// current default
constexpr double q_weight_1 = 500;
constexpr double q_weight_2 = 500;
constexpr double q_weight_3 = 500;
constexpr double q_weight_4 = 1000;
constexpr double q_weight_5 = 1000;
constexpr double q_weight_6 = 1000;
constexpr double q_weight_7 = 0.1;
constexpr double q_weight_8 = 0.1;
constexpr double q_weight_9 = 1;
constexpr double q_weight_10 = 10;
constexpr double q_weight_11 = 10;
constexpr double q_weight_12 = 10;
constexpr double r_weight_val = 1e-6;

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

// // Muqun and Leo's
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

#endif //GO1_PARAMS_H