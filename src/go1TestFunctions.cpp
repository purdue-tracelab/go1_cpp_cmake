#include "go1_cpp_cmake/go1TestFunctions.h"

/*
    This serves as a simple testing platform for any functions. You
    can adjust this to whatever you like, since this is meant to help
    verify your understanding of the functions, but also debug issues
    with performance in the MuJoCo simulation, like the current tilting
    issue during regular standing. Keep in mind that the current answer
    keys are not correct, expect for when no feet are expected to be on
    the ground (contact = false).
*/

go1TestFunctions::go1TestFunctions() {
/*
    Initializes the test proctor for your use.
*/
    std::cout << std::fixed << std::setprecision(5) << "\n";
    std::cout << "#############################################\n";
    std::cout << "### go1TestFunctions has been initialized ###\n";
    std::cout << "#############################################\n" << std::endl;

}

int go1TestFunctions::testZeroPosErrorGRF() {
/*
    This is meant to test the grfMPC behavior when there is no error between
    the current and desired state. It should have no problem standing up, but
    explore with these cases to see if you can figure out why the system breaks
    in MuJoCo at the moment. It could be inertia, mass, rotations, etc.
*/
    // initialize objects
    tester_state.resetState();

    mjtNum temp_joint_angles[19] = {0.0, 0.0, 0.27, 1.0, 0.0, 0.0, 0.0, 0.0, 0.9, -1.8, 0.0, 0.9, -1.8, 0.0, 0.9, -1.8, 0.0, 0.9, -1.8};
    mjtNum temp_joint_velocities[18] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    std::memcpy(tester_joint_angles, temp_joint_angles, sizeof(temp_joint_angles));
    std::memcpy(tester_joint_velocities, temp_joint_velocities, sizeof(temp_joint_velocities));
    Eigen::Vector3d tester_lin_acc (0.0, 0.0, -9.81);

    tester_state.updateStateFromMujoco(tester_joint_angles, tester_joint_velocities, tester_lin_acc);
    tester_state.root_pos_d << 0, 0, 0.27;

    int subtestsPassed = 0;

    std::cout << "##################################################################\n";
    std::cout << "### Test: grfMPC w/ no error between current and desired state ###\n";
    std::cout << "##################################################################" << std::endl;
    std::cout << "-- Subtest 1: All feet on ground --" << std::endl;
    
    go1MPC tester_mpc1;
    tester_mpc1.solveMPCForState(tester_state);

    std::cout << "\nC++ GRF:\n" << tester_state.foot_forces_grf << std::endl;

    Eigen::Matrix<double, 3, NUM_LEG> matlabForces;
    matlabForces << -2.36255459640233e-13, 2.11386463888630e-13, -2.43360886997834e-13, 3.55271367880050e-13, 
                    3.01980662698043e-13, 9.05941988094128e-14, -3.76587649952853e-13, 7.10542735760100e-15, 
                    -28.2619400564001, -28.2619400563999, -28.2619400563998, -28.2619400563999;

    std::cout << "\nMATLAB GRF:\n" << matlabForces << std::endl;

    tester_state.convertForcesToTorquesMujoco(tester_joint_angles);

    std::cout << "\nC++ torques:\n" << tester_state.joint_torques << std::endl;

    Eigen::Matrix<double, 3, NUM_LEG> matlabTorques;
    matlabTorques << 2.26095520451210, -2.26095520451196, 2.26095520451187, -2.26095520451199, 
                    7.11666388192843e-14, -6.44598400946920e-14, 7.33069888589620e-14, -1.07017501983886e-13,
                    4.25663661569747, 4.25663661569736, 4.25663661569742, 4.25663661569735;

    std::cout << "\nMATLAB torques:\n" << matlabTorques << std::endl;

    int subtestCheck = 0;

    if ((matlabForces - tester_state.foot_forces_grf).norm() <= 1e-3) {
        std::cout << "\n--Forces passed--" << std::endl;
        subtestCheck++;
    } else {
        std::cout << "\n--Forces failed: " << (matlabForces - tester_state.foot_forces_grf).norm() << " norm error--" << std::endl;
    }

    if ((matlabTorques - tester_state.joint_torques).norm() <= 1e-3) {
        std::cout << "\n--Torques passed--" << std::endl;
        subtestCheck++;
    } else {
        std::cout << "\n--Torques failed: " << (matlabTorques - tester_state.joint_torques).norm() << " norm error--" << std::endl;
    }

    if (subtestCheck == 2) {
        subtestsPassed++;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << "\n-- Subtest 2: No feet on ground --" << std::endl;

    tester_state.resetState();
    tester_state.updateStateFromMujoco(tester_joint_angles, tester_joint_velocities, tester_lin_acc);
    tester_state.root_pos_d << 0, 0, 0.335;

    for (int i = 0; i < NUM_LEG; ++i) {
        tester_state.contacts[i] = false;
    }
    
    go1MPC tester_mpc2;
    tester_mpc2.solveMPCForState(tester_state);

    std::cout << "\nC++ GRF:\n" << tester_state.foot_forces_grf << std::endl;

    matlabForces.setZero();
    matlabForces << 3.66408292595821e-14, -1.71182512609391e-14, -4.77881623162091e-14, -1.75658099177412e-14,
                    -3.09474668114262e-14, -1.20736753927986e-15, 3.13846171273724e-14, -2.84494650060196e-14,
                    5.68434188608080e-14, -2.84217094304040e-14, -2.84217094304040e-14, 2.84217094304040e-14;

    std::cout << "\nMATLAB GRF:\n" << matlabForces << std::endl;

    tester_state.convertForcesToTorquesMujoco(tester_joint_angles);

    std::cout << "\nC++ torques:\n" << tester_state.joint_torques << std::endl;

    matlabTorques.setZero();
    matlabTorques << -1.38697012207422e-14, -2.63742904633068e-15, 1.17276461582041e-14, -6.29602414662106e-15,
                    -1.10372249848811e-14, 5.15648783973139e-15, 1.43951081281841e-14, 5.29130481000406e-15,
                    -1.40800126511514e-14, 6.85894399922114e-15, 1.14782541434475e-14, -1.63504767435342e-15;

    std::cout << "\nMATLAB torques:\n" << matlabTorques << std::endl;

    subtestCheck = 0;

    if ((matlabForces - tester_state.foot_forces_grf).norm() <= 1e-3) {
        std::cout << "\n--Forces passed--" << std::endl;
        subtestCheck++;
    } else {
        std::cout << "\n--Forces failed: " << (matlabForces - tester_state.foot_forces_grf).norm() << " norm error--" << std::endl;
    }

    if ((matlabTorques - tester_state.joint_torques).norm() <= 1e-3) {
        std::cout << "\n--Torques passed--" << std::endl;
        subtestCheck++;
    } else {
        std::cout << "\n--Torques failed: " << (matlabTorques - tester_state.joint_torques).norm() << " norm error--" << std::endl;
    }

    if (subtestCheck == 2) {
        subtestsPassed++;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << "\n-- Subtest 3: FL + RR --" << std::endl;

    tester_state.resetState();
    tester_state.updateStateFromMujoco(tester_joint_angles, tester_joint_velocities, tester_lin_acc);
    tester_state.root_pos_d << 0, 0, 0.335;

    tester_state.contacts[0] = false;
    tester_state.contacts[3] = false;
    
    go1MPC tester_mpc3;
    tester_mpc3.solveMPCForState(tester_state);

    std::cout << "\nC++ GRF:\n" << tester_state.foot_forces_grf << std::endl;

    matlabForces.setZero();
    matlabForces << -1.97982255589757e-13, 1.49213974509621e-13, 7.10542735760100e-15, 1.81613404870440e-13,
                    -8.08415834274712e-14, 1.27897692436818e-13, -4.12114786740858e-13, 9.70161451174789e-14,
                    -2.13162820728030e-13, -53.2288858059343, -53.2288858059343, 2.29150032282632e-13;

    std::cout << "\nMATLAB GRF:\n" << matlabForces << std::endl;

    tester_state.convertForcesToTorquesMujoco(tester_joint_angles);

    std::cout << "\nC++ torques:\n" << tester_state.joint_torques << std::endl;

    matlabTorques.setZero();
    matlabTorques << -7.29868150705574e-15, -4.25831086447470, 4.25831086447462, 4.75559323479954e-14,
                    5.96376976753854e-14, -4.64247491981545e-14, -2.14035003967772e-15, -5.47069498789013e-14,
                    6.19240994328586e-14, 8.01700180108503, 8.01700180108505, -6.18666193292539e-14;

    std::cout << "\nMATLAB torques:\n" << matlabTorques << std::endl;

    subtestCheck = 0;

    if ((matlabForces - tester_state.foot_forces_grf).norm() <= 1e-3) {
        std::cout << "\n--Forces passed--" << std::endl;
        subtestCheck++;
    } else {
        std::cout << "\n--Forces failed: " << (matlabForces - tester_state.foot_forces_grf).norm() << " norm error--" << std::endl;
    }

    if ((matlabTorques - tester_state.joint_torques).norm() <= 1e-3) {
        std::cout << "\n--Torques passed--" << std::endl;
        subtestCheck++;
    } else {
        std::cout << "\n--Torques failed: " << (matlabTorques - tester_state.joint_torques).norm() << " norm error--" << std::endl;
    }

    if (subtestCheck == 2) {
        subtestsPassed++;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << "\n-- Subtest 4: FR + RL --" << std::endl;

    tester_state.resetState();
    tester_state.updateStateFromMujoco(tester_joint_angles, tester_joint_velocities, tester_lin_acc);
    tester_state.root_pos_d << 0, 0, 0.335;

    tester_state.contacts[1] = false;
    tester_state.contacts[2] = false;
    
    go1MPC tester_mpc4;
    tester_mpc4.solveMPCForState(tester_state);

    std::cout << "\nC++ GRF:\n" << tester_state.foot_forces_grf << std::endl;

    matlabForces.setZero();
    matlabForces << -4.12114786740858e-13, 3.44778025573866e-13, -3.58896939944842e-13, 3.44613226843649e-13,
                    5.11590769747272e-13, 1.91034688068470e-13, -1.87200949186561e-13, -2.09610107049230e-13,
                    -53.2288858059345, -1.58095758706622e-13, 1.04805053524615e-13, -53.2288858059349;

    std::cout << "\nMATLAB GRF:\n" << matlabForces << std::endl;

    tester_state.convertForcesToTorquesMujoco(tester_joint_angles);

    std::cout << "\nC++ torques:\n" << tester_state.joint_torques << std::endl;

    matlabTorques.setZero();
    matlabTorques << 4.25831086447492, 4.48972386612508e-14, -6.47744761036932e-14, -4.25831086447485,
                    1.24140302301308e-13, -1.03856618832028e-13, 1.08109623952374e-13, -1.03806976924370e-13,
                    8.01700180108514, -2.81169152245994e-14, 3.82697304335638e-14, 8.01700180108508;

    std::cout << "\nMATLAB torques:\n" << matlabTorques << std::endl;

    subtestCheck = 0;

    if ((matlabForces - tester_state.foot_forces_grf).norm() <= 1e-3) {
        std::cout << "\n--Forces passed--" << std::endl;
        subtestCheck++;
    } else {
        std::cout << "\n--Forces failed: " << (matlabForces - tester_state.foot_forces_grf).norm() << " norm error--" << std::endl;
    }

    if ((matlabTorques - tester_state.joint_torques).norm() <= 1e-3) {
        std::cout << "\n--Torques passed--" << std::endl;
        subtestCheck++;
    } else {
        std::cout << "\n--Torques failed: " << (matlabTorques - tester_state.joint_torques).norm() << " norm error--" << std::endl;
    }

    if (subtestCheck == 2) {
        subtestsPassed++;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////

    if (subtestsPassed == 4) {
        std::cout << "\nAll subtests passed successfully!" << std::endl;
        return 1;
    } else {
        std::cout << "\nSubtests passed: " << subtestsPassed << "/4\n" << std::endl;
        return 0;
    }
}

int go1TestFunctions::testNonzeroPosErrorGRF() {
/*
    This is meant to test the grfMPC behavior when we do have error between
    the current and desired state. It should accelerate upwards more than before,
    but explore with these cases to see if you can figure out why the system breaks
    in MuJoCo at the moment. It could be inertia, mass, rotations, etc.
*/
    // grfMPC mpc_proctor; // retired until further notice;
    // initialize current tester state
    tester_state.resetState();

    mjtNum temp_joint_angles[19] = {0.0, 0.0, 0.27, 1.0, 0.0, 0.0, 0.0, 0.0, 0.9, -1.8, 0.0, 0.9, -1.8, 0.0, 0.9, -1.8, 0.0, 0.9, -1.8};
    mjtNum temp_joint_velocities[18] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Eigen::Vector3d temp_desired_pos (0.0, 0.0, 0.32);

    std::memcpy(tester_joint_angles, temp_joint_angles, sizeof(temp_joint_angles));
    std::memcpy(tester_joint_velocities, temp_joint_velocities, sizeof(temp_joint_velocities));
    Eigen::Vector3d tester_lin_acc (0.0, 0.0, -9.81);

    tester_state.updateStateFromMujoco(tester_joint_angles, tester_joint_velocities, tester_lin_acc);
    tester_state.root_pos_d = temp_desired_pos;

    int subtestsPassed = 0;

    std::cout << "#######################################################################\n";
    std::cout << "### Test: grfMPC w/ nonzero error between current and desired state ###\n";
    std::cout << "#######################################################################\n" << std::endl;
    std::cout << "** Note that the answer keys for this test are not properly set, so expect some error based on your desired position **" << std::endl;
    std::cout << "-- Subtest 1: All feet on ground --" << std::endl;

    go1MPC tester_mpc1;
    tester_mpc1.solveMPCForState(tester_state);

    std::cout << "\nC++ GRF:\n" << tester_state.foot_forces_grf << std::endl;

    Eigen::Matrix<double, 3, NUM_LEG> matlabForces;
    matlabForces << -2.36255459640233e-13, 2.11386463888630e-13, -2.43360886997834e-13, 3.55271367880050e-13, 
                    3.01980662698043e-13, 9.05941988094128e-14, -3.76587649952853e-13, 7.10542735760100e-15, 
                    -28.2619400564001, -28.2619400563999, -28.2619400563998, -28.2619400563999;

    std::cout << "\nMATLAB GRF:\n" << matlabForces << std::endl;

    tester_state.convertForcesToTorquesMujoco(tester_joint_angles);

    std::cout << "\nC++ torques:\n" << tester_state.joint_torques << std::endl;

    Eigen::Matrix<double, 3, NUM_LEG> matlabTorques;
    matlabTorques << 2.26095520451210, -2.26095520451196, 2.26095520451187, -2.26095520451199, 
                    7.11666388192843e-14, -6.44598400946920e-14, 7.33069888589620e-14, -1.07017501983886e-13,
                    4.25663661569747, 4.25663661569736, 4.25663661569742, 4.25663661569735;

    std::cout << "\nMATLAB torques:\n" << matlabTorques << std::endl;

    int subtestCheck = 0;

    if ((matlabForces - tester_state.foot_forces_grf).norm() <= 1e-3) {
        std::cout << "\n--Forces passed--" << std::endl;
        subtestCheck++;
    } else {
        std::cout << "\n--Forces failed: " << (matlabForces - tester_state.foot_forces_grf).norm() << " norm error--" << std::endl;
    }

    if ((matlabTorques - tester_state.joint_torques).norm() <= 1e-3) {
        std::cout << "\n--Torques passed--" << std::endl;
        subtestCheck++;
    } else {
        std::cout << "\n--Torques failed: " << (matlabTorques - tester_state.joint_torques).norm() << " norm error--" << std::endl;
    }

    if (subtestCheck == 2) {
        subtestsPassed++;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << "\n-- Subtest 2: No feet on ground --" << std::endl;

    tester_state.resetState();
    tester_state.updateStateFromMujoco(tester_joint_angles, tester_joint_velocities, tester_lin_acc);
    tester_state.root_pos_d = temp_desired_pos;

    for (int i = 0; i < NUM_LEG; ++i) {
        tester_state.contacts[i] = false;
    }

    go1MPC tester_mpc2;
    tester_mpc2.solveMPCForState(tester_state);

    std::cout << "\nC++ GRF:\n" << tester_state.foot_forces_grf << std::endl;

    matlabForces.setZero();
    matlabForces << 3.66408292595821e-14, -1.71182512609391e-14, -4.77881623162091e-14, -1.75658099177412e-14,
                    -3.09474668114262e-14, -1.20736753927986e-15, 3.13846171273724e-14, -2.84494650060196e-14,
                    5.68434188608080e-14, -2.84217094304040e-14, -2.84217094304040e-14, 2.84217094304040e-14;

    std::cout << "\nMATLAB GRF:\n" << matlabForces << std::endl;

    tester_state.convertForcesToTorquesMujoco(tester_joint_angles);

    std::cout << "\nC++ torques:\n" << tester_state.joint_torques << std::endl;

    matlabTorques.setZero();
    matlabTorques << -1.38697012207422e-14, -2.63742904633068e-15, 1.17276461582041e-14, -6.29602414662106e-15,
                    -1.10372249848811e-14, 5.15648783973139e-15, 1.43951081281841e-14, 5.29130481000406e-15,
                    -1.40800126511514e-14, 6.85894399922114e-15, 1.14782541434475e-14, -1.63504767435342e-15;

    std::cout << "\nMATLAB torques:\n" << matlabTorques << std::endl;

    subtestCheck = 0;

    if ((matlabForces - tester_state.foot_forces_grf).norm() <= 1e-3) {
        std::cout << "\n--Forces passed--" << std::endl;
        subtestCheck++;
    } else {
        std::cout << "\n--Forces failed: " << (matlabForces - tester_state.foot_forces_grf).norm() << " norm error--" << std::endl;
    }

    if ((matlabTorques - tester_state.joint_torques).norm() <= 1e-3) {
        std::cout << "\n--Torques passed--" << std::endl;
        subtestCheck++;
    } else {
        std::cout << "\n--Torques failed: " << (matlabTorques - tester_state.joint_torques).norm() << " norm error--" << std::endl;
    }

    if (subtestCheck == 2) {
        subtestsPassed++;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << "\n-- Subtest 3: FL + RR --" << std::endl;

    tester_state.resetState();
    tester_state.updateStateFromMujoco(tester_joint_angles, tester_joint_velocities, tester_lin_acc);
    tester_state.root_pos_d = temp_desired_pos;

    tester_state.contacts[0] = false;
    tester_state.contacts[3] = false;

    go1MPC tester_mpc3;
    tester_mpc3.solveMPCForState(tester_state);

    std::cout << "\nC++ GRF:\n" << tester_state.foot_forces_grf << std::endl;

    matlabForces.setZero();
    matlabForces << -1.97982255589757e-13, 1.49213974509621e-13, 7.10542735760100e-15, 1.81613404870440e-13,
                    -8.08415834274712e-14, 1.27897692436818e-13, -4.12114786740858e-13, 9.70161451174789e-14,
                    -2.13162820728030e-13, -53.2288858059343, -53.2288858059343, 2.29150032282632e-13;

    std::cout << "\nMATLAB GRF:\n" << matlabForces << std::endl;

    tester_state.convertForcesToTorquesMujoco(tester_joint_angles);

    std::cout << "\nC++ torques:\n" << tester_state.joint_torques << std::endl;

    matlabTorques.setZero();
    matlabTorques << -7.29868150705574e-15, -4.25831086447470, 4.25831086447462, 4.75559323479954e-14,
                    5.96376976753854e-14, -4.64247491981545e-14, -2.14035003967772e-15, -5.47069498789013e-14,
                    6.19240994328586e-14, 8.01700180108503, 8.01700180108505, -6.18666193292539e-14;

    std::cout << "\nMATLAB torques:\n" << matlabTorques << std::endl;

    subtestCheck = 0;

    if ((matlabForces - tester_state.foot_forces_grf).norm() <= 1e-3) {
        std::cout << "\n--Forces passed--" << std::endl;
        subtestCheck++;
    } else {
        std::cout << "\n--Forces failed: " << (matlabForces - tester_state.foot_forces_grf).norm() << " norm error--" << std::endl;
    }

    if ((matlabTorques - tester_state.joint_torques).norm() <= 1e-3) {
        std::cout << "\n--Torques passed--" << std::endl;
        subtestCheck++;
    } else {
        std::cout << "\n--Torques failed: " << (matlabTorques - tester_state.joint_torques).norm() << " norm error--" << std::endl;
    }

    if (subtestCheck == 2) {
        subtestsPassed++;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << "\n-- Subtest 4: FR + RL --" << std::endl;

    tester_state.resetState();
    tester_state.updateStateFromMujoco(tester_joint_angles, tester_joint_velocities, tester_lin_acc);
    tester_state.root_pos_d = temp_desired_pos;

    tester_state.contacts[1] = false;
    tester_state.contacts[2] = false;

    go1MPC tester_mpc4;
    tester_mpc4.solveMPCForState(tester_state);

    std::cout << "\nC++ GRF:\n" << tester_state.foot_forces_grf << std::endl;

    matlabForces.setZero();
    matlabForces << -4.12114786740858e-13, 3.44778025573866e-13, -3.58896939944842e-13, 3.44613226843649e-13,
                    5.11590769747272e-13, 1.91034688068470e-13, -1.87200949186561e-13, -2.09610107049230e-13,
                    -53.2288858059345, -1.58095758706622e-13, 1.04805053524615e-13, -53.2288858059349;

    std::cout << "\nMATLAB GRF:\n" << matlabForces << std::endl;

    tester_state.convertForcesToTorquesMujoco(tester_joint_angles);

    std::cout << "\nC++ torques:\n" << tester_state.joint_torques << std::endl;

    matlabTorques.setZero();
    matlabTorques << 4.25831086447492, 4.48972386612508e-14, -6.47744761036932e-14, -4.25831086447485,
                    1.24140302301308e-13, -1.03856618832028e-13, 1.08109623952374e-13, -1.03806976924370e-13,
                    8.01700180108514, -2.81169152245994e-14, 3.82697304335638e-14, 8.01700180108508;

    std::cout << "\nMATLAB torques:\n" << matlabTorques << std::endl;

    subtestCheck = 0;

    if ((matlabForces - tester_state.foot_forces_grf).norm() <= 1e-3) {
        std::cout << "\n--Forces passed--" << std::endl;
        subtestCheck++;
    } else {
        std::cout << "\n--Forces failed: " << (matlabForces - tester_state.foot_forces_grf).norm() << " norm error--" << std::endl;
    }

    if ((matlabTorques - tester_state.joint_torques).norm() <= 1e-3) {
        std::cout << "\n--Torques passed--" << std::endl;
        subtestCheck++;
    } else {
        std::cout << "\n--Torques failed: " << (matlabTorques - tester_state.joint_torques).norm() << " norm error--" << std::endl;
    }

    if (subtestCheck == 2) {
        subtestsPassed++;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////

    if (subtestsPassed == 4) {
        std::cout << "\nAll subtests passed successfully!" << std::endl;
        return 1;
    } else {
        std::cout << "\nSubtests passed: " << subtestsPassed << "/4\n" << std::endl;
        return 0;
    }
}

int go1TestFunctions::testZeroPosErrorWalk() {
/*
    This is meant to test the grfMPC + swing PD behavior when there is 
    no error between the current and desired state during walking.
*/
    // grfMPC mpc_proctor; // retired until further notice;
    // initialize current tester state
    tester_state.resetState();
    tester_state.walking_mode = true;

    mjtNum temp_joint_angles[19] = {0.0, 0.0, 0.27, 1.0, 0.0, 0.0, 0.0, 0.0, 0.9, -1.8, 0.0, 0.9, -1.8, 0.0, 0.9, -1.8, 0.0, 0.9, -1.8};
    mjtNum temp_joint_velocities[18] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    std::memcpy(tester_joint_angles, temp_joint_angles, sizeof(temp_joint_angles));
    std::memcpy(tester_joint_velocities, temp_joint_velocities, sizeof(temp_joint_velocities));
    Eigen::Vector3d tester_lin_acc (0.0, 0.0, -9.81);
    tester_state.root_pos_d << 0, 0, 0.27;

    int subtestsPassed = 0;

    std::cout << "###################################################################\n";
    std::cout << "### Test: Walking w/ no error between current and desired state ###\n";
    std::cout << "###################################################################" << std::endl;
    std::cout << "-- Subtest 1: FR + RL stance, FL + RR swing --" << std::endl;

    tester_state.updateStateFromMujoco(tester_joint_angles, tester_joint_velocities, tester_lin_acc);

    go1MPC tester_mpc1;
    tester_mpc1.solveMPCForState(tester_state);

    std::cout << "\nC++ GRF:\n" << tester_state.foot_forces_grf << std::endl;
    std::cout << "\nC++ swing forces:\n" << tester_state.foot_forces_swing << std::endl;

    Eigen::Matrix<double, 3, NUM_LEG> matlabForces;
    matlabForces << -2.36255459640233e-13, 2.11386463888630e-13, -2.43360886997834e-13, 3.55271367880050e-13, 
                    3.01980662698043e-13, 9.05941988094128e-14, -3.76587649952853e-13, 7.10542735760100e-15, 
                    -28.2619400564001, -28.2619400563999, -28.2619400563998, -28.2619400563999;

    std::cout << "\nMATLAB forces:\n" << matlabForces << std::endl;

    tester_state.convertForcesToTorquesMujoco(tester_joint_angles);

    std::cout << "\nC++ torques:\n" << tester_state.joint_torques << std::endl;

    Eigen::Matrix<double, 3, NUM_LEG> matlabTorques;
    matlabTorques << 2.26095520451210, -2.26095520451196, 2.26095520451187, -2.26095520451199, 
                    7.11666388192843e-14, -6.44598400946920e-14, 7.33069888589620e-14, -1.07017501983886e-13,
                    4.25663661569747, 4.25663661569736, 4.25663661569742, 4.25663661569735;

    std::cout << "\nMATLAB torques:\n" << matlabTorques << std::endl;

    int subtestCheck = 0;

    if ((matlabForces - (tester_state.foot_forces_grf + tester_state.foot_forces_swing)).norm() <= 1e-3) {
        std::cout << "\n--Forces passed--" << std::endl;
        subtestCheck++;
    } else {
        std::cout << "\n--Forces failed: " << (matlabForces - (tester_state.foot_forces_grf + tester_state.foot_forces_swing)).norm() << " norm error--" << std::endl;
    }

    if ((matlabTorques - tester_state.joint_torques).norm() <= 1e-3) {
        std::cout << "\n--Torques passed--" << std::endl;
        subtestCheck++;
    } else {
        std::cout << "\n--Torques failed: " << (matlabTorques - tester_state.joint_torques).norm() << " norm error--" << std::endl;
    }

    if (subtestCheck == 2) {
        subtestsPassed++;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << "\n-- Subtest 2: FL + RR stance, FR + RL swing --" << std::endl;

    tester_state.resetState();
    tester_state.walking_mode = true;
    tester_state.swing_phase = SWING_PHASE_MAX/2 + 1;
    tester_state.root_pos_d << 0, 0, 0.27;

    tester_state.updateStateFromMujoco(tester_joint_angles, tester_joint_velocities, tester_lin_acc);

    go1MPC tester_mpc2;
    tester_mpc2.solveMPCForState(tester_state);

    matlabForces.setZero();
    matlabForces << -2.36255459640233e-13, 2.11386463888630e-13, -2.43360886997834e-13, 3.55271367880050e-13, 
                    3.01980662698043e-13, 9.05941988094128e-14, -3.76587649952853e-13, 7.10542735760100e-15, 
                    -28.2619400564001, -28.2619400563999, -28.2619400563998, -28.2619400563999;

    std::cout << "\nMATLAB forces:\n" << matlabForces << std::endl;

    tester_state.convertForcesToTorquesMujoco(tester_joint_angles);

    std::cout << "\nC++ torques:\n" << tester_state.joint_torques << std::endl;

    matlabTorques.setZero();
    matlabTorques << 2.26095520451210, -2.26095520451196, 2.26095520451187, -2.26095520451199, 
                    7.11666388192843e-14, -6.44598400946920e-14, 7.33069888589620e-14, -1.07017501983886e-13,
                    4.25663661569747, 4.25663661569736, 4.25663661569742, 4.25663661569735;

    std::cout << "\nMATLAB torques:\n" << matlabTorques << std::endl;

    subtestCheck = 0;

    if ((matlabForces - (tester_state.foot_forces_grf + tester_state.foot_forces_swing)).norm() <= 1e-3) {
        std::cout << "\n--Forces passed--" << std::endl;
        subtestCheck++;
    } else {
        std::cout << "\n--Forces failed: " << (matlabForces - (tester_state.foot_forces_grf + tester_state.foot_forces_swing)).norm() << " norm error--" << std::endl;
    }

    if ((matlabTorques - tester_state.joint_torques).norm() <= 1e-3) {
        std::cout << "\n--Torques passed--" << std::endl;
        subtestCheck++;
    } else {
        std::cout << "\n--Torques failed: " << (matlabTorques - tester_state.joint_torques).norm() << " norm error--" << std::endl;
    }

    if (subtestCheck == 2) {
        subtestsPassed++;
    }
    /////////////////////////////////////////////////////////////////////////////////////////////

    if (subtestsPassed == 2) {
        std::cout << "\nAll subtests passed successfully!" << std::endl;
        return 1;
    } else {
        std::cout << "\nSubtests passed: " << subtestsPassed << "/2\n" << std::endl;
        return 0;
    }
}

int go1TestFunctions::testNonzeroPosErrorWalk() {
    return 0;
}

int go1TestFunctions::testRaibertHeuristic() {
/*
    This is meant to test the performance of the Raibert Heuristic footstep planner
    in the go1State class. It should be able to generate a set of footstep locations
    based on the current state of the robot.
*/
    mjtNum temp_joint_angles[19] = {0.0, 0.0, 0.27, 1.0, 0.0, 0.0, 0.0, 0.0, 0.9, -1.8, 0.0, 0.9, -1.8, 0.0, 0.9, -1.8, 0.0, 0.9, -1.8};
    mjtNum temp_joint_velocities[18] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Eigen::Vector3d tester_lin_acc (0.0, 0.0, -9.81);

    std::memcpy(tester_joint_angles, temp_joint_angles, sizeof(temp_joint_angles));
    std::memcpy(tester_joint_velocities, temp_joint_velocities, sizeof(temp_joint_velocities));

    std::cout << "################################################\n";
    std::cout << "### Test: Raibert Heuristic footstep planner ###\n";
    std::cout << "################################################\n" << std::endl;
    std::cout << "-- Subtest 1: 0.1 m/s in x-direction --" << std::endl;

    tester_state.resetState();
    tester_state.updateStateFromMujoco(tester_joint_angles, tester_joint_velocities, tester_lin_acc);
    tester_state.root_pos_d << 0.05, 0, 0.27;
    tester_state.root_lin_vel_d << 0.1, 0, 0;
    tester_state.raibertHeuristic();
    
    std::cout << "\nDelta x: " << tester_state.foot_deltaX << " m, Delta y: " << tester_state.foot_deltaY << " m" << std::endl;

    /////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << "\n-- Subtest 2: 0.1 m/s in y-direction --" << std::endl;
    tester_state.resetState();
    tester_state.updateStateFromMujoco(tester_joint_angles, tester_joint_velocities, tester_lin_acc);
    tester_state.root_lin_vel_d << 0, 0.1, 0;
    tester_state.raibertHeuristic();
    
    std::cout << "\nDelta x: " << tester_state.foot_deltaX << " m, Delta y: " << tester_state.foot_deltaY << " m" << std::endl;

    /////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << "\n-- Subtest 3: 0.1 m/s in z-direction --" << std::endl;
    tester_state.resetState();
    tester_state.updateStateFromMujoco(tester_joint_angles, tester_joint_velocities, tester_lin_acc);
    tester_state.root_lin_vel_d << 0, 0, 0.1;
    tester_state.raibertHeuristic();
    
    std::cout << "\nDelta x: " << tester_state.foot_deltaX << " m, Delta y: " << tester_state.foot_deltaY << " m\n" << std::endl;

    /////////////////////////////////////////////////////////////////////////////////////////////

    return 1;

}

int go1TestFunctions::testAmirHLIP() {
/*
    This is meant to test the performance of Amir's HT-LIP footstep planner
    in the go1State class. It should be able to generate a set of footstep locations
    based on the current state of the robot.
*/
    mjtNum temp_joint_angles[19] = {0.0, 0.0, 0.27, 1.0, 0.0, 0.0, 0.0, 0.0, 0.9, -1.8, 0.0, 0.9, -1.8, 0.0, 0.9, -1.8, 0.0, 0.9, -1.8};
    mjtNum temp_joint_velocities[18] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Eigen::Vector3d tester_lin_acc (0.0, 0.0, -9.81);

    std::memcpy(tester_joint_angles, temp_joint_angles, sizeof(temp_joint_angles));
    std::memcpy(tester_joint_velocities, temp_joint_velocities, sizeof(temp_joint_velocities));

    std::cout << "############################################\n";
    std::cout << "### Test: Amir's HT-LIP footstep planner ###\n";
    std::cout << "############################################\n" << std::endl;
    std::cout << "-- Subtest 1: 0.1 m/s in x-direction --" << std::endl;

    tester_state.resetState();
    tester_state.updateStateFromMujoco(tester_joint_angles, tester_joint_velocities, tester_lin_acc);
    tester_state.root_pos_d << 0.05, 0, 0.27;
    tester_state.root_lin_vel_d << 0.1, 0, 0;
    tester_state.amirHLIP();

    std::cout << "\nDelta x: " << tester_state.foot_deltaX << " m, Delta y: " << tester_state.foot_deltaY << " m" << std::endl;

    /////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << "\n-- Subtest 2: 0.1 m/s in y-direction --" << std::endl;
    tester_state.resetState();
    tester_state.updateStateFromMujoco(tester_joint_angles, tester_joint_velocities, tester_lin_acc);
    tester_state.root_lin_vel_d << 0, 0.1, 0;
    tester_state.amirHLIP();

    std::cout << "\nDelta x: " << tester_state.foot_deltaX << " m, Delta y: " << tester_state.foot_deltaY << " m" << std::endl;

    /////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << "\n-- Subtest 3: 0.1 m/s in z-direction --" << std::endl;
    tester_state.resetState();
    tester_state.updateStateFromMujoco(tester_joint_angles, tester_joint_velocities, tester_lin_acc);
    tester_state.root_lin_vel_d << 0, 0, 0.1;
    tester_state.amirHLIP();

    std::cout << "\nDelta x: " << tester_state.foot_deltaX << " m, Delta y: " << tester_state.foot_deltaY << " m\n" << std::endl;

    /////////////////////////////////////////////////////////////////////////////////////////////

    return 1;
}

int go1TestFunctions::testSwingPD() {
/*
    This is meant to test the performance of the swing leg PD controller
    in the go1State class. It should be able to generate swing leg forces
    for swing legs, and provide no output for stance legs.
*/
    mjtNum temp_joint_angles[19] = {0.0, 0.0, 0.27, 1.0, 0.0, 0.0, 0.0, 0.0, 0.9, -1.8, 0.0, 0.9, -1.8, 0.0, 0.9, -1.8, 0.0, 0.9, -1.8};
    mjtNum temp_joint_velocities[18] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Eigen::Vector3d tester_lin_acc (0.0, 0.0, -9.81);

    std::memcpy(tester_joint_angles, temp_joint_angles, sizeof(temp_joint_angles));
    std::memcpy(tester_joint_velocities, temp_joint_velocities, sizeof(temp_joint_velocities));

    std::cout << "##################################\n";
    std::cout << "### Test: Swing leg PD control ###\n";
    std::cout << "##################################\n" << std::endl;
    std::cout << "-- Subtest 1: Swing leg PD for swing leg --" << std::endl;

    tester_state.resetState();
    tester_state.updateStateFromMujoco(tester_joint_angles, tester_joint_velocities, tester_lin_acc);

    tester_state.contacts[1] = false;
    tester_state.foot_pos_d = tester_state.foot_pos;
    tester_state.foot_pos_d.col(1) << 0.1, 0.1, 0.0;

    tester_state.swingPD(1, tester_state.foot_pos_d.col(1), Eigen::Vector3d::Zero());

    std::cout << "\nFL swing leg force: \n" << tester_state.foot_forces_swing.col(1) << std::endl;

    /////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << "\n-- Subtest 2: Swing leg PD for stance leg --\n" << std::endl;
    tester_state.swingPD(0, tester_state.foot_pos_d.col(0), Eigen::Vector3d::Zero());

    return 1;
}

int go1TestFunctions::testBezierPos() {
/*
    This is meant to test the performance of the positional Bezier curve based
    on the current swing_phase and the foot_deltaX and foot_deltaY values
    generated by the Raibert Heuristic or Amir's HT-LIP footstep planner.
*/
    std::cout << "###################################\n";
    std::cout << "### Test: Bezier position curve ###\n";
    std::cout << "###################################\n" << std::endl;

    tester_state.resetState();
    tester_state.foot_deltaX = 0.1;
    tester_state.foot_deltaY = 0.2;
    tester_state.swing_phase = 30;

    Eigen::Vector3d bezierPos = tester_state.bezierPos();

    std::cout << "\nBezier curve position: \n" << bezierPos << "\n" << std::endl;

    return 1;
}

int go1TestFunctions::testBezierVel() {
/*
    This is meant to test the performance of the velocity Bezier curve based
    on the current swing_phase and the foot_deltaX and foot_deltaY values
    generated by the Raibert Heuristic or Amir's HT-LIP footstep planner.
*/
    std::cout << "###################################\n";
    std::cout << "### Test: Bezier velocity curve ###\n";
    std::cout << "###################################\n" << std::endl;

    tester_state.resetState();
    tester_state.foot_deltaX = 0.1;
    tester_state.foot_deltaY = 0.2;
    tester_state.swing_phase = 30;

    Eigen::Vector3d bezierVel = tester_state.bezierVel();

    std::cout << "\nBezier curve velocity: \n" << bezierVel << "\n" << std::endl;

    return 1;
}

int go1TestFunctions::testNumericJacobian() {
    auto start = std::chrono::high_resolution_clock::now();

    Eigen::VectorXd x(22);
    x.setZero();
    Eigen::Vector3d f_meas(0, 0, 9.8);
    Eigen::Vector3d omg_meas(1, 0, 0.5);

    Eigen::MatrixXd jacobian = numericalJacobian(fState, x, 1e-6, true, f_meas, omg_meas);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end - start;
    std::cout << "Numerical Jacobian df/dx:\n" << jacobian;
    std::cout << "\nCalculation time: " << duration.count() << " ms" << std::endl;

    Eigen::MatrixXd h_jac = numericalJacobian(hState, x);

    auto end2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration2 = end2 - end;
    std::cout << "Numerical Jacobian dh/dx:\n" << h_jac;
    std::cout << "\nCalculation time: " << duration2.count() << " ms" << std::endl;

    return 1;
}