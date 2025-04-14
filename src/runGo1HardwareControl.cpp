/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>

// Package-specific header files
#include "go1_cpp_cmake/go1StanceMPC.h"
#include "go1_cpp_cmake/go1StateEstimator.h"

using namespace UNITREE_LEGGED_SDK;

class Custom {
public:
  Custom(uint8_t level) : safe(LeggedType::Go1),
                          udp(level, 8090, "192.168.123.10", 8007)
  {
    udp.InitCmdData(cmd);
  }
  void UDPSend();
  void UDPRecv();
  void RobotControl(); // where all of our code will go, maybe split into swing PD and stance MPC for different loop rates

  Safety safe;
  UDP udp;
  LowCmd cmd = {0};
  LowState state = {0};
  float motiontime = 0;
  float dt = 0.002; // 0.001~0.
  go1State hardware_go1_state;
  go1StateEstimator hardware_go1_estimator;
  float lastMPCUpdateTime = 0.0;
  float mpcInterval = 0.02;

};

void Custom::UDPRecv() {
  udp.Recv();
}

void Custom::UDPSend() {
  udp.Send();
}

void Custom::RobotControl() {
  motiontime = dt + motiontime; // swap order with udp.GetRecv(state);
  udp.GetRecv(state);

  if (hardware_go1_state.walking_mode) {
    //get the desired state of the robot for x
    //run with velocity and time
    hardware_go1_state.root_lin_vel_d << 0.2, 0, 0;
    hardware_go1_state.root_pos_d << hardware_go1_state.root_pos.x() + hardware_go1_state.root_lin_vel_d.x() * (SWING_PHASE_MAX + 1) * DT_CTRL, 0, WALK_HEIGHT; // add buffer to eliminate drift
    
    //get desired state of the robot for y
    //run with velocity and time
    //hardware_go1_state.root_lin_vel_d << 0, 0.1, 0;
    //hardware_go1_state.root_pos_d << hardware_go1_state.root_pos.y() + hardware_go1_state.root_lin_vel_d.y() * (SWING_PHASE_MAX + 1) * DT_CTRL, 0, WALK_HEIGHT;


    //get the desired state of the robot for xy
    //run with velocity and time
 
  } else {
    //stand. **Add this**
    hardware_go1_state.root_lin_vel_d << 0, 0, 0;
    hardware_go1_state.root_pos_d << hardware_go1_state.root_pos; // add buffer to eliminate drift
  }

  //get the joint position
  // fix what the variables are equal to
  Eigen::Vector3d lin_acc = Eigen::Map<const Eigen::Vector3d>(data->cacc + 3 * base_id);
  Eigen::Vector3d lin_acc_meas = Eigen::Map<const Eigen::Vector3d>(data->sensordata + sensor_adr);

  //update go1 state
  hardware_go1_state.updateStateFromMujoco(qpos_estimator, qvel_estimator, lin_acc);
  // need to change for hardware
  // hardware_go1_estimator.simpleKalmanFilterMujoco(mujoco_go1_state, lin_acc_meas);

  //update mpc controller
  if (motiontime - lastMPCUpdateTime >= mpcInterval) {
    // findMPCSolution();
    go1StanceMPC(hardware_go1_state);
    lastMPCUpdateTime = motiontime;
  }

  //convert foot forces to torques
  hardware_go1_state.convertForcesToTorquesMujoco(qpos_estimator);
  joint_torques_stacked << hardware_go1_state.joint_torques.block<3, 1>(0, 0), // FR
                          hardware_go1_statejoint_torques.block<3, 1>(0, 1), // FL
                          hardware_go1_state.joint_torques.block<3, 1>(0, 2), // RR
                          hardware_go1_state.joint_torques.block<3, 1>(0, 3); // RL

  std::vector<int> legIndices = {FR_0, FR_1, FR_2, FL_0, FL_1, FL_2, RR_0, RR_1, RR_2, RL_0, RL_1, RL_2}; // leg indices for easier access
  for (int j = 0; j < model->nu; j++) {
    cmd.motorCmd[legIndices[j]].q = PosStopF;
    cmd.motorCmd[legIndices[j]].dq = VelStopF;
    cmd.motorCmd[legIndices[j]].Kp = 0;
    cmd.motorCmd[legIndices[j]].Kd = 0;
    cmd.motorCmd[legIndices[j]].tau = joint_torques_stacked(j, 0);
  }

  udp.SetSend(cmd);
}

int main(void) {
  std::cout << "Communication level is set to LOW-level." << std::endl
            << "WARNING: Make sure the robot is hung up." << std::endl
            << "NOTE: The robot also needs to be set to LOW-level mode, otherwise it will make strange noises and this example will not run successfully! " << std::endl
            << "Press Enter to continue..." << std::endl;
  std::cin.ignore();

  Custom custom(LOWLEVEL);
  LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom));
  LoopFunc loop_udpSend("udp_send", custom.dt, 3, boost::bind(&Custom::UDPSend, &custom));
  LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv, &custom));

  loop_udpSend.start();
  loop_udpRecv.start();
  loop_control.start();

  while (1)
  {
    sleep(10);
  };

  return 0;
}
