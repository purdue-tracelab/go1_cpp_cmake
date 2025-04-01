/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>

// Package-specific header files
#include "go1_cpp_cmake/go1StanceMPC.h"

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
  int motiontime = 0;
  float dt = 0.002; // 0.001~0.
  go1State hardware_go1_state;

};

void Custom::UDPRecv() {
  udp.Recv();
}

void Custom::UDPSend() {
  udp.Send();
}

void Custom::RobotControl() {
  motiontime++;
  udp.GetRecv(state);

  if (hardware_go1_state.walking_mode) {
    //get the desired state of the robot for x
    //run with velocity and time
    hardware_go1_state.root_lin_vel_d << 0.2, 0, 0;
    hardware_go1_state.root_pos_d << hardware_go1_state.root_pos.x() + hardware_go1_state.root_lin_vel_d.x() * (SWING_PHASE_MAX + 1) * DT_CTRL, 0, WALK_HEIGHT;
    
    //get desired state of the robot for y
    //run with velocity and time
    //hardware_go1_state.root_lin_vel_d << 0, 0.1, 0;
    //hardware_go1_state.root_pos_d << hardware_go1_state.root_pos.y() + hardware_go1_state.root_lin_vel_d.y() * (SWING_PHASE_MAX + 1) * DT_CTRL, 0, WALK_HEIGHT;


    //get the desired state of the robot for xy
    //run with velocity and time
 
  } else {
    //stand. **Add this**
  }

  //move to position for x, y, or xy

  //get the joint position

  //update go1 state

  //update mpc controller

  //convert foot forces to torques


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
