/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>

using namespace std;
using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
  Custom(uint8_t level) : safe(LeggedType::Go1),
                          udp(level, 8090, "192.168.123.10", 8007)
  {
    udp.InitCmdData(cmd);
  }
  void UDPRecv();
  void RobotControl();

  Safety safe;
  UDP udp;
  LowCmd cmd = {0};
  LowState state = {0};
  float dt = 0.002; // 0.001~0.01
};

void Custom::UDPRecv()
{
  udp.Recv();
}

void Custom::RobotControl()
{
  udp.GetRecv(state);
  printf("FR_0: %f, FR_1: %f, FR_2: %f\n", state.motorState[FR_0].q, state.motorState[FR_1].q, state.motorState[FR_2].q);
  printf("FL_0: %f, FL_1: %f, FL_2: %f\n", state.motorState[FL_0].q, state.motorState[FL_1].q, state.motorState[FL_2].q);
  printf("RR_0: %f, RR_1: %f, RR_2: %f\n", state.motorState[RR_0].q, state.motorState[RR_1].q, state.motorState[RR_2].q);
  printf("RL_0: %f, RL_1: %f, RL_2: %f\n", state.motorState[RL_0].q, state.motorState[RL_1].q, state.motorState[RL_2].q);
  printf("ACC_X: %f, ACC_Y: %f, ACC_Z: %f\n", state.imu.accelerometer[0], state.imu.accelerometer[1], state.imu.accelerometer[2]);
  printf("GYR_X: %f, GRY_Y: %f, GYR_Z: %f\n", state.imu.gyroscope[0], state.imu.gyroscope[1], state.imu.gyroscope[2]);
  printf("FR_atm: %u, FL_atm: %u, RR_atm: %u, RL_atm: %u\n", state.footForce[0], state.footForce[1], state.footForce[2], state.footForce[3]);

  std::cout << "###############################################" << std::endl
            << "Output break afer one timestamp for all motors." << std::endl
            << "###############################################" << std::endl;

}

int main(void)
{
  std::cout << "Communication level is set to LOW-level." << std::endl
            << "WARNING: Make sure the robot is hung up." << std::endl
            << "NOTE: The robot also needs to be set to LOW-level mode, otherwise it will make strange noises and this example will not run successfully! " << std::endl
            << "Press Enter to continue..." << std::endl;
  std::cin.ignore();

  Custom custom(LOWLEVEL);
  // InitEnvironment();
  LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom));
  LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv, &custom));

  loop_udpRecv.start();
  loop_control.start();

  while (1)
  {
    sleep(10);
  };

  return 0;
}
