/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <ncurses.h>

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
  void UDPSend();
  void RobotControl();
  void keyControl(); //added

  Safety safe;
  UDP udp;
  LowCmd cmd = {0};
  LowState state = {0};
  float qInit[12] = {0};
  float qDes[12] = {0};
  float Kp[12] = {0};
  float Kd[12] = {0};
  double time_consume = 0;
  int rate_count = 0;
  int motiontime = 0;
  float dt = 0.002; // 0.001~0.01
  bool squat = false;

};

void Custom::UDPRecv()
{
  udp.Recv();
}

void Custom::UDPSend()
{
  udp.Send();
}

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
  double p;
  rate = std::min(std::max(rate, 0.0), 1.0);
  p = initPos * (1 - rate) + targetPos * rate;
  return p;
}

void Custom::RobotControl()
{
  motiontime++;
  udp.GetRecv(state);
  printf("%d  %b\n", motiontime, squat);

  // gravity compensation
  cmd.motorCmd[FR_0].tau = -0.65f;
  cmd.motorCmd[FL_0].tau = +0.65f;
  cmd.motorCmd[RR_0].tau = -0.65f;
  cmd.motorCmd[RL_0].tau = +0.65f;

  // if( motiontime >= 100){
  if (motiontime >= 0)
  {
    // first, get record initial position
    // if( motiontime >= 100 && motiontime < 500){

    //-0.3 1.21 -2.818 0.3 1.21 -2.818 -0.3 1.21 -2.818 0.3 1.21 -2.818
    //FR FL RR RL
    if (motiontime >= 0 && motiontime < 10)
    {
      qInit[0] = -0.3; //state.motorState[FR_0].q;
      qInit[1] = 1.21; //state.motorState[FR_1].q;
      qInit[2] = -2.818; //state.motorState[FR_2].q;

      //Add all legs
      qInit[3] = 0.3; //state.motorState[FL_0].q;
      qInit[4] = 1.21; //state.motorState[FL_1].q;
      qInit[5] = -2.818; //state.motorState[FL_2].q;
      qInit[6] = -0.3; //state.motorState[RR_0].q;
      qInit[7] = 1.21; //state.motorState[RR_1].q;
      qInit[8] = -2.818; //state.motorState[RR_2].q;
      qInit[9] = 0.3; //state.motorState[RL_0].q;
      qInit[10] = 1.21; //state.motorState[RL_1].q;
      qInit[11] = -2.818; //state.motorState[RL_2].q;

    }
    // second, move to the origin point of a sine movement with Kp Kd
    // if( motiontime >= 500 && motiontime < 1500){
    if (motiontime >= 10 && motiontime < 1000)
    {
      rate_count++;
      double rate = rate_count / 1000.0; // needs count to 200
      Kp[0] = 60.0;
      Kp[1] = 60.0;
      Kp[2] = 60.0;
      Kd[0] = 3.0;
      Kd[1] = 3.0;
      Kd[2] = 3.0;

      //Add all legs
      Kp[3] = 60.0;
      Kp[4] = 60.0;
      Kp[5] = 60.0;
      Kd[3] = 3.0;
      Kd[4] = 3.0;
      Kd[5] = 3.0;

      Kp[6] = 60.0;
      Kp[7] = 60.0;
      Kp[8] = 60.0;
      Kd[6] = 3.0;
      Kd[7] = 3.0;
      Kd[8] = 3.0;

      Kp[9] = 60.0;
      Kp[10] = 60.0;
      Kp[11] = 60.0;
      Kd[9] = 3.0;
      Kd[10] = 3.0;
      Kd[11] = 3.0;


      // Kp[0] = 20.0; Kp[1] = 20.0; Kp[2] = 20.0;
      // Kd[0] = 2.0; Kd[1] = 2.0; Kd[2] = 2.0;

      //0 0.9 -1.8 0 0.9 -1.8 0 0.9 -1.8 0 0.9 -1.8

      qDes[0] = jointLinearInterpolation(qInit[0], 0, rate);
      qDes[1] = jointLinearInterpolation(qInit[1], 0.9, rate);
      qDes[2] = jointLinearInterpolation(qInit[2], -1.8, rate);

      //Add all legs
      qDes[3] = jointLinearInterpolation(qInit[3], 0, rate);
      qDes[4] = jointLinearInterpolation(qInit[4], 0.9, rate);
      qDes[5] = jointLinearInterpolation(qInit[5], -1.8, rate);
      qDes[6] = jointLinearInterpolation(qInit[6], 0, rate);
      qDes[7] = jointLinearInterpolation(qInit[7], 0.9, rate);
      qDes[8] = jointLinearInterpolation(qInit[8], -1.8, rate);
      qDes[9] = jointLinearInterpolation(qInit[9], 0, rate);
      qDes[10] = jointLinearInterpolation(qInit[10], 0.9, rate);
      qDes[11] = jointLinearInterpolation(qInit[11], -1.8, rate);
    }
    else if (motiontime >= 1000)
    {     
      qDes[0] = 0;
      qDes[1] = 0.9;
      qDes[2] = -1.8;

      //Add all legs
      qDes[3] = 0;
      qDes[4] = 0.9;
      qDes[5] = -1.8;
      qDes[6] = 0;
      qDes[7] = 0.9;
      qDes[8] = -1.8;
      qDes[9] = 0;
      qDes[10] = 0.9;
      qDes[11] = -1.8;
    }

    else if (motiontime >= 2000 && squat == true)
    {    
      double rate = (rate_count-2000.0) / 1000.0;
      qDes[0] = jointLinearInterpolation(0, qInit[0], rate);
      qDes[1] = jointLinearInterpolation(0.9, qInit[1], rate);
      qDes[2] = jointLinearInterpolation(-1.8, qInit[2], rate);

      //Add all legs
      qDes[3] = jointLinearInterpolation(0, qInit[3], rate);
      qDes[4] = jointLinearInterpolation(0.9, qInit[4], rate);
      qDes[5] = jointLinearInterpolation(-1.8, qInit[5], rate);
      qDes[6] = jointLinearInterpolation(0, qInit[6], rate);
      qDes[7] = jointLinearInterpolation(0.9, qInit[7], rate);
      qDes[8] = jointLinearInterpolation(-1.8, qInit[8], rate);
      qDes[9] = jointLinearInterpolation(0, qInit[9], rate);
      qDes[10] = jointLinearInterpolation(0.9, qInit[10], rate);
      qDes[11] = jointLinearInterpolation(-1.8, qInit[11], rate);
    }

    //FR
    cmd.motorCmd[FR_0].q = qDes[0];
    cmd.motorCmd[FR_0].dq = 0;
    cmd.motorCmd[FR_0].Kp = Kp[0];
    cmd.motorCmd[FR_0].Kd = Kd[0];
    cmd.motorCmd[FR_0].tau = 0.0f;

    cmd.motorCmd[FR_1].q = qDes[1];
    cmd.motorCmd[FR_1].dq = 0;
    cmd.motorCmd[FR_1].Kp = Kp[1];
    cmd.motorCmd[FR_1].Kd = Kd[1];
    cmd.motorCmd[FR_1].tau = 0.0f;

    cmd.motorCmd[FR_2].q = qDes[2];
    cmd.motorCmd[FR_2].dq = 0;
    cmd.motorCmd[FR_2].Kp = Kp[2];
    cmd.motorCmd[FR_2].Kd = Kd[2];
    cmd.motorCmd[FR_2].tau = 0.0f;


    //FL
    cmd.motorCmd[FL_0].q = qDes[0];
    cmd.motorCmd[FL_0].dq = 0;
    cmd.motorCmd[FL_0].Kp = Kp[0];
    cmd.motorCmd[FL_0].Kd = Kd[0];
    cmd.motorCmd[FL_0].tau = 0.0f;

    cmd.motorCmd[FL_1].q = qDes[1];
    cmd.motorCmd[FL_1].dq = 0;
    cmd.motorCmd[FL_1].Kp = Kp[1];
    cmd.motorCmd[FL_1].Kd = Kd[1];
    cmd.motorCmd[FL_1].tau = 0.0f;

    cmd.motorCmd[FL_2].q = qDes[2];
    cmd.motorCmd[FL_2].dq = 0;
    cmd.motorCmd[FL_2].Kp = Kp[2];
    cmd.motorCmd[FL_2].Kd = Kd[2];
    cmd.motorCmd[FL_2].tau = 0.0f;

        //RR
    cmd.motorCmd[RR_0].q = qDes[0];
    cmd.motorCmd[RR_0].dq = 0;
    cmd.motorCmd[RR_0].Kp = Kp[0];
    cmd.motorCmd[RR_0].Kd = Kd[0];
    cmd.motorCmd[RR_0].tau = 0.0f;

    cmd.motorCmd[RR_1].q = qDes[1];
    cmd.motorCmd[RR_1].dq = 0;
    cmd.motorCmd[RR_1].Kp = Kp[1];
    cmd.motorCmd[RR_1].Kd = Kd[1];
    cmd.motorCmd[RR_1].tau = 0.0f;

    cmd.motorCmd[RR_2].q = qDes[2];
    cmd.motorCmd[RR_2].dq = 0;
    cmd.motorCmd[RR_2].Kp = Kp[2];
    cmd.motorCmd[RR_2].Kd = Kd[2];
    cmd.motorCmd[RR_2].tau = 0.0f;

    //RL
    cmd.motorCmd[RL_0].q = qDes[0];
    cmd.motorCmd[RL_0].dq = 0;
    cmd.motorCmd[RL_0].Kp = Kp[0];
    cmd.motorCmd[RL_0].Kd = Kd[0];
    cmd.motorCmd[RL_0].tau = 0.0f;

    cmd.motorCmd[RL_1].q = qDes[1];
    cmd.motorCmd[RL_1].dq = 0;
    cmd.motorCmd[RL_1].Kp = Kp[1];
    cmd.motorCmd[RL_1].Kd = Kd[1];
    cmd.motorCmd[RL_1].tau = 0.0f;

    cmd.motorCmd[RL_2].q = qDes[2];
    cmd.motorCmd[RL_2].dq = 0;
    cmd.motorCmd[RL_2].Kp = Kp[2];
    cmd.motorCmd[RL_2].Kd = Kd[2];
    cmd.motorCmd[RL_2].tau = 0.0f;
  }

  if (motiontime > 10)
  {
    safe.PositionLimit(cmd);
    int res1 = safe.PowerProtect(cmd, state, 1);
    // You can uncomment it for position protection
    // int res2 = safe.PositionProtect(cmd, state, 10);
    if (res1 < 0)
      exit(-1);
  }

  udp.SetSend(cmd);
}


//Lowers to squat when p is pressed  
void Custom::keyControl()
{ 
// Initialize ncurses
initscr(); // Start ncurses mode
timeout(0); // Non-blocking input, no delay
nodelay(stdscr, TRUE);
noecho();            // Don't display pressed keys
cbreak();            // Disable line buffering (don't need to press Enter)

  int ch = getch();

  if (ch != ERR) {
      switch (ch) {
          case 'p':  // Move forwards
              squat = true;
              break;
      }
    }
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
  LoopFunc loop_udpSend("udp_send", custom.dt, 3, boost::bind(&Custom::UDPSend, &custom));
  LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv, &custom));

  loop_udpSend.start();
  loop_udpRecv.start();
  loop_control.start();
  

  while (1)
  {
    sleep(10);
    Custom keyControl();
  };

  return 0;
}
