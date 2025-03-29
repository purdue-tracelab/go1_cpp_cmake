/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <set>
#include <type_traits>

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

void Custom::UDPSend()
{
  udp.Send();
}

std::vector<std::vector<std::string>> torqueFile(std::string fileName) {
    std::ifstream inputFile(fileName);
    std::vector<std::vector<std::string>> matrix;
    std::string line;
    std::set<int> selectedColumns = {108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119};
    
    if (!inputFile.is_open()) {
        std::cerr << "Unable to open file." << std::endl;
        matrix.clear();
        return matrix;
    }
    
    // Read the file line by line
    while (std::getline(inputFile, line)) {
        std::vector<std::string> row;
        std::stringstream ss(line);
        std::string cell;
        int colIndex = 0;
        
        // Split the line by commas
        while (std::getline(ss, cell, ',')) {
            // Only add the cell if it's one of the desired columns
            if (selectedColumns.find(colIndex) != selectedColumns.end()) {
                row.push_back(cell);
            }
            ++colIndex;
        }
        
        matrix.push_back(row);
        
    }
    return matrix;
}

void Custom::RobotControl()
{
  udp.GetRecv(state);
  udp.SetSend(cmd);
}

int main(void)
{
  std::cout << "Communication level is set to LOW-level." << std::endl
            << "WARNING: Make sure the robot is hung up." << std::endl
            << "NOTE: The robot also needs to be set to LOW-level mode, otherwise it will make strange noises and this example will not run successfully! " << std::endl
            << "Press Enter to continue..." << std::endl;
  std::cin.ignore();

//   Custom custom(LOWLEVEL);

//   LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom));
//   LoopFunc loop_udpSend("udp_send", custom.dt, 3, boost::bind(&Custom::UDPSend, &custom));
//   LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv, &custom));

//   loop_udpSend.start();
//   loop_udpRecv.start();
//   loop_control.start();

//   while (1)
//   {
//     sleep(10);
//   };

    std::vector<std::vector<std::string>> matrix = torqueFile("../data/go1_mujoco_data.csv");
    for(int i = 0; i < matrix.size(); i++) {
        for(int j = 0; j < matrix[i].size(); j++) {
            std::cout << matrix[i][j] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << typeid(matrix[2][2]).name() << std::endl;

  return 0;
}
