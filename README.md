
# Overview
`go1_cpp_cmake` is a CMake project created for preserving our lab's research done on footstep planning in noninertial environments surfaces using quadrupedal legged locomotion, and serves as a clear repo for comparing the effects of different footstep planners and state estimators on the stability of locomotion atop these dynamic rigid surfaces (DRS). The footstep planners used/implemented are the Raibert Heuristic (RH), the Capture Point augmentation of the Raibert Heuristic(RH+CP), and our lab's new Hybrid Time-varying Linear-Inverted Pendulum (HT-LIP) planner. The state estimators implemented are the contact-aided Kalman filter (CAKF) proposed by MIT, and the contact-aided extended Kalman filter proposed by ETH Zurich (CAEKF). For control, the stance leg controller uses model predictive control (MPC) using quadratic programming (QP) for optimzation, and the swing leg controller uses joint-level proportional-derivative (PD) control. The code constructs a full-dimensional torque-control framework for ensuring stable locomotion for quadrupedal robots in simulation and hardware. 

![Framework Diagram](/models/assets/framework_diag.png?raw=true "Framework Diagram")

For a simplified and detailed explanation of these modules, look at [this Overleaf link](https://www.overleaf.com/read/rswbkdgmngpz#2c8aa1) to know more, or take a look at my thesis [here](https://hammer.purdue.edu/articles/thesis/_b_Evaluation_and_Improvement_of_a_Hybrid_Time-Varying_Linear_Inverted_Pendulum_Model_Footstep_Planner_for_Quadrupedal_Locomotion_on_Dynamic_Rigid_Surfaces_b_/29895506?file=57137330) for a longer look!


# CMake Project Installation Guide

This guide covers the installation of dependencies required for building and running the CMake project `go1_cpp_cmake`. The package is intended for use on Ubuntu 22.04 and integrates with MuJoCo 3.2.7 for simulation, Eigen for efficient matrix representation, and OSQP for QP-MPC optimization. By the end of the installation, you should have these packages/libraries properly configured and installed:

- **MuJoCo 3.2.7 (newest version at the moment)** (Simulation)
- **Eigen3** (Header-only library for matrix representation and operations)
- **OSQP v0.6.3** (QP Solver for GRF MPC)
- **OsqpEigen v0.7.0** (C++ wrapper for OSQP w/ Eigen linear algebra library interface)

Note that this is for installing these packages **system wide**, so if you need to make separate workspaces for this, consider Docker or a virtual machine to make sure dependencies don't clash across projects. Also, this is a C++ project; I did not implement a Python version, and I'm sure it's possible if you'd like for simulation purposes, but I can't guarantee that Python can run on your hardware as fast or efficiently as C++ can. Lastly, if you want to upgrade to Ubuntu 24 or newer or on another OS, you'll have to determine the dependencies yourself, as a lot of extra steps (installing Java 8/11, upgrading Python 3.6 to higher, installing LCM, etc.) were skipped or invalidated with Ubuntu 22, so general caution is advised.

## Pre-Installation Step: Add Self to Sudo Group
Naturally, you can't really install or change anything on your system without being part of the administrator, or `sudo` group. Running any of the following terminal commands is going to fail since you're not automatically given `sudo` status when you create the Ubuntu 22 system/workspace, so follow these steps if you're not yet part of the `sudo` group.

To login as root, run:

```bash
su -  # when prompted for the password, enter the password you made for your VM
```

Once logged in as root, you need to add yourself to the sudo group. For this, you have two options:

```bash
# Option 1
sudo usermod -aG sudo your_username  # replace your_username with the username you set for the vm
su - your_username  # run this to save your changes
sudo whoami  # should return "root"

# Option 2
visudo  # will replace your current terminal with a text editor
# Find a line that says root ALL=(ALL) ALL
# Below it, re-write it, but replace the word "root" with your username
# Type ctrl+X, then Y, then hit enter to save it to the suggested file location
```

Once you've made the edits:

```bash
exit  # run this twice for option 1, once for option 2
# Close the terminal and open a new terminal
sudo whoami  # should return "root"
```
This should ensure that you can run all the commands with administrator access from now on. Note that you'll have to enter your user password each time you use a `sudo` command in a new terminal for the first time.

## Step 1: Install MuJoCo 3.2.7

1. Download MuJoCo 3.2.7 from the [official MuJoCo website](https://mujoco.org/) by hitting the **Download** tab.
2. Create a file directory for MuJoCo and extract the tar ball contents into it

```bash
mkdir -p ~/mujoco && cd ~/mujoco
wget https://github.com/deepmind/mujoco/releases/latest/download/mujoco-3.2.7-linux-x86_64.tar.gz
# Double-check the above link, may have changed due to updates by Google
tar -xvzf mujoco-3.2.7-linux-x86_64.tar.gz
sudo mv mujoco-3.2.7 /usr/local/mujoco
export MUJOCO_PATH=/usr/local/mujoco
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/mujoco/lib' >> ~/.bashrc
source ~/.bashrc
```

3. Download the dependencies for MuJoCo visualization.

```bash
sudo apt-get install libglfw3-dev libglew-dev libosmesa6-dev
```

This should enable you to now simulate XML, URDF, and MJCF models in MuJoCo. To test it works:

```bash
cd /usr/local/mujoco/bin
./simulate ./../model/humanoid/humanoid.xml
```

If a simulation window pops up and a human falls over, then you're able to run MuJoCo successfully! Note that 3.2.7 was the latest stable release at the time of making this repo, so if you want to use newer versions, double-check it won't clash with Python dependencies for your system. If it will, ***please*** don't replace your Python installation. You will break your Linux terminal and OS, and have to factor the drive/VM, so ***please please please*** be careful about this (AJ and I did this like three times before realizing what the issue was, and had to reset our VMs).

## Step 2: Install Eigen3

Eigen is a header-only library that provides matrix manipulation functionality. Install Eigen3 with:

```bash
sudo apt-get install libeigen3-dev
```

## Step 3: Install OSQP Version 0.6.3

To install OSQP version 0.6.3 specifically, follow these steps:

1. Clone the OSQP repository at the desired version:

```bash
cd ~
git clone --branch v0.6.3 https://github.com/oxfordcontrol/osqp.git
cd osqp
git submodule update --init --recursive
```

2. Build and install OSQP:

```bash
mkdir build
cd build
cmake ..
make
sudo make install
```

## Step 4: Install OsqpEigen Version 0.7.0

To install OsqpEigen version 0.7.0, follow these steps:

1. Clone the OsqpEigen repository at the desired version:

```bash
cd ~
git clone --branch v0.7.0 https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
```

2. Build and install OsqpEigen:

```bash
mkdir build
cd build
cmake ..
make
sudo make install
```

Note that the versions of `OSQP` and `OsqpEigen` are specific to each other, so if you do want to upgrade this package or use it on a newer system, make sure your versions are compatible with each other and your system requirements.

## Step 5: Install the Package

Once all dependencies are installed, clone the `go1_cpp_cmake` project onto your home directory, then build.

1. **Clone the repo**:

```bash
git clone https://github.com/purdue-tracelab/go1_cpp_cmake.git
# requires a GitHub SSH key for cloning this repo, look up the tutorial on GitHub
```

2. **Build the project**:

```bash
cd go1_cpp_cmake
mkdir build && cd build
cmake ..
make
```

3. **Run the project executables**:

```bash
ls # shows whatever files are in the current directory, including executables
./run_go1Tests  # runs a basic MPC + swing PD calculation
./example_position  # runs Unitree's example hardware code
```

# Post-Installation
## Running the Package

Once successfully built, you can run my simulation executables for the project:

- `run_go1Tests` - checks if the calculation scripts are working properly
- `run_go1InertiaCalc` - calculates the nominal inertia of Go1
- `run_go1MujocoSim` - runs the Go1 MuJoCo sim for walking
- `run_go1FSMMujoco` - runs the FSM version of Go1 MuJoCo sim (only walks in place)
- `run_go1FSMMujocoLoopFunc` - runs the above but with directional movement and with enforced 500 Hz loop time
- `run_go1ExperimentMujocoSim` - runs the DRS experiment for Go1

You can also run Unitree's example code and my hardware executables:

- `example_position` - moves the FR foot of Go1 while printing the thigh's joint position and velocity
- `example_velocity` - moves the FR foot of Go1 with a time-varying velocity
- `example_torque` - applies a constant torque to the FR foot of Go1
- `example_walk` - executes high-level walking example on Go1
- `example_joystick` - prints out joystick information received by Go1
- `test_go1ReadHardware` - tests reading Go1's low-level information
- `test_go1TorqueHardware` - tests sending Go1 torque commands (careful with this one, change values if necessary)
- `test_go1JointPDHardware` - tests the Startup/Shutdown sequences of Go1
- `run_go1FSMHardware` - runs the FSM code from simulation onto Go1 hardware

All of these scripts can be run by entering the build folder of the project, and typing `./` before any executable file's name. You will know what files are executable if you type `ls` in terminal and their names are highlighted in a different color from the rest of the files.

```bash
cd ~/go1_cpp_cmake
cd build
ls
./* # replace the asterisk with any executable listed above
```

Lastly, after running any of the simulation or hardware executables, you can plot some of the internal data using the Python files from the `py_src` directory. 

- `plotGo1CalcTime.py` - records the calculation time for each module you record, look in `runGo1Mujoco.cpp`
- `plotGo1EstResidualData.py` - records the residuals for the filters, important for tuning state estimators
- `plotGo1MujocoData.py` - plots the desired, actual, and estimated values of Go1 in simulation
- `plotGo1HardwareData.py` - plots the internal desired and estimated values of Go1 on hardware

I usually type:

```bash
cd ~/go1_cpp_cmake
./py_src/plotGo1* # replace the asterisk with any Python executables above
```

when plotting anything from the CSV files saved from the simulations in the `data` directory. You can format this yourself as well, just make sure the CSV you generate and the file name matches what the Python function is looking for.

## Changing/Improving the Project

The goal of this code is to reimplement earlier research from our lab, but in a clearer and modular fashion for training newer lab mates. There's a lot to understand about robotics before even touching code, and it can be overwhelming, so I hope this can help you get past the initial hump faster and more gracefully than I did. If you want to adjust anything in the code (change FSM structure, controllers, planners, motion generators, etc.), then you should first understand how each file and function interacts with each other in the simulation executables, since the controllers for simulation are the exact same ones used for hardware. I recommend looking at `runGo1FSMMujoco.cpp` for the general control structure, `go1FSM.cpp` for the controller swapping behavior, and `go1Params.h` to see what settings are enabled, and then you can track the files through your IDE (I used VS Code and Ctrl+Click to jump between files) and see what is necessary, which is harder with ROS code unless you're familiar. To create new hardware code, you'll have to add these three lines to the CMakeLists.txt when you make a new file:

```bash
add_executable(example_* example/example_*.cpp)
target_link_libraries(example_* ${EXTRA_LIBS})
target_link_options(example_* PRIVATE -no-pie)  # this is a workaround for the moment
```

You also need to add the name of the executable in the `install(TARGETS)` section below, but that should be all the changes you need to make. To build the project so you can run the executables, you will always run these lines in the build folder of the project:

```bash
cd ~go1_cpp_cmake/build  # if you're not already there
cmake ..
make
```

This way, you'll always catch pre-build issues on the spot. If those pass and you still get errors, you likely mishandled information inside the script instead of leaving syntax errors. Follow the existing formats, use ChatGPT, or ask me to make sure you understand how to make those changes, if you so desire. I'm going to move on to implementing a personal repo for legged and manipulator robots in general, since I'd like to explore reinforcement learning and other model-based techniques to improve my understanding and skills for the job.

Finally, my code in here is not perfect. There are parts that aren't ideal (mostly the state estimator tunings) and parts that aren't finished (terrain adaptation, more robust MPC to use early contacts for stance control) that can be implemented and perfected by you! If there's anything I'd like you to learn from this, it's how to properly manage a repo with a ton of moving parts and make it clear to debug for yourself and others in the future. By all means, make code that looks and works better than this, so you can save everybody's time and energy in the future, including yours. Otherwise, thanks for reading! Big shoutout to Prof. Gu, Zijian, I-Chia, Zenan, Wenxi, Muqun, Falak, AJ, Katy, Leo, Annalise, Andy, and you.

## Troubleshooting

1. **MuJoCo Header Not Found**: If you receive an error regarding MuJoCo headers not being found, ensure that the path to the MuJoCo installation is correctly specified in your `CMakeLists.txt`.

2. **Library Linking Errors**: If you encounter issues linking OSQP or OsqpEigen, verify that the libraries are properly installed, and the paths are correctly set in the `CMakeLists.txt` file.

---

Feel free to contact me if you have any issues with the installation, or if you wanna flex your code working.