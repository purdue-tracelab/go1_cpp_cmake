
# CMake Project Installation Guide

This guide covers the installation of dependencies required for building and running the CMake project `go1_cpp_cmake`. The package is intended for use on Ubuntu 22.04 and integrates with MuJoCo 3.2.7 for simulation and OSQP for quadratic programming (QP) used in Model Predictive Control (MPC) formulations. Make sure you go to [this Overleaf link](https://www.overleaf.com/read/rswbkdgmngpz#2c8aa1) to take a look at the overall project formulation and associated goals.

By the end of the installation, you should have these packages/libraries properly configured and installed:

- **MuJoCo 3.2.7 (newest version at the moment)** (Simulation)
- **OSQP v0.6.3** (QP Solver for GRF MPC)
- **OsqpEigen v0.7.0** (C++ wrapper for OSQP w/ Eigen linear algebra library interface)

## Pre-Installation Step: Add Self to Sudo Group
Naturally, you can't really run any of these installation commands without being part of the administrator, or `sudo` group. Running any of the following terminal commands is going to fail since you're not automatically given `sudo` status when you create the Ubuntu 22 VM, so follow these steps.

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
This should ensure that you can run all the commands with administrator access from now on. Note that you'll have to enter your VM password each time you use a `sudo` command in a new terminal for the first time.

## Step 1: Install MuJoCo (version 3.2.7, the latest version at the moment)

1. Download MuJoCo 3.2.7 from the [official MuJoCo website](https://mujoco.org/) by hitting the **Download** tab.
2. Create a file directory for MuJoCo and extract the tar ball contents into it

```bash
mkdir -p ~/mujoco && cd ~/mujoco
wget https://github.com/deepmind/mujoco/releases/latest/download/mujoco-3.2.7-linux-x86_64.tar.gz
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

If a simulation window pops up and a human falls over, then you're able to run MuJoCo successfully!

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
- `run_go1MujocoSim` - runs the walking Go1 MuJoCo sim

You can also run Unitree's example hardware executables:

- `example_position` - moves the FR foot of Go1 while printing the thigh's joint position and velocity
- `example_velocity` - moves the FR foot of Go1 with a time-varying velocity
- `example_torque` - applies a constant torque to the FR foot of Go1
- `example_walk` - executes high-level walking example on Go1
- `example_joystick` - prints out joystick information received by Go1

All of these scripts can be run by entering the build folder of the project, and typing `./` before any executable file. You will know what files are executable if you type `ls` in terminal and their names are highlighted in a different color from the rest of the files.

```bash
cd ~/go1_cpp_cmake
cd build
ls
./* # replace the asterisk with any executable listed above
```

## Making Changes to the Project

The goal of the CMake project is to enable physical control of the robot based on the code I've already written for simulation control. So, we can leverage the existing example code that Unitree has provided to create new example scripts. We need to extract the low-level information from the robot and send torque commands to the robot at all times, so we can use the format of the example files to achieve this. To add make these new files executable, make sure to edit the CMakeLists.txt file in `unitree_legged_sdk` instead of the top-level one (we'll experiment with that later). For now, all you have to do is add these three lines to the CMakeLists.txt when you make a new file:

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

This way, you'll always catch pre-build issues on the spot. If those pass and you still get errors, you likely mishandled information inside the script instead of leaving syntax errors. Follow the existing formats, use ChatGPT, or ask me to make sure you understand how to make those changes, if you so desire.

To push your changes to the GitHub repo:

```bash
cd ~/go1_cpp_cmake  # make sure you're here before pushing
git add .
git commit -m "Write your commit message here"
git push -u origin main  # you'll have to enter your GitHub username and SSH key here
```

## Troubleshooting

1. **MuJoCo Header Not Found**: If you receive an error regarding MuJoCo headers not being found, ensure that the path to the MuJoCo installation is correctly specified in your `CMakeLists.txt`.

2. **Library Linking Errors**: If you encounter issues linking OSQP or OsqpEigen, verify that the libraries are properly installed, and the paths are correctly set in the `CMakeLists.txt` file.

---

Feel free to contact me if you have any issues with the installation. It can be very finnicky, but once it's done, there's no more hassle. We may download more when working on the physical system, but that's a concern for later.
