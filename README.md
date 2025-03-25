
# CMake Project Installation Guide

This guide covers the installation of dependencies required for building and running the CMake project `go1_cpp_cmake`. The package is intended for use on Ubuntu 22.04 and integrates with MuJoCo 3.2.7 for simulation and OSQP for quadratic programming (QP) used in Model Predictive Control (MPC) formulations. Make sure you go to [this Overleaf link](https://www.overleaf.com/read/rswbkdgmngpz#2c8aa1) to take a look at the overall project formulation and associated goals.

By the end of the installation, you should have these packages/libraries properly configured and installed:

- **MuJoCo 3.2.7 (newest version at the moment)** (Simulation)
- **OSQP v0.6.3** (QP Solver for GRF MPC)
- **OsqpEigen v0.7.0** (C++ wrapper for OSQP w/ Eigen linear algebra library interface)

## Pre-Installation Step: Add Self to Sudo Group
Naturally, you can't really run any of these installation commands without being part of the administrator, or `sudo` group. Running any of the following terminal commands is going to fail since you're not automatically given `sudo` status when you create the Ubuntu 18 VM, so follow these steps.

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

Once all dependencies are installed, create a catkin workspace and clone the `go1_cpp_cmake` project, then build.

1. **Create a workspace**:

```bash
mkdir -p ~/ros2_ws/src
```

2. **Build the workspace**:

```bash
cd ~/ros2_ws
colcon build
```

3. **Source the workspace**:

```bash
source install/setup.bash
```

4. **Clone the `go1_cpp_humble` package into the workspace's src folder**:

```bash
cd ~/ros2_ws/src
git clone https://github.com/purdue-tracelab/go1_cpp_humble.git
# requires a GitHub SSH key for cloning this repo, look up the tutorial on GitHub
```

5. **Build the package**:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

# Post-Installation
## Running the Package

Once successfully built, you can run the package executables as follows:

- To test the package class functions from **go1State**, **go1FK**, **grfMPC**, etc. :

```bash
ros2 run go1_cpp_humble run_go1Tests
```

- To run the **MuJoCo simulation**:

```bash
ros2 run go1_cpp_humble run_go1MujocoSim
```

There are also launch files available, which do the exact same thing.

```bash
ros2 launch go1_cpp_humble runTests.launch.py
ros2 launch go1_cpp_humble runMujoco.launch.py
```

## Making Changes to the Package

If you feel comfortable venturing into modifying the package for your own purposes, feel free to start editing the existing scripts in the `src` directory and the header files in the `include` directory. This could range from adding terminal print statements with `std::cout <<`, recording data into a CSV file, or adjusting the parameters in `go1Params.h` to modify the overall behavior of the `grfMPC` functions and `swingPD` in the `go1State` class. I've set up a branch on the repo called `undergrads` for you guys to meddle with at home/in lab.

To checkout this specific branch and work on it exclusively, run `git checkout undergrads` to swap to that branch, then work away! We can discuss the results in the weekly meetings or whenever y'all are in lab. Just make sure you remember these three lines everytime you want to test new versions of code you've written:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

With these, you'll always catch pre-build issues on the spot. If those pass and you still get errors, you likely mishandled information inside the script instead of leaving syntax errors. Another thing to note: if you want to make completely new scripts and headers, you need to reflect those changes in the CMakeLists.txt file (in `go1_cpp_humble`, not the root of the `ros2_ws`). Follow the existing format and use ChatGPT to make sure you understand how to make those changes, if you so desire.

## Troubleshooting

1. **MuJoCo Header Not Found**: If you receive an error regarding MuJoCo headers not being found, ensure that the path to the MuJoCo installation is correctly specified in your `CMakeLists.txt`.

2. **Library Linking Errors**: If you encounter issues linking OSQP or OsqpEigen, verify that the libraries are properly installed, and the paths are correctly set in the `CMakeLists.txt` file.

---

Feel free to contact me if you have any issues with the installation. It can be very finnicky, but once it's done, there's no more hassle. We may download more when working on the physical system, but that's a concern for later.
