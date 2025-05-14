cmake_minimum_required(VERSION 3.5)
project(go1_cpp_cmake)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
cmake_policy(SET CMP0072 NEW)

set(CMAKE_BUILD_TYPE Release) # Release = fast & quiet, Debug = slow & verbose

# check arch and os
message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
  set(ARCH arm64)
endif()

# # Add unitree_legged_sdk library
# add_library(unitree_legged_sdk STATIC IMPORTED)
# set_target_properties(unitree_legged_sdk
#     PROPERTIES
#     IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/unitree_legged_sdk/lib/cpp/amd64/libunitree_legged_sdk.a"
# )
# target_include_directories(unitree_legged_sdk
#     INTERFACE
#     "${CMAKE_CURRENT_SOURCE_DIR}/unitree_legged_sdk/include"
# )

# Add unitree_legged_sdk subdirectory
add_subdirectory(unitree_legged_sdk)

# Find linear algebra and optimization libraries
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

add_definitions(-DEIGEN_STACK_ALLOCATION_LIMIT=0) # Disable stack allocation limit for Eigen if desired

# Find graphics and interaction libraries
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Curses REQUIRED) # for keyboard control through terminal

# Manually define MuJoCo paths
set(MUJOCO_INCLUDE_DIR /usr/local/mujoco/include)  # Adjust based on your system
set(MUJOCO_LIBRARY /usr/local/mujoco/lib/libmujoco.so)  # Adjust based on your system

# Ensure MuJoCo exists
find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h HINTS /usr/local/mujoco/include)
find_library(MUJOCO_LIBRARY mujoco HINTS /usr/local/mujoco/lib)

if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
    message(FATAL_ERROR "MuJoCo 3.2.7 not found. Please check your installation.")
endif()

# Include directories
include_directories(
  include
  ${MUJOCO_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${CURSES_INCLUDE_DIR}
)

# Add main Go1 control library
add_library(go1_main_lib
  include/go1_cpp_cmake/go1Params.h
  # include/go1_cpp_cmake/numericalJacobian.tpp # remove old numericalJacobian function
  include/go1_cpp_cmake/go1Utils.h
  src/go1Utils.cpp
  include/go1_cpp_cmake/go1FK.h
  src/go1FK.cpp
  include/go1_cpp_cmake/go1State.h
  src/go1State.cpp
  include/go1_cpp_cmake/go1StanceMPC.h
  src/go1StanceMPC.cpp
  include/go1_cpp_cmake/go1StateEstimator.h # EKF needs more tuning + needs analytical for faster computation
  src/go1StateEstimator.cpp
  include/go1_cpp_cmake/go1TestFunctions.h
  src/go1TestFunctions.cpp
  include/go1_cpp_cmake/go1FSM.h
  src/go1FSM.cpp
  include/go1_cpp_cmake/go1DataInterface.h
)
target_link_libraries(go1_main_lib
    OsqpEigen::OsqpEigen
    ${MUJOCO_LIBRARY}
    "${CMAKE_CURRENT_SOURCE_DIR}/unitree_legged_sdk/lib/cpp/${ARCH}/libunitree_legged_sdk.a"
)
target_include_directories(go1_main_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/unitree_legged_sdk/include
)

# Add function testing executable
add_executable(run_go1Tests src/runGo1Tests.cpp)
target_link_libraries(run_go1Tests go1_main_lib)

# Add inertia calculation executable
add_executable(run_go1InertiaCalc src/runGo1InertiaCalc.cpp)
target_link_libraries(run_go1InertiaCalc 
    go1_main_lib
    tinyxml2)

# Basic Go1 MuJoCo simulation executable
add_executable(run_go1MujocoSim src/runGo1Mujoco.cpp)
target_link_libraries(run_go1MujocoSim
    go1_main_lib
    glfw
    ${GLEW_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${CURSES_LIBRARIES}
)

# Full Go1 FSM MuJoCo simulation executable w/ interactive mode switching (add velocity control)
add_executable(run_go1FSMMujoco src/runGo1FSMMujoco.cpp)
target_link_libraries(run_go1FSMMujoco
    go1_main_lib
    glfw
    ${GLEW_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${CURSES_LIBRARIES}
)

# Full Go1 FSM hardware executable (use Curses instead of MuJoCo window for interaction)
add_executable(run_go1FSMHardware src/runGo1FSMHardware.cpp)
target_link_libraries(run_go1FSMHardware
    go1_main_lib
    ${CURSES_LIBRARIES}
)
target_link_options(run_go1FSMHardware PRIVATE -no-pie)

# Small reading tester for Go1 hardware control (will help decide the structure of go1DataInterface for hardware)
add_executable(test_go1ReadHardware src/testGo1ReadHardware.cpp)
target_link_libraries(test_go1ReadHardware
    go1_main_lib
)
target_link_options(test_go1ReadHardware PRIVATE -no-pie)

# Small command tester for Go1 hardware control (unsure if will properly work)
add_executable(test_go1JointPDHardware src/testGo1JointPDHardware.cpp)
target_link_libraries(test_go1JointPDHardware
    go1_main_lib
)
target_link_options(test_go1JointPDHardware PRIVATE -no-pie)

# Install libraries and executables
install(TARGETS
    go1_main_lib
    run_go1InertiaCalc
    run_go1Tests
    run_go1MujocoSim
    run_go1FSMMujoco
    run_go1FSMHardware
    DESTINATION lib/${PROJECT_NAME}
)

# Install models directory
install(
  DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
  PATTERN ".gitignore" EXCLUDE
)

# Install include directory
install(
  DIRECTORY include/
  DESTINATION include
  PATTERN ".gitignore" EXCLUDE
)

# Install Python scripts
install(PROGRAMS
  py_src/plotGo1MujocoData.py
  py_src/plotGo1CalcTime.py
  py_src/plotGo1HardwareData.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install data directory
install(
  DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
  PATTERN ".gitignore" EXCLUDE
)
