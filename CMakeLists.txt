cmake_minimum_required(VERSION 3.5)
project(go1_cpp_cmake)

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

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find third-party dependencies
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

cmake_policy(SET CMP0072 NEW)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Curses REQUIRED) # for keyboard control

# # Manually link TinyXML2 for inertia calculation
# link_directories(/usr/lib/x86_64-linux-gnu)
# include_directories(/usr/include)

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
  include/go1_cpp_cmake/go1Utils.h
  src/go1Utils.cpp
  include/go1_cpp_cmake/go1FK.h
  src/go1FK.cpp
  include/go1_cpp_cmake/go1State.h
  src/go1State.cpp
  include/go1_cpp_cmake/go1StanceMPC.h
  src/go1StanceMPC.cpp
  include/go1_cpp_cmake/go1StateEstimator.h
  src/go1StateEstimator.cpp
  include/go1_cpp_cmake/go1TestFunctions.h
  src/go1TestFunctions.cpp
)
target_link_libraries(go1_main_lib
    OsqpEigen::OsqpEigen
    ${MUJOCO_LIBRARY}
    # unitree_legged_sdk
    # "${CMAKE_CURRENT_SOURCE_DIR}/unitree_legged_sdk/lib/cpp/amd64/libunitree_legged_sdk.a"
)

# Add function testing executable
add_executable(run_go1Tests src/runGo1Tests.cpp)
target_link_libraries(run_go1Tests go1_main_lib)

# Add inertia calculation executable
add_executable(run_go1InertiaCalc src/runGo1InertiaCalc.cpp)
target_link_libraries(run_go1InertiaCalc 
    go1_main_lib
    # /usr/lib/x86_64-linux-gnu/libtinyxml2.so
    tinyxml2)

# Add Mujoco simulation executables
add_executable(run_go1MujocoSim src/runGo1Mujoco.cpp)
target_link_libraries(run_go1MujocoSim
    go1_main_lib
    glfw
    ${GLEW_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${CURSES_LIBRARIES}
)

add_executable(run_go1Stand src/runGo1StartupMujoco.cpp)
target_link_libraries(run_go1Stand
    go1_main_lib
    glfw
    ${GLEW_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${CURSES_LIBRARIES}
)

# Add hardware control executable
add_executable(run_go1HardwareControl src/runGo1HardwareControl.cpp)
target_include_directories(run_go1HardwareControl PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/unitree_legged_sdk/include")
target_link_libraries(run_go1HardwareControl
    go1_main_lib
    "${CMAKE_CURRENT_SOURCE_DIR}/unitree_legged_sdk/lib/cpp/${ARCH}/libunitree_legged_sdk.a"
)
target_link_options(run_go1HardwareControl PRIVATE -no-pie)

# Install libraries and executables
install(TARGETS
    go1_main_lib
    run_go1Tests
    run_go1MujocoSim
    run_go1InertiaCalc
    run_go1Stand
    run_go1HardwareControl
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
  DESTINATION lib/${PROJECT_NAME}
)

# Install data directory
install(
  DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
  PATTERN ".gitignore" EXCLUDE
)
