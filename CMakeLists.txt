cmake_minimum_required(VERSION 3.10)
project(s_pipe_kinematics)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Source files
set(SOURCES
    src/forward_kinematics.cpp
    src/inverse_kinematics.cpp
    src/main.cpp
)

# Add executable
add_executable(s_pipe_demo ${SOURCES})

# Link libraries
target_link_libraries(s_pipe_demo Eigen3::Eigen)

# Optional: Add an option to build with debug info
# option(BUILD_DEBUG "Build with debug information" OFF)
# if(BUILD_DEBUG)
#     set(CMAKE_BUILD_TYPE Debug)
# else()
#     set(CMAKE_BUILD_TYPE Release)
# endif()

# Print configuration summary
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
message(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")