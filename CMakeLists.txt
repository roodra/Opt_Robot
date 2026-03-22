cmake_minimum_required(VERSION 3.18)

project(OptRobot)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- MuJoCo ---
# Default to the system install location; override with -DMUJOCO_DIR=<path>
if(NOT DEFINED MUJOCO_DIR)
    set(MUJOCO_DIR "$ENV{HOME}/.local/mujoco-3.2.3")
endif()

find_library(MUJOCO_LIB
    NAMES mujoco
    HINTS "${MUJOCO_DIR}/lib"
    REQUIRED
)

add_library(mujoco SHARED IMPORTED)
set_target_properties(mujoco PROPERTIES
    IMPORTED_LOCATION             "${MUJOCO_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_DIR}/include"
)

# --- Main executable ---
add_executable(Main
    src/main.cpp
    src/physics/mujoco/MuJoCoEngine.cpp
    src/problems/swingup/SwingUpProblem.cpp
    src/optimization/random_search/RandomSearch.cpp
    src/optimization/simulated_annealing/SimulatedAnnealing.cpp
)
target_include_directories(Main PRIVATE src)
target_link_libraries(Main PRIVATE mujoco)

# Embed the MuJoCo lib path so the binary runs without LD_LIBRARY_PATH
set_target_properties(Main PROPERTIES
    BUILD_RPATH  "${MUJOCO_DIR}/lib"
    INSTALL_RPATH "${MUJOCO_DIR}/lib"
)
