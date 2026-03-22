cmake_minimum_required(VERSION 3.18)

project(OptRobot)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Main entry point
add_executable(Main
    src/main.cpp
)

# --- MuJoCo engine (uncomment when libmujoco is installed) ---
# find_package(mujoco REQUIRED)
# add_library(physics_mujoco
#     src/physics/mujoco/MuJoCoEngine.cpp
# )
# target_include_directories(physics_mujoco PUBLIC src/interfaces)
# target_link_libraries(physics_mujoco PUBLIC mujoco::mujoco)
