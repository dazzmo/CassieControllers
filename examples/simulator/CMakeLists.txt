cmake_minimum_required(VERSION 3.20)
project(mujoco_simulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)

#TODO: Jesse suggested not usin GLOB, write out file path instead?
# TODO: Noted, will look at alternatives
file(GLOB_RECURSE SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cc
)

add_library(mujoco_simulator SHARED ${SRC})
target_include_directories(mujoco_simulator PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/")
target_link_libraries(mujoco_simulator PUBLIC mujoco::mujoco glfw)
set_target_properties(mujoco_simulator PROPERTIES LINKER_LANGUAGE CXX)