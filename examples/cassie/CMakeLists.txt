cmake_minimum_required(VERSION 3.20)
project(cassie_stand_sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# TODO: How many of these do we still need?
find_package(glog REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(pinocchio)
find_package(casadi)

# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

file(GLOB_RECURSE CTRL_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/controller/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/controller/*.cc
)

file(GLOB_RECURSE CASSIE_CODEGEN_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/model/cg/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/model/cg/*.c
)

add_library(cassie_control SHARED ${CTRL_SRC} ${CASSIE_CODEGEN_SRC})
target_link_libraries(cassie_control controllers)
target_include_directories(cassie_control PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/")
set_target_properties(cassie_control PROPERTIES LINKER_LANGUAGE CXX)

# Full-body Cassie dynamics
add_executable(cassie_codegen ${CMAKE_CURRENT_SOURCE_DIR}/model/codegen.cc)
target_link_libraries(cassie_codegen PUBLIC 
code_generator
)
target_include_directories(cassie_codegen PUBLIC ${pinocchio_INCLUDE_DIRS})

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/model/urdf/cassie_fixed.urdf 
${CMAKE_CURRENT_BINARY_DIR}/cassie_fixed.urdf
COPYONLY)

add_executable(cassie_sim sim/simulate.cc)
target_link_libraries(cassie_sim PUBLIC mujoco_simulator Eigen3::Eigen cassie_control glog::glog)

# Single-leg Cassie dynamics
# TODO: Remove these when we don't need them anymore!
add_executable(cassie_leg_codegen ${CMAKE_CURRENT_SOURCE_DIR}/model/codegen_leg.cc)
target_link_libraries(cassie_leg_codegen PUBLIC 
code_generator
)
target_include_directories(cassie_leg_codegen PUBLIC ${pinocchio_INCLUDE_DIRS})

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/model/urdf/cassie_leg.urdf 
${CMAKE_CURRENT_BINARY_DIR}/cassie_leg.urdf
COPYONLY)

add_executable(cassie_leg_sim sim/simulate_leg.cc)
target_link_libraries(cassie_leg_sim PUBLIC mujoco_simulator Eigen3::Eigen cassie_control glog::glog)

file(CREATE_LINK "${CMAKE_CURRENT_SOURCE_DIR}/sim/agility_cassie" "${CMAKE_CURRENT_BINARY_DIR}/agility_cassie" SYMBOLIC)
