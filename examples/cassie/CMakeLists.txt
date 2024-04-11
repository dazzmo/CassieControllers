cmake_minimum_required(VERSION 3.20)
project(cassie-osc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Boost REQUIRED)
find_package(glog REQUIRED)
find_package(damotion REQUIRED)
find_package(pinocchio REQUIRED)
find_package(casadi REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

file(GLOB_RECURSE CTRL_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/controller/osc_fixed.h
    ${CMAKE_CURRENT_SOURCE_DIR}/controller/osc.h
    ${CMAKE_CURRENT_SOURCE_DIR}/controller/osc_fixed.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/controller/osc.cc

    ${CMAKE_CURRENT_SOURCE_DIR}/model/actuation.h
    ${CMAKE_CURRENT_SOURCE_DIR}/model/closed_loop_constraint.h
    ${CMAKE_CURRENT_SOURCE_DIR}/model/springs.h
    ${CMAKE_CURRENT_SOURCE_DIR}/model/actuation.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/model/closed_loop_constraint.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/model/springs.cc
)

add_library(cassie_control SHARED ${CTRL_SRC})
target_include_directories(cassie_control PUBLIC ${pinocchio_INCLUDE_DIRS} "${CMAKE_CURRENT_SOURCE_DIR}/")
target_link_libraries(cassie_control damotion::damotion pinocchio::pinocchio )
set_target_properties(cassie_control PROPERTIES LINKER_LANGUAGE CXX)

add_executable(cassie_sim_fixed sim/simulate_fixed.cc)
target_link_libraries(cassie_sim_fixed PUBLIC mujoco_simulator cassie_control glog::glog)

add_executable(cassie_sim sim/simulate.cc)
target_link_libraries(cassie_sim PUBLIC mujoco_simulator cassie_control glog::glog)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/model/urdf/cassie.urdf 
${CMAKE_CURRENT_BINARY_DIR}/cassie.urdf
COPYONLY)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/model/urdf/cassie_fixed.urdf 
${CMAKE_CURRENT_BINARY_DIR}/cassie_fixed.urdf
COPYONLY)

file(CREATE_LINK "${CMAKE_CURRENT_SOURCE_DIR}/sim/agility_cassie" "${CMAKE_CURRENT_BINARY_DIR}/agility_cassie" SYMBOLIC)
