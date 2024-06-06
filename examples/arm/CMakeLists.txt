cmake_minimum_required(VERSION 3.20)
project(arm_simulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Boost REQUIRED)
find_package(glog REQUIRED)
find_package(damotion REQUIRED)
find_package(pinocchio REQUIRED)
find_package(casadi REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)


add_library(arm_control SHARED 
${CMAKE_CURRENT_SOURCE_DIR}/controller/osc.h
${CMAKE_CURRENT_SOURCE_DIR}/controller/osc.cc
)

target_include_directories(arm_control PUBLIC ${pinocchio_INCLUDE_DIRS} "${CMAKE_CURRENT_SOURCE_DIR}/")
target_link_libraries(arm_control damotion pinocchio::pinocchio )
set_target_properties(arm_control PROPERTIES LINKER_LANGUAGE CXX)

add_executable(arm_simulator sim/simulate.cc)
target_link_libraries(arm_simulator PUBLIC mujoco_simulator arm_control glog::glog)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/model/urdf/arm.urdf 
${CMAKE_CURRENT_BINARY_DIR}/arm.urdf
COPYONLY)

file(CREATE_LINK "${CMAKE_CURRENT_SOURCE_DIR}/model/urdf/arm.urdf" "${CMAKE_CURRENT_BINARY_DIR}/arm.urdf" SYMBOLIC)
file(CREATE_LINK "${CMAKE_CURRENT_SOURCE_DIR}/sim/arm.xml" "${CMAKE_CURRENT_BINARY_DIR}/arm.xml" SYMBOLIC)
file(CREATE_LINK "${CMAKE_CURRENT_SOURCE_DIR}/sim/scene.xml" "${CMAKE_CURRENT_BINARY_DIR}/scene.xml" SYMBOLIC)