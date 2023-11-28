cmake_minimum_required(VERSION 3.20)
project(arm_simulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(mujoco REQUIRED)
find_package(pinocchio REQUIRED)
find_package(casadi REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

file(GLOB_RECURSE CTRL_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/controller/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/controller/*.cc
)

file(GLOB_RECURSE ARM_CG_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/model/cg/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/model/cg/*.c
)

add_library(arm_control SHARED ${CTRL_SRC} ${ARM_CG_SRC})
target_link_libraries(arm_control controllers)
target_include_directories(arm_control PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/")
set_target_properties(arm_control PROPERTIES LINKER_LANGUAGE CXX)

add_executable(arm_codegen ${CMAKE_CURRENT_SOURCE_DIR}/model/codegen.cc)
target_link_libraries(arm_codegen PUBLIC 
code_generator
)
target_include_directories(arm_codegen PUBLIC ${pinocchio_INCLUDE_DIRS})

file(CREATE_LINK "${CMAKE_CURRENT_SOURCE_DIR}/model/urdf/arm.urdf" "${CMAKE_CURRENT_BINARY_DIR}/arm.urdf" SYMBOLIC)
file(CREATE_LINK "${CMAKE_CURRENT_SOURCE_DIR}/sim/arm.xml" "${CMAKE_CURRENT_BINARY_DIR}/arm.xml" SYMBOLIC)
file(CREATE_LINK "${CMAKE_CURRENT_SOURCE_DIR}/sim/scene.xml" "${CMAKE_CURRENT_BINARY_DIR}/scene.xml" SYMBOLIC)


add_executable(arm_simulator sim/simulate.cc)
target_link_libraries(arm_simulator PUBLIC mujoco_simulator Eigen3::Eigen arm_control)