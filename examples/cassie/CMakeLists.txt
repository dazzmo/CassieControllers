cmake_minimum_required(VERSION 3.20)
project(cassie_simulator)

set(CMAKE_CXX_STANDARD 11)
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

file(GLOB_RECURSE CASSIE_CG_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/model/cg/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/model/cg/*.c
)

add_library(cassie_control SHARED ${CTRL_SRC} ${CASSIE_CG_SRC})
target_link_libraries(cassie_control controllers)
target_include_directories(cassie_control PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/")
set_target_properties(cassie_control PROPERTIES LINKER_LANGUAGE CXX)

add_executable(cassie_codegen ${CMAKE_CURRENT_SOURCE_DIR}/model/codegen.cc)
target_link_libraries(cassie_codegen PUBLIC 
${pinocchio_LIBRARIES}
casadi
Eigen3::Eigen
)
target_include_directories(cassie_codegen PUBLIC ${pinocchio_INCLUDE_DIRS})

add_executable(cassie_codegen_leg ${CMAKE_CURRENT_SOURCE_DIR}/model/codegen_leg.cc)
target_link_libraries(cassie_codegen_leg PUBLIC 
${pinocchio_LIBRARIES}
casadi
Eigen3::Eigen
)
target_include_directories(cassie_codegen_leg PUBLIC ${pinocchio_INCLUDE_DIRS})

add_executable(cassie_heel_spring_test test/heel_spring_test.cc)
target_link_libraries(cassie_heel_spring_test PUBLIC Eigen3::Eigen cassie_control)

add_executable(cassie_simulator sim/simulate.cc)
target_link_libraries(cassie_simulator PUBLIC mujoco::mujoco glfw Eigen3::Eigen cassie_control)