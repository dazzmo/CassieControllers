cmake_minimum_required(VERSION 3.20)
project(cassie_stand_sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(glog REQUIRED)
find_package(damotion REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

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

# Full-body Cassie dynamics, fixed to stand
add_executable(cassie_codegen_fixed ${CMAKE_CURRENT_SOURCE_DIR}/model/codegen_fixed.cc)
target_link_libraries(cassie_codegen_fixed PUBLIC 
code_generator
)
target_include_directories(cassie_codegen_fixed PUBLIC ${pinocchio_INCLUDE_DIRS})

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/model/urdf/cassie_fixed.urdf 
${CMAKE_CURRENT_BINARY_DIR}/cassie_fixed.urdf
COPYONLY)

add_executable(cassie_sim_fixed sim/simulate_fixed.cc)
target_link_libraries(cassie_sim_fixed PUBLIC mujoco_simulator Eigen3::Eigen cassie_control glog::glog)

# Full-body cassie dynamics, free from stand
add_executable(cassie_codegen ${CMAKE_CURRENT_SOURCE_DIR}/model/codegen.cc)
target_link_libraries(cassie_codegen PUBLIC 
code_generator
)
target_include_directories(cassie_codegen PUBLIC ${pinocchio_INCLUDE_DIRS})

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/model/urdf/cassie.urdf 
${CMAKE_CURRENT_BINARY_DIR}/cassie.urdf
COPYONLY)

add_executable(cassie_sim sim/simulate.cc)
target_link_libraries(cassie_sim PUBLIC mujoco_simulator Eigen3::Eigen cassie_control glog::glog)

file(CREATE_LINK "${CMAKE_CURRENT_SOURCE_DIR}/sim/agility_cassie" "${CMAKE_CURRENT_BINARY_DIR}/agility_cassie" SYMBOLIC)
