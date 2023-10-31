cmake_minimum_required(VERSION 3.20)
project(code_generator)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(pinocchio REQUIRED)
find_package(casadi REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

file(GLOB_RECURSE SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cc
)

add_library(code_generator SHARED ${SRC})
target_include_directories(code_generator PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/" ${pinocchio_INCLUDE_DIRS})
target_link_libraries(code_generator PUBLIC ${pinocchio_LIBRARIES} casadi Eigen3::Eigen
)
set_target_properties(code_generator PROPERTIES LINKER_LANGUAGE CXX)