cmake_minimum_required(VERSION 3.20)
# project(controllers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Eigen3 3.3 REQUIRED)
find_package(glog REQUIRED)
find_package(qpOASES REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

file(GLOB_RECURSE INCLUDE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/controllers/*.h
)

file(GLOB_RECURSE SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc
)

# TODO: Figure out why we need path to qpOASES
add_library(controllers SHARED ${INCLUDE} ${SRC})
target_link_libraries(controllers PUBLIC Eigen3::Eigen glog::glog /usr/local/lib/libqpOASES.dylib)
target_include_directories(controllers PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/")
set_target_properties(controllers PROPERTIES LINKER_LANGUAGE CXX)