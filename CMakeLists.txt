cmake_minimum_required(VERSION 3.2)
project(ddp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-Wall")

find_package(Eigen3 REQUIRED)

include_directories(husky_control_ilqg/include)

add_library(controller_1d controller_1d.cpp)
target_link_libraries(controller_1d Eigen3::Eigen)

#add_library(controller_1d controller_1d.cpp)
#target_link_libraries(controller_1d Eigen3::Eigen)

add_executable(ddp main.cpp)
target_link_libraries(ddp controller_1d Eigen3::Eigen)
