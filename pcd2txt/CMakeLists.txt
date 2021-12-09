cmake_minimum_required(VERSION 3.0.2)
project(pcd2txt)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

set(MAIN_SOURCE "pcd2txt.cpp")
add_executable(${PROJECT_NAME} ${MAIN_SOURCE})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})