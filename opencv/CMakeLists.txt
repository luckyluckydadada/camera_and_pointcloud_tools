PROJECT(opencv_test)

cmake_minimum_required(VERSION 3.5)


find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})


add_executable(mouse mouse.cpp)
target_link_libraries(mouse ${OpenCV_LIBS})
