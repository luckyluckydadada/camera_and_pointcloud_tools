cmake_minimum_required(VERSION 3.9)
project(pcl-tools)
set(CMAKE_CXX_STANDARD 17)

add_executable(las2txt las2txt.cpp)
target_link_libraries(las2txt
/usr/lib/liblas.so
/usr/lib/liblas_c.so
)

