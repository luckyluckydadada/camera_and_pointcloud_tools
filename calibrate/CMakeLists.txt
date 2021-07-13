cmake_minimum_required(VERSION 3.9)
project(demo)
set(CMAKE_CXX_STANDARD 17)


add_executable(demo test.cpp)
FIND_PACKAGE( OpenCV 3.2 REQUIRED )
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )
 
TARGET_LINK_LIBRARIES( demo
    ${OpenCV_LIBS}
)
