cmake_minimum_required(VERSION 3.9)
project(fusion)
set(CMAKE_CXX_STANDARD 17)
# eigen
find_package(Eigen3 REQUIRED)
include_directories("/usr/include/eigen3")
# pcl
find_package( PCL REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS} )  #.h 文件
link_directories(${PCL_LIBRARY_DIRS}) # .so文件
#Boost
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
#VTK
find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})
link_directories(${VTK_LIBRARY_DIRS})
# #OPENCV
# find_package(OpenCV REQUIRED)
# include_directories(${OPENCV_INCLUDE_DIRS})
# link_directories(${OPENCV_LIBRARY_DIRS})

add_executable(fusion fusion.cpp)
target_link_libraries(fusion
libboost_system.so libboost_filesystem.so
${VTK_LIBRARIES} ${PCL_LIBRARIES} 
/usr/lib/x86_64-linux-gnu/libopencv_core.so
/usr/lib/x86_64-linux-gnu/libopencv_highgui.so
/usr/lib/x86_64-linux-gnu/libopencv_imgproc.so
/usr/lib/x86_64-linux-gnu/libopencv_videoio.so
/usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so
)

