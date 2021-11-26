cmake_minimum_required(VERSION 3.9)
project(pcl-tools)
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
# find_package(OPENCV REQUIRED)
# include_directories(${OPENCV_INCLUDE_DIRS})
# link_directories(${OPENCV_LIBRARY_DIRS})

add_executable(pcdbin2pcdascii_pcdascii2pcdbin pcdbin2pcdascii_pcdascii2pcdbin.cpp)
target_link_libraries(pcdbin2pcdascii_pcdascii2pcdbin
libboost_system.so libboost_filesystem.so
${PCL_LIBRARIES} 
)

add_executable(csv2pcd csv2pcd.cpp)
target_link_libraries(csv2pcd
libboost_system.so libboost_filesystem.so
${PCL_LIBRARIES} 
)

add_executable(kittibin2pcd kittibin2pcd.cpp)
target_link_libraries(kittibin2pcd
libboost_system.so libboost_filesystem.so
${PCL_LIBRARIES} 
)


add_executable(ply2pcd ply2pcd.cpp)
target_link_libraries(ply2pcd
libboost_system.so libboost_filesystem.so
${PCL_LIBRARIES} 
)