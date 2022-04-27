cmake_minimum_required( VERSION 2.8 )

# Create Project
project(solution)

# eigen
find_package(Eigen3 REQUIRED)
include_directories("/usr/include/eigen3")
#Boost
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
#VTK
find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})
link_directories(${VTK_LIBRARY_DIRS})

#add_executable( VLP_grabber main16.cpp )
#add_executable( HDL_grabber main32.cpp )
add_executable( pcap_checker.exe HDPcapChecker.cpp hdl_viewer_grabber_helper.cpp hdl_grabber.cpp)

# Set StartUp Project (This setting is able to enable by using CMake 3.6.0 RC1 or later.)
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "project" )

# Find Packages
find_package( PCL 1.8 REQUIRED )
# [C/C++]>[General]>[Additional Include Directories]
include_directories( ${PCL_INCLUDE_DIRS} )
# [C/C++]>[Preprocessor]>[Preprocessor Definitions]
add_definitions( ${PCL_DEFINITIONS} )
# For Use Not PreCompiled Features 
#add_definitions( -DPCL_NO_PRECOMPILE )
# [Linker]>[General]>[Additional Library Directories]
link_directories( ${PCL_LIBRARY_DIRS} )
# [Linker]>[Input]>[Additional Dependencies]
#target_link_libraries( VLP_grabber ${PCL_LIBRARIES})
#target_link_libraries( HDL_grabber ${PCL_LIBRARIES})
target_link_libraries(  pcap_checker.exe  ${PCL_LIBRARIES} )