cmake_minimum_required( VERSION 2.8 )

# Create Project
project(solution)

add_executable( VLP_grabber main16.cpp )
add_executable( HDL_grabber main32.cpp )

# Set StartUp Project (This setting is able to enable by using CMake 3.6.0 RC1 or later.)
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "project" )

include(FindPCAP.cmake)

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
target_link_libraries( VLP_grabber ${PCL_LIBRARIES} ${PCAP_LIBRARY})
target_link_libraries( HDL_grabber ${PCL_LIBRARIES} ${PCAP_LIBRARY})
