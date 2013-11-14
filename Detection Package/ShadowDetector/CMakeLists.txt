cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PCLShadowDetector)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} /usr/local/include/pcl-1.7)
link_directories(${PCL_LIBRARY_DIRS} /usr/local/lib)
add_definitions(${PCL_DEFINITIONS})
set(DCMAKE_BUILD_TYPE = Debug)

#my modifications
set(PROJECT_FILE main.cpp)


add_executable (PCLShadowDetector ${PROJECT_FILE})
target_link_libraries (PCLShadowDetector ${PCL_LIBRARIES})
