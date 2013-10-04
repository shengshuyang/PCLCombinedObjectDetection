cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PCLGeonDetection)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#my modifications
set(PROJECT_FILE geonDetector.cpp geonDetector.h PCLGeonDetection.cpp std_include.h)


add_executable (PCLGeonDetection ${PROJECT_FILE})
target_link_libraries (PCLGeonDetection ${PCL_LIBRARIES})
