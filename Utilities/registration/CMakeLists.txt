cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PCLRegistration)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#my modifications
set(PROJECT_FILE registration.cpp)


add_executable (PCLRegistration ${PROJECT_FILE})
target_link_libraries (PCLRegistration ${PCL_LIBRARIES})
