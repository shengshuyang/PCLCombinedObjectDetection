cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(grab_sample)

find_package(PCL 1.7 REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
set(DCMAKE_BUILD_TYPE Release)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIR} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (grab_sample grab_sample.cpp)
target_link_libraries (grab_sample ${PCL_LIBRARIES})
