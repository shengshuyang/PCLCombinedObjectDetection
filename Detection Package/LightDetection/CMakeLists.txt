cmake_minimum_required(VERSION 2.8)

PROJECT(LightDetection)

find_package(PCL 1.6 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET( PROJECT_SRCS
 extremeFinder.h
 std_include.h
 vector3.h
 extremeFinder.cpp
 hough_accumulator.cpp
 main.cpp
)

ADD_EXECUTABLE(LightDetection ${PROJECT_SRCS})
target_link_libraries(LightDetection ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}  ${PCL_LIBRARIES})