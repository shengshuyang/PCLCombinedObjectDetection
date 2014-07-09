cmake_minimum_required(VERSION 2.8)

PROJECT(DownSample)

find_package(PCL 1.6 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET( PROJECT_SRCS
main.cpp
extremeFinder.cpp
extremeFinder.h

)

ADD_EXECUTABLE(DownSample ${PROJECT_SRCS})
target_link_libraries(DownSample ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}  ${PCL_LIBRARIES})