cmake_minimum_required(VERSION 2.8)

PROJECT(RegionGrowingHSV)

find_package(PCL 1.6 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET( PROJECT_SRCS
main.cpp
region_growing_hsv.cpp
region_growing_hsv.h
region_growing_hsv.hpp

)

ADD_EXECUTABLE(RegionGrowingHSV ${PROJECT_SRCS})
target_link_libraries(RegionGrowingHSV ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}  ${PCL_LIBRARIES})