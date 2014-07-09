cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "possible configurations" FORCE)

PROJECT( Shadow_Detection)

add_subdirectory( DownSample)
add_subdirectory( PlaneDetection)
add_subdirectory( RegionGrowingHSV)
add_subdirectory( VisualizeResults)
add_subdirectory( LightDetection)