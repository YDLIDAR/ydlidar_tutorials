cmake_minimum_required(VERSION 2.8)
PROJECT(lidar_tutorial)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_definitions(-std=c++11) # Use C++11


#Include directories
include_directories(
     ${CMAKE_SOURCE_DIR}
)
############## YDLIDAR SDK START#####################################
#find ydlidar_sdk package
find_package(ydlidar_sdk REQUIRED)
#Include directories
include_directories(
  ${ydlidar_sdk_INCLUDE_DIRS}
)

#link library directories
link_directories(${ydlidar_sdk_LIBRARY_DIRS})

add_executable(${PROJECT_NAME} lidar_tutorial.cpp)

#Link your project to ydlidar_sdk library.
target_link_libraries(${PROJECT_NAME} ${ydlidar_sdk_LIBRARIES})

############## YDLIDAR SDK END#####################################
