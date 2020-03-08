cmake_minimum_required(VERSION 2.8)
PROJECT(lidar_driver)

#Include directories
include_directories(
     ${CMAKE_SOURCE_DIR}
)
############## YDLIDAR SDK START#####################################
#link library directories

if(WIN32)
include_directories(${CMAKE_SOURCE_DIR}/windows)
add_executable(${PROJECT_NAME} main.cpp driver.cpp driver.h serial.h windows/win_serial.cpp windows/win_serial.h)
else()
include_directories(${CMAKE_SOURCE_DIR}/unix)
add_executable(${PROJECT_NAME} main.cpp driver.cpp driver.h serial.h unix/unix_serial.cpp unix/unix_serial.h)
endif()





############## YDLIDAR SDK END#####################################
