cmake_minimum_required(VERSION 2.8)
PROJECT(lidar_tutorial C)

#Include directories
include_directories(
     ${CMAKE_SOURCE_DIR}
)
############## YDLIDAR SDK START#####################################
#find ydlidar_sdk package
find_package(ydlidar_sdk REQUIRED)
#Include directories
include_directories(
  ${YDLIDAR_SDK_INCLUDE_DIRS}
)
#link library directories
link_directories(${YDLIDAR_SDK_LIBRARY_DIRS})

add_executable(${PROJECT_NAME} lidar_tutorial.c)
#Link your project to ydlidar_sdk library.
target_link_libraries(${PROJECT_NAME} ${YDLIDAR_SDK_LIBRARIES} -lstdc++ -lm)

############## YDLIDAR SDK END#####################################
