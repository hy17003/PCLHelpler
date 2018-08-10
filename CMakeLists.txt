cmake_minimum_required(VERSION 2.8.4)
project(lidar2image)

set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

include_directories(/usr/local/include/eigen3)
#include_directories("/usr/local/include/sophus")
include_directories("../include")
include_directories("./include")

aux_source_directory(src SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} ${Sophus_LIBRARIES})
