cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
#set(CMAKE_BUILD_TYPE Release)
project(test)

include_directories("/usr/local/include/eigen3")

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})


aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(test ${PCL_LIBRARIES})
target_link_libraries(test ${OpenCV_LIBS})
target_link_libraries(test ${Sophus_LIBRARIES})
