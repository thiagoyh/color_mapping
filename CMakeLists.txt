
cmake_minimum_required(VERSION 3.5)
project(mapping)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(PCL REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

message(STATUS "OpenCV_INCLUDE_DIRS:" ${OpenCV_INCLUDE_DIRS})
message(STATUS "OpenCV_LIBRARIES:" ${OpenCV_LIBRARIES})
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


file(GLOB_RECURSE ALL_SRCS "src/*.cc" "include/*.cc")
# file(GLOB_RECURSE ALL_EXECUTABLES "src/*_main.cc" "src/*_test.cc")
# list(REMOVE_ITEM ALL_SRCS ${ALL_EXECUTABLES})

add_executable(mapping node_main.cc ${ALL_SRCS})
target_link_libraries(mapping ${PCL_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)

add_executable(color color_main.cc include/color_mapping.cc)
target_link_libraries(color ${PCL_LIBRARIES})
target_link_libraries(color ${OpenCV_LIBS} yaml-cpp)

add_executable(color_realtime real_time_main.cc include/real_time_color.cc)
target_link_libraries(color_realtime ${PCL_LIBRARIES})
target_link_libraries(color_realtime ${OpenCV_LIBS} yaml-cpp)
