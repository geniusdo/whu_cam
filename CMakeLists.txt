cmake_minimum_required(VERSION 3.6)
project(whu_cam)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_C_FLAGS_Release "${CMAKE_C_FLAGS_Release} -O3")

find_package(OpenCV 4 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    tf
    roscpp
    roslib
    mavros_msgs
    std_msgs
    sensor_msgs
)
include_directories(${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

add_executable(camDriver src/whu_cam.cpp)
target_link_libraries(camDriver ${OpenCV_LIBS} ${catkin_LIBRARIES} gxiapi -lpthread -lm -ltbb yaml-cpp)


