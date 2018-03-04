cmake_minimum_required(VERSION 2.8.3)
project(graph_laser)

add_definitions(-std=c++11)

find_package(catkin_simple 0.1.0 REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    genmsg
    message_generation
    tf
    rosbag
    visualization_msgs
)
set(CMAKE_PREFIX_PATH "/home/ren/opencv2/opencv2/share/OpenCV")
FIND_PACKAGE(OpenCV 2 REQUIRED)
include_directories(
    ${OpenCV_INCLUDE_DIRS}
)
link_directories(
  ${OpenCV_LIBRARY_DIRS}
)

find_package(Boost REQUIRED COMPONENTS system thread)

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(laser_slam_node1
    src/laser_slam_worker.cpp
)

cs_add_library(graph_laser1
    src/graph_laser.cpp
)
target_link_libraries(graph_laser1
    ${catkin_LIBRARIES}
    laser_slam_node1
)

add_executable(graph_laser_node
    src/graph_laser_node.cpp
)
target_link_libraries(graph_laser_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    laser_slam_node1
    graph_laser1
)

add_executable(save_map_client1
    src/save_map_client.cpp
)

target_link_libraries(save_map_client1
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

add_executable(pub_offline_cloud
    src/pub_offline_cloud.cpp
)
target_link_libraries(pub_offline_cloud
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

cs_install()
cs_export()

