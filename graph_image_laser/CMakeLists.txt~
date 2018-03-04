cmake_minimum_required(VERSION 2.8.3)
project(graph_image_laser)

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

cs_add_library(laser_slam_node
    src/laser_slam_worker.cpp
)
cs_add_library(laser_image_slam_node
    src/laser_image_slam_worker.cpp
)
target_link_libraries(laser_image_slam_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

cs_add_library(graph_laser
    src/graph_laser.cpp
)
target_link_libraries(graph_laser
    ${catkin_LIBRARIES}
    laser_slam_node
)

cs_add_library(graph_image_laser
    src/graph_image_laser.cpp
)
target_link_libraries(graph_image_laser
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    laser_image_slam_node
)

#add_executable(graph_laser_node
#    src/graph_laser_node.cpp
#)
#target_link_libraries(graph_laser_node
#    ${catkin_LIBRARIES}
#    laser_slam_node
#    graph_laser
#)


add_executable(graph_image_laser_node
    src/graph_image_laser_node.cpp
    src/visual/utility/utility.cpp
    src/visual/utility/CameraPoseVisualization.cpp
    src/visual/loop-closure/ThirdParty/DBoW/BowVector.cpp
    src/visual/loop-closure/ThirdParty/DBoW/FBrief.cpp
    src/visual/loop-closure/ThirdParty/DBoW/FeatureVector.cpp
    src/visual/loop-closure/ThirdParty/DBoW/QueryResults.cpp
    src/visual/loop-closure/ThirdParty/DBoW/ScoringObject.cpp
    src/visual/loop-closure/ThirdParty/DUtils/Random.cpp
    src/visual/loop-closure/ThirdParty/DUtils/Timestamp.cpp
    src/visual/loop-closure/ThirdParty/DVision/BRIEF.cpp
    src/visual/loop-closure/ThirdParty/VocabularyBinary.cpp
    src/visual/loop-closure/loop_closure.cpp
)
target_link_libraries(graph_image_laser_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    laser_image_slam_node
    graph_image_laser
)

add_executable(demo_dooploop
    src/demo_dooploop.cpp
    src/visual/utility/utility.cpp
    src/visual/utility/CameraPoseVisualization.cpp
    src/visual/loop-closure/ThirdParty/DBoW/BowVector.cpp
    src/visual/loop-closure/ThirdParty/DBoW/FBrief.cpp
    src/visual/loop-closure/ThirdParty/DBoW/FeatureVector.cpp
    src/visual/loop-closure/ThirdParty/DBoW/QueryResults.cpp
    src/visual/loop-closure/ThirdParty/DBoW/ScoringObject.cpp
    src/visual/loop-closure/ThirdParty/DUtils/Random.cpp
    src/visual/loop-closure/ThirdParty/DUtils/Timestamp.cpp
    src/visual/loop-closure/ThirdParty/DVision/BRIEF.cpp
    src/visual/loop-closure/ThirdParty/VocabularyBinary.cpp
    src/visual/loop-closure/loop_closure.cpp
)

target_link_libraries(demo_dooploop ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(save_map_client
    src/save_map_client.cpp
)
target_link_libraries(save_map_client
    ${catkin_LIBRARIES}
)

cs_install()
cs_export()

