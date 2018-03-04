#include <thread>

#include <ros/ros.h>

#include "graph_laser/graph_laser.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "LaserMapper");
  ros::NodeHandle node_handle("~");

  graphLaser mapper(node_handle);

  //mapping线程
  std::thread publish_map_thread(&graphLaser::publishMapThread, &mapper);
  //发布变换线程
  std::thread publish_tf_thread(&graphLaser::publishTfThread, &mapper);
  //闭环优化线程
  std::thread loopProcess_thread(&graphLaser::loopProcessThread, &mapper);
  //闭环检测线程
  std::thread loopDetector_thread(&graphLaser::loopDetector, &mapper);

  try {
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Exception: " << e.what());
    return 1;
  }
  catch (...)
  {
    ROS_ERROR_STREAM("Unknown Exception");
    return 1;
  }

  publish_map_thread.join();
  publish_tf_thread.join();
  loopProcess_thread.join();
  loopDetector_thread.join();

  return 0;
}
