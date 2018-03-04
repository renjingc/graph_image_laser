#include <thread>

#include <ros/ros.h>

#include "graph_image_laser/graph_image_laser.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "LaserMapper");
  ros::NodeHandle node_handle("~");

  graphImageLaser mapper(node_handle);

  //mapping线程
  std::thread publish_map_thread(&graphImageLaser::publishMapThread, &mapper);
  //发布变换线程
  std::thread publish_tf_thread(&graphImageLaser::publishTfThread, &mapper);
  //闭环处理线程
  std::thread loopProcess_thread(&graphImageLaser::loopProcessThread, &mapper);
  //词包检测线程
  std::thread dLoopDetector_thread(&graphImageLaser::dLoopDetector, &mapper);

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
  dLoopDetector_thread.join();

  return 0;
}
