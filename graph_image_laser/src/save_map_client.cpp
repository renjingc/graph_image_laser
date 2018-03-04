#include "ros/ros.h"
#include "graph_image_laser/SaveMap.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "save_map_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<graph_image_laser::SaveMap>("/graph_image_laser/save_map");
  graph_image_laser::SaveMap srv;
  srv.request.filename.data="/home/ren/knowmeking";
  if (client.call(srv))
  {
    ROS_INFO("save_map succ");
  }
  else
  {
    ROS_ERROR("Failed to call save_map");
    return 1;
  }

  return 0;
}
