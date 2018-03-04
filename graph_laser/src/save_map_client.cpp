#include "ros/ros.h"
#include "graph_laser/SaveMap.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "save_map_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<graph_laser::SaveMap>("/graph_laser/save_map");
  graph_laser::SaveMap srv;
  srv.request.filename.data="/media/ren/99146341-07be-4601-9682-0539688db03f/mypcd4";

  std::cout<<"save_map service call"<<std::endl;
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
