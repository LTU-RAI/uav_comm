#include "uav_comm/uav_comm_kfly.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uav_comm_node");

  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  uav_communication comm(n, pn);

  comm.ros_loop();

  return 0;
}
