#include "DrivetrainInterface/DrivetrainInterfaceNode.h"

int main(int argc, char **argv)
{
  //Setup ros node
  std::string node_name = "drivetrain_node";
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  //Create class
  DrivetrainInterfaceNode node(nh, nh_private);

  ros::spin(); // spin() will not return until the node has been shutdown

  return 0;
}