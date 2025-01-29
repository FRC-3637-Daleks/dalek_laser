#include "DrivetrainInterface/DrivetrainInterfaceNode.h"

#include <wpi/timestamp.h>
#include <ros/time.h>

int main(int argc, char **argv)
{
  //Setup ros node
  std::string node_name = "drivetrain_node";
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  // Configure wpilib to use ros' time
  wpi::SetNowImpl([]() ->  uint64_t {return ros::Time::now().toNSec()/1000;});

  //Create class
  DrivetrainInterfaceNode node(nh, nh_private);

  ros::spin(); // spin() will not return until the node has been shutdown

  return 0;
}