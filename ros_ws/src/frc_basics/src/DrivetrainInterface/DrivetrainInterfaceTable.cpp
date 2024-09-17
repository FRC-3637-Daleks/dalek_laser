#include "DrivetrainInterface/DrivetrainInterfaceTable.h"


DrivetrainInterfaceTable::DrivetrainInterfaceTable(const std::string& _node_name, const std::string& _nt_table, const int& _team_number, const std::string& _hostname, const unsigned int& _port)
{
  inst_ = nt::NetworkTableInstance::GetDefault();

  if(_hostname != "" )
    // ROS_INFO_STREAM("Setting up DrivetrainInterface NT subscriber by hostname.");
    // ROS_INFO_STREAM("  Hostname: " << *_hostname);
    // ROS_INFO_STREAM("  Port: " << *_port); 

    inst_.SetServer(_hostname.c_str(), _port);
  else
    // ROS_INFO_STREAM("Setting up DrivetrainInterface NT subscriber by team number.");
    // ROS_INFO_STREAM("  Team: " << *_team_number);
    // ROS_INFO_STREAM("  Port: " << *_port);

    inst_.SetServerTeam(_team_number, _port);  // where TEAM=190, 294, etc, or use inst.startClient("hostname") or similar

  inst_.StartClient4(_node_name);

  // ROS_INFO_STREAM("  Finding table " << _nt_table);
  drivetrainTable = inst_.GetTable(_nt_table);
  
  // Odom Pub
  timestampOdomPub = drivetrainTable->GetIntegerTopic("ros2nt/odom/timestamp").Publish();
  linearOdomPub    = drivetrainTable->GetDoubleArrayTopic("ros2nt/odom/position/linear").Publish();
  angularOdomPub   = drivetrainTable->GetDoubleArrayTopic("ros2nt/odom/position/angular").Publish();
  linearVelOdomPub = drivetrainTable->GetDoubleArrayTopic("ros2nt/odom/velocity/linear").Publish();
  angularVelOdomPub= drivetrainTable->GetDoubleArrayTopic("ros2nt/odom/velocity/angular").Publish();

  // Twist Pub
  timestampTwistPub= drivetrainTable->GetIntegerTopic("ros2nt/twist/timestamp").Publish();  
  linearTwistPub   = drivetrainTable->GetDoubleArrayTopic("ros2nt/twist/velocity/linear").Publish();
  angularTwistPub  = drivetrainTable->GetDoubleArrayTopic("ros2nt/twist/velocity/angular").Publish();

  // Odom, Twist, IMU Sub
  std::vector<double> defaultDoubleValue = {0.0, 0.0, 0.0};
  timestampOdomSub = drivetrainTable->GetIntegerTopic("nt2ros/odom/timestamp").Subscribe(0);  
  linearOdomSub    = drivetrainTable->GetDoubleArrayTopic("nt2ros/odom/position/linear").Subscribe(defaultDoubleValue);
  angularOdomSub   = drivetrainTable->GetDoubleArrayTopic("nt2ros/odom/position/angular").Subscribe(defaultDoubleValue);
  linearVelOdomSub = drivetrainTable->GetDoubleArrayTopic("nt2ros/odom/velocity/linear").Subscribe(defaultDoubleValue);
  angularVelOdomSub= drivetrainTable->GetDoubleArrayTopic("nt2ros/odom/velocity/angular").Subscribe(defaultDoubleValue);
  linearAccOdomSub = drivetrainTable->GetDoubleArrayTopic("nt2ros/odom/acceleration/linear").Subscribe(defaultDoubleValue);

  // Motor Sub
  std::vector<int64_t> defaultIntValue = {0};
  timestampMotorSub= drivetrainTable->GetIntegerTopic("nt2ros/motors/timestamp").Subscribe(0);    
  voltageMotorSub  = drivetrainTable->GetDoubleArrayTopic("nt2ros/motors/voltage").Subscribe(defaultDoubleValue);
  currentMotorSub  = drivetrainTable->GetDoubleArrayTopic("nt2ros/motors/current").Subscribe(defaultDoubleValue);
  pwmRatioMotorSub = drivetrainTable->GetDoubleArrayTopic("nt2ros/motors/pwm_ratio").Subscribe(defaultDoubleValue);
  tempMotorSub     = drivetrainTable->GetDoubleArrayTopic("nt2ros/motors/temperature").Subscribe(defaultDoubleValue);
  angleMotorSub    = drivetrainTable->GetDoubleArrayTopic("nt2ros/motors/angle").Subscribe(defaultDoubleValue);
  rateMotorSub     = drivetrainTable->GetDoubleArrayTopic("nt2ros/motors/rate").Subscribe(defaultDoubleValue);

  ROS_INFO_STREAM("  Connected!"); 
}

bool DrivetrainInterfaceTable::IsConnected()
{
  return inst_.IsConnected();
}

void DrivetrainInterfaceTable::Flush()
{
  inst_.Flush();
}

