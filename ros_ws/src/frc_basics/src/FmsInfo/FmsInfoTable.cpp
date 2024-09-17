#include "FmsInfo/FmsInfoTable.h"


FmsTable::FmsTable(const std::string& _node_name, const int& _team_number, const std::string& _hostname, const unsigned int& _port)
{
  inst_ = nt::NetworkTableInstance::GetDefault();

  if(_hostname != "" )
  {
    // ROS_INFO_STREAM("Setting up FmsInfo NT subscriber by hostname.");
    // ROS_INFO_STREAM("  Hostname: " << _hostname);
    // ROS_INFO_STREAM("  Port: " << _port); 
    
    inst_.SetServer(_hostname.c_str(), _port);
  }
  else
  {
    // ROS_INFO_STREAM("Setting up FmsInfo NT subscriber by team number.");
    // ROS_INFO_STREAM("  Team: " << _team_number);
    // ROS_INFO_STREAM("  Port: " << _port); 

    inst_.SetServerTeam(_team_number, _port);  // where TEAM=190, 294, etc, or use inst.startClient("hostname") or similar
  }

  inst_.StartClient4(_node_name);

  ROS_WARN_STREAM("  Finding table FMSInfo");
  fmsTable = inst_.GetTable("FMSInfo");
  
  EventName = fmsTable->GetStringTopic("EventName").Subscribe("");
  GameSpecificMessage = fmsTable->GetStringTopic("GameSpecificMessage").Subscribe("");
  StationNumber = fmsTable->GetIntegerTopic("StationNumber").Subscribe(0.0);
  MatchType = fmsTable->GetIntegerTopic("MatchType").Subscribe(0.0);
  MatchNumber = fmsTable->GetIntegerTopic("MatchNumber").Subscribe(0.0);
  ReplayNumber = fmsTable->GetIntegerTopic("ReplayNumber").Subscribe(0.0);
  FmsControlData = fmsTable->GetIntegerTopic("FMSControlData").Subscribe(0.0);
  IsRedAlliance = fmsTable->GetBooleanTopic("IsRedAlliance").Subscribe(false);

  ROS_INFO_STREAM("  Connected!");  
}

bool FmsTable::IsConnected()
{
  return inst_.IsConnected();
}

