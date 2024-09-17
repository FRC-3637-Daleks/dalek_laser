#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <ntcore/networktables/NetworkTable.h>
#include <ntcore/networktables/NetworkTableInstance.h>
#include <ntcore/networktables/DoubleTopic.h>
#include <ntcore/networktables/IntegerTopic.h>
#include <ntcore/networktables/StringTopic.h>
#include <ntcore/networktables/BooleanTopic.h>

#include <ros/ros.h>

using namespace std::chrono_literals;

class FmsTable
{
  public:

    FmsTable(const std::string& _node_name, const int& _team_number, const std::string& _hostname, const unsigned int& _port);
    
    std::shared_ptr<nt::NetworkTable> fmsTable;

    nt::StringSubscriber EventName;
    nt::StringSubscriber GameSpecificMessage;
    nt::IntegerSubscriber StationNumber;
    nt::IntegerSubscriber MatchType;
    nt::IntegerSubscriber MatchNumber;
    nt::IntegerSubscriber ReplayNumber;
    nt::IntegerSubscriber FmsControlData;
    nt::BooleanSubscriber IsRedAlliance;

    bool IsConnected();

  private:
    
    nt::NetworkTableInstance inst_;

};
