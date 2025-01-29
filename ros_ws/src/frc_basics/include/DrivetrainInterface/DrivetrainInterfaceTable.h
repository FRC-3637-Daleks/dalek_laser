#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <ntcore/networktables/NetworkTable.h>
#include <ntcore/networktables/NetworkTableInstance.h>
#include <ntcore/networktables/IntegerTopic.h>
#include <ntcore/networktables/DoubleArrayTopic.h>
#include <ntcore/networktables/IntegerArrayTopic.h>

#include <ros/ros.h>

using namespace std::chrono_literals;

class DrivetrainInterfaceTable
{
  public:

    DrivetrainInterfaceTable(const std::string& _node_name, const std::string& _nt_table, const int& _team_number, const std::string& _hostname, const unsigned int& _port);

    std::shared_ptr<nt::NetworkTable> drivetrainTable;

    nt::IntegerPublisher timestampPosePub;
    nt::DoubleArrayPublisher linearPosePub;
    nt::DoubleArrayPublisher angularPosePub;
    nt::DoubleArrayPublisher linearCorrectionPub;
    nt::DoubleArrayPublisher angularCorrectionPub;

    nt::IntegerPublisher timestampTwistPub;
    nt::DoubleArrayPublisher linearTwistPub;
    nt::DoubleArrayPublisher angularTwistPub;

    nt::IntegerSubscriber timestampOdomSub;
    nt::DoubleArraySubscriber linearOdomSub;
    nt::DoubleArraySubscriber angularOdomSub;
    nt::DoubleArraySubscriber linearVelOdomSub;
    nt::DoubleArraySubscriber angularVelOdomSub;
    nt::DoubleArraySubscriber linearAccOdomSub;

    nt::IntegerSubscriber timestampMotorSub;
    nt::DoubleArraySubscriber voltageMotorSub;
    nt::DoubleArraySubscriber currentMotorSub;
    nt::DoubleArraySubscriber pwmRatioMotorSub;
    nt::DoubleArraySubscriber tempMotorSub;
    nt::DoubleArraySubscriber angleMotorSub;
    nt::DoubleArraySubscriber rateMotorSub;

    nt::IntegerSubscriber timestampSimSub;
    nt::DoubleArraySubscriber simPoseLinearSub;
    nt::DoubleArraySubscriber simPoseAngularSub;


    bool IsConnected();

    void Flush();

  private:

    nt::NetworkTableInstance inst_;

};
