#pragma once

#include <chrono>
#include <functional>
#include <memory>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <frc_msgs/FmsInfo.h>

#include "FmsInfo/FmsInfoTable.h"
#include "FmsInfo/FmsInfoParams.h"


using namespace std::chrono_literals;

class FmsInfoNode
{
  struct HAL_ControlWord {
    uint32_t enabled : 1;
    uint32_t autonomous : 1;
    uint32_t test : 1;
    uint32_t eStop : 1;
    uint32_t fmsAttached : 1;
    uint32_t dsAttached : 1;
    uint32_t control_reserved : 26;
  };
typedef struct HAL_ControlWord HAL_ControlWord;
  
  public:
    FmsInfoNode(const ros::NodeHandle& _node_handle, 
                 const ros::NodeHandle& _private_node_handle);

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    void timer_callback();
    
    ros::Timer timer_;
    ros::Publisher publisher_;

    std::shared_ptr<FmsTable> table_;

    std::shared_ptr<FmsTableParameters> parameters_;
};
