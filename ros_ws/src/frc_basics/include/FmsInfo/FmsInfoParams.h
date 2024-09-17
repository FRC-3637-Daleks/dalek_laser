#pragma once

#include <string>
#include <memory>

#include "ros/ros.h"

class FmsTableParameters
{  
  public:
    FmsTableParameters(const ros::NodeHandle& _node_handle, 
                       const ros::NodeHandle& _private_node_handle)
    : nh_(_node_handle)
    , pnh_(_private_node_handle)
    {

    }

    void get_parameters()
    {
      //Static Parameters
      pnh_.param<int>("team_number", team_number, 456);
      pnh_.param<std::string>("hostname", hostname, "");
      pnh_.param<int>("port", port, 0);
    }

    int team_number;
    int port;
    std::string hostname;

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

};
