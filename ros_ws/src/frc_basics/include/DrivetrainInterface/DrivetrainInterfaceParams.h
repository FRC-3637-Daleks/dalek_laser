#pragma once

#include <string>
#include <memory>

#include "ros/ros.h"


class DrivetrainInterfaceParameters
{  
  public:
    DrivetrainInterfaceParameters(const ros::NodeHandle& _node_handle, 
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
      pnh_.param<std::string>("nt_table", nt_table, "Drivetrain");

      pnh_.getParam("acc_covariance_diagonal", acc_cov_list);
      ROS_ASSERT(acc_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
      ROS_ASSERT(acc_cov_list.size() == 3);   

      pnh_.getParam("pose_covariance_diagonal", pose_cov_list);
      ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
      ROS_ASSERT(pose_cov_list.size() == 6);   

      pnh_.getParam("twist_covariance_diagonal", twist_cov_list);
      ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
      ROS_ASSERT(twist_cov_list.size() == 6);   

      for (int i = 0; i < twist_cov_list.size(); ++i)
        ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    }

    int team_number;
    int port;
    std::string hostname;
    std::string nt_table;

    XmlRpc::XmlRpcValue acc_cov_list;
    XmlRpc::XmlRpcValue pose_cov_list;
    XmlRpc::XmlRpcValue twist_cov_list;

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

};
