#pragma once

#include <chrono>
#include <functional>
#include <memory>

#include "DrivetrainInterface/DrivetrainInterfaceTable.h"
#include "DrivetrainInterface/DrivetrainInterfaceParams.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "frc_msgs/Motors.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "tf2_ros/transform_broadcaster.h"


using namespace std::chrono_literals;

class DrivetrainInterfaceNode
{
  public:
    DrivetrainInterfaceNode(const ros::NodeHandle& _node_handle,
                 const ros::NodeHandle& _private_node_handle);

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    void subTwist_callback(const geometry_msgs::Twist::ConstPtr& message);
    void subOdom_callback(const nav_msgs::Odometry::ConstPtr& message);
    void pub_callback();

    ros::Subscriber subTwist_;
    ros::Subscriber subOdom_;

    ros::Timer pubTimer_;
    ros::Publisher pubTwist_;
    ros::Publisher pubOdom_;
    ros::Publisher pubImu_;
    ros::Publisher pubMotors_;

    tf2_ros::TransformBroadcaster tf_br_;

    std::shared_ptr<DrivetrainInterfaceTable> table_;

    std::shared_ptr<DrivetrainInterfaceParameters> parameters_;
};
