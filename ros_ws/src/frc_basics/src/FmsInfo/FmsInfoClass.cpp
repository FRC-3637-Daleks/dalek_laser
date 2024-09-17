#include "FmsInfo/FmsInfoNode.h"

FmsInfoNode::FmsInfoNode(const ros::NodeHandle& _node_handle, 
                           const ros::NodeHandle& _private_node_handle)
: nh_(_node_handle)
, pnh_(_private_node_handle)
{
  ROS_INFO_STREAM("Initialize " << ros::this_node::getName());

  // ROS Parameters
  parameters_ = std::make_shared<FmsTableParameters>(nh_, pnh_);

  ROS_INFO_STREAM("  get params");
  parameters_->get_parameters();
  
  // Publishers and Subscribers
  publisher_ = pnh_.advertise<frc_msgs::FmsInfo>("FmsInfo", 10);
  timer_ = nh_.createTimer(ros::Duration(0.020), std::bind(&FmsInfoNode::timer_callback, this));

  // Network table connection
  table_ = std::make_shared<FmsTable>(FmsTable(ros::this_node::getName(),parameters_->team_number, parameters_->hostname, (unsigned int)parameters_->port));
}

void FmsInfoNode::timer_callback()
{
  if(table_->IsConnected())
  {
    frc_msgs::FmsInfo message;
    message.header.stamp = ros::Time::now();
    message.event_name = table_->EventName.Get();
    message.game_specific_message = table_->GameSpecificMessage.Get();
    message.is_red_alliance = table_->IsRedAlliance.Get();
    message.station_number = (int)(table_->StationNumber.Get());
    message.match_type = (int)(table_->MatchType.Get());
    message.match_number = (int)(table_->MatchNumber.Get());
    message.replay_number = (int)(table_->ReplayNumber.Get());

    HAL_ControlWord fms_control_data;
    std::memset(&fms_control_data, table_->FmsControlData.Get(), sizeof(fms_control_data));
    message.enabled = fms_control_data.enabled;
    message.estopped = fms_control_data.eStop;
    message.autonomous = fms_control_data.autonomous;
    message.ds_attached= fms_control_data.dsAttached;
    message.fms_attached = fms_control_data.fmsAttached;
    message.control_reserved = fms_control_data.control_reserved;

    publisher_.publish(message);
  }
  else
  {
    ROS_WARN_STREAM_THROTTLE(1.0, ros::this_node::getName() << " disconnected from NT server!");
  }
}
