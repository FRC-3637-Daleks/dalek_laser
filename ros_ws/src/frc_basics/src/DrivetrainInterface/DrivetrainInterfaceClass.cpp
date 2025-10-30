#include "DrivetrainInterface/DrivetrainInterfaceNode.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

DrivetrainInterfaceNode::DrivetrainInterfaceNode(const ros::NodeHandle& _node_handle,
              const ros::NodeHandle& _private_node_handle)
: nh_(_node_handle)
, pnh_(_private_node_handle)
, tf_listen_(tf_buffer_)
{
  ROS_INFO_STREAM("Initialize " << ros::this_node::getName());

  // ROS Parameters
  parameters_ = std::make_shared<DrivetrainInterfaceParameters>(nh_, pnh_);

  ROS_INFO_STREAM("  get params");
  parameters_->get_parameters();

  // ROS Publishers and Subscribers
  ROS_INFO_STREAM("  start subscribers");
  subTwist_ = nh_.subscribe<geometry_msgs::Twist>("ros2nt/cmd_vel", 10, std::bind(&DrivetrainInterfaceNode::subTwist_callback, this, std::placeholders::_1));
  subOdom_  = nh_.subscribe<nav_msgs::Odometry>("ros2nt/odom", 10, std::bind(&DrivetrainInterfaceNode::subOdom_callback, this, std::placeholders::_1));

  ROS_INFO_STREAM("  start publishers");
  pubTwist_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("nt2ros/twist", 10);
  pubOdom_  = nh_.advertise<nav_msgs::Odometry>("nt2ros/odom", 10);
  pubSim_   = nh_.advertise<geometry_msgs::PoseStamped>("nt2ros/sim_pose", 10);
  pubImu_   = nh_.advertise<sensor_msgs::Imu>("nt2ros/imu", 10);
  pubMotors_ = nh_.advertise<frc_msgs::Motors>("nt2ros/motors", 10);

  ROS_INFO_STREAM("  start timer");
  pubTimer_ = nh_.createTimer(ros::Duration(0.020), std::bind(&DrivetrainInterfaceNode::pub_callback, this));

  // Network table connection
  table_ = std::make_shared<DrivetrainInterfaceTable>(DrivetrainInterfaceTable(ros::this_node::getName(),parameters_->nt_table, parameters_->team_number, parameters_->hostname, (unsigned int)parameters_->port));
}

void DrivetrainInterfaceNode::subTwist_callback(const geometry_msgs::Twist::ConstPtr& message)
{
  if(table_->IsConnected())
  {
    int64_t timestamp = ros::Time::now().toNSec();
    table_->timestampTwistPub.Set(timestamp / 1000);

    std::vector<double> linear = {message->linear.x, message->linear.y, message->linear.z};
    table_->linearTwistPub.Set(linear);

    std::vector<double> angular = {message->angular.x, message->angular.y, message->angular.z};
    table_->angularTwistPub.Set(angular);

    table_->Flush();
  }
  else
  {
    ROS_WARN_STREAM_THROTTLE(1.0, ros::this_node::getName() << " disconnected from NT server!");
  }
}

void DrivetrainInterfaceNode::subOdom_callback(const nav_msgs::Odometry::ConstPtr&)
{
  if(table_->IsConnected())
  {
    // TODO: Publish current tf state
    /*
    ros::Time stamp = message->header.stamp;
    int64_t timestamp = stamp.toNSec();
    table_->timestampPosePub.Set(timestamp / 1000);

    std::vector<double> linear = {message->pose.pose.position.x, message->pose.pose.position.y, message->pose.pose.position.z};
    table_->linearPosePub.Set(linear);

    std::vector<double> linearVel = {message->twist.twist.linear.x, message->twist.twist.linear.y, message->twist.twist.linear.z};
    table_->linearVelPosePub.Set(linearVel);

    tf2::Quaternion q;
    tf2::fromMsg(message->pose.pose.orientation, q);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    std::vector<double> angular = {roll, pitch, yaw};
    table_->angularPosePub.Set(angular);

    tf2::Quaternion qVel;
    tf2::fromMsg(message->pose.pose.orientation, qVel);
    tf2::Matrix3x3 mVel(qVel);
    double rollVel, pitchVel, yawVel;
    mVel.getRPY(rollVel, pitchVel, yawVel);
    std::vector<double> angularVel = {rollVel, pitchVel, yawVel};
    table_->angularVelPosePub.Set(angularVel);

    table_->Flush();
    */
  }
  else
  {
    ROS_WARN_STREAM_THROTTLE(1.0, ros::this_node::getName() << " disconnected from NT server!");
  }
}

void DrivetrainInterfaceNode::pub_callback()
{
 if(table_->IsConnected())
  {
    // Motors
    if(pubMotors_.getNumSubscribers() > 0)
    {
      frc_msgs::Motors motors_msg;
      auto timestampMotor = table_->timestampMotorSub.GetAtomic();
      ros::Time rostimeMotor;
      rostimeMotor.fromNSec(timestampMotor.time * 1000);
      std::vector<double> voltage     = table_->voltageMotorSub.Get();
      std::vector<double> current     = table_->currentMotorSub.Get();
      std::vector<double> pwmRatio    = table_->pwmRatioMotorSub.Get();
      std::vector<double> temperature = table_->tempMotorSub.Get();
      std::vector<double> angle       = table_->angleMotorSub.Get();
      std::vector<double> rate        = table_->rateMotorSub.Get();
      motors_msg.header.frame_id = "";
      // motors_msg.header.stamp = ros::Time::now();
      motors_msg.header.stamp = rostimeMotor;
      motors_msg.voltage = voltage;
      motors_msg.current = current;
      motors_msg.pwm_ratio = pwmRatio;
      motors_msg.temperature = temperature;
      motors_msg.angle = angle;
      motors_msg.rate = rate;

      pubMotors_.publish(motors_msg);
    }

    // Twist and Odom
    if(pubTwist_.getNumSubscribers() >= 0 || pubOdom_.getNumSubscribers() >= 0)
    {
      //Twist
      auto timestampTwist = table_->timestampOdomSub.GetAtomic();
      ros::Time rostimeTwist;
      rostimeTwist.fromNSec(timestampTwist.time * 1000);

      geometry_msgs::TwistWithCovarianceStamped twist_msg;
      std::vector<double> velLinear = table_->linearVelOdomSub.Get();
      std::vector<double> velAngular = table_->angularVelOdomSub.Get();
      twist_msg.header.frame_id = "base_footprint";
      twist_msg.header.stamp = rostimeTwist;
      twist_msg.twist.twist.linear.x  = velLinear[0];
      twist_msg.twist.twist.linear.y  = velLinear[1];
      twist_msg.twist.twist.linear.z  = velLinear[2];
      twist_msg.twist.twist.angular.x = velAngular[0];
      twist_msg.twist.twist.angular.y = velAngular[1];
      twist_msg.twist.twist.angular.z = velAngular[2];

      twist_msg.twist.covariance = {
          static_cast<double>(parameters_->twist_cov_list[0]), 0., 0., 0., 0., 0.,
          0., static_cast<double>(parameters_->twist_cov_list[1]), 0., 0., 0., 0.,
          0., 0., static_cast<double>(parameters_->twist_cov_list[2]), 0., 0., 0.,
          0., 0., 0., static_cast<double>(parameters_->twist_cov_list[3]), 0., 0.,
          0., 0., 0., 0., static_cast<double>(parameters_->twist_cov_list[4]), 0.,
          0., 0., 0., 0., 0., static_cast<double>(parameters_->twist_cov_list[5]) };

      pubTwist_.publish(twist_msg);
      // Odom
      const auto updates = table_->timestampOdomSub.ReadQueue();
      if (updates.size() != 0) {
        auto timestampOdom = updates.back();
        ros::Time rostimeOdom;
        rostimeOdom.fromNSec(timestampOdom.time*1000);

        nav_msgs::Odometry odom_msg;
        std::vector<double> linear = table_->linearOdomSub.Get();
        std::vector<double> angular = table_->angularOdomSub.Get();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_footprint";
        odom_msg.header.stamp = rostimeOdom;
        odom_msg.twist = twist_msg.twist;
        odom_msg.pose.pose.position.x  = linear[0];
        odom_msg.pose.pose.position.y  = linear[1];
        odom_msg.pose.pose.position.z  = linear[2];

        tf2::Quaternion q;
        q.setRPY(angular[0], angular[1], angular[2]);
        odom_msg.pose.pose.orientation.x = q.getX();
        odom_msg.pose.pose.orientation.y = q.getY();
        odom_msg.pose.pose.orientation.z = q.getZ();
        odom_msg.pose.pose.orientation.w = q.getW();

        odom_msg.pose.covariance = {
          static_cast<double>(parameters_->pose_cov_list[0]), 0., 0., 0., 0., 0.,
          0., static_cast<double>(parameters_->pose_cov_list[1]), 0., 0., 0., 0.,
          0., 0., static_cast<double>(parameters_->pose_cov_list[2]), 0., 0., 0.,
          0., 0., 0., static_cast<double>(parameters_->pose_cov_list[3]), 0., 0.,
          0., 0., 0., 0., static_cast<double>(parameters_->pose_cov_list[4]), 0.,
          0., 0., 0., 0., 0., static_cast<double>(parameters_->pose_cov_list[5]) };

        pubOdom_.publish(odom_msg);

        geometry_msgs::TransformStamped tf_odom_to_base;
        tf_odom_to_base.header.stamp = odom_msg.header.stamp;
        tf_odom_to_base.header.frame_id = "odom";
        tf_odom_to_base.child_frame_id = "base_footprint";
        tf_odom_to_base.transform.translation.x = linear[0];
        tf_odom_to_base.transform.translation.y = linear[1];
        tf_odom_to_base.transform.translation.z = linear[2];
        tf_odom_to_base.transform.rotation.x = q.x();
        tf_odom_to_base.transform.rotation.y = q.y();
        tf_odom_to_base.transform.rotation.z = q.z();
        tf_odom_to_base.transform.rotation.w = q.w();

        tf_br_.sendTransform(tf_odom_to_base);
      }
    }

    if(pubSim_.getNumSubscribers() >= 0)
    {
      auto timestampSim = table_->timestampSimSub.GetAtomic();
      ros::Time rostimeSim;
      rostimeSim.fromNSec(timestampSim.time*1000);

      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header.frame_id = "world";
      pose_msg.header.stamp = rostimeSim;

      std::vector<double> posLinear = table_->simPoseLinearSub.Get();
      std::vector<double> posAngular = table_->simPoseAngularSub.Get();

      pose_msg.pose.position.x = posLinear[0];
      pose_msg.pose.position.y = posLinear[1];
      pose_msg.pose.position.z = posLinear[2];

      tf2::Quaternion q;
      q.setRPY(posAngular[0], posAngular[1], posAngular[2]);

      pose_msg.pose.orientation.x = q.x();
      pose_msg.pose.orientation.y = q.y();
      pose_msg.pose.orientation.z = q.z();
      pose_msg.pose.orientation.w = q.w();

      pubSim_.publish(pose_msg);
    }

    // IMU
    if(pubImu_.getNumSubscribers() > 0)
    {
      auto timestampImu = table_->timestampOdomSub.GetAtomic();
      ros::Time rostimeImu;
      rostimeImu.fromNSec(timestampImu.time * 1000);

      sensor_msgs::Imu imu_msg;
      std::vector<double> angular = table_->angularOdomSub.Get();
      std::vector<double> velAngular = table_->angularVelOdomSub.Get();
      std::vector<double> accLinear  = table_->linearAccOdomSub.Get();
      imu_msg.header.frame_id = "base_footprint";
      imu_msg.header.stamp = rostimeImu;
      imu_msg.angular_velocity.x    = velAngular[0];
      imu_msg.angular_velocity.y    = velAngular[1];
      imu_msg.angular_velocity.z    = velAngular[2];

      imu_msg.angular_velocity_covariance = {
        static_cast<double>(parameters_->twist_cov_list[3]), 0., 0.,
        0., static_cast<double>(parameters_->twist_cov_list[4]), 0.,
        0., 0., static_cast<double>(parameters_->twist_cov_list[5]) };

      imu_msg.linear_acceleration.x = accLinear[0];
      imu_msg.linear_acceleration.y = accLinear[1];
      imu_msg.linear_acceleration.z = accLinear[2];

      imu_msg.linear_acceleration_covariance = {
        static_cast<double>(parameters_->acc_cov_list[0]), 0., 0.,
        0., static_cast<double>(parameters_->acc_cov_list[1]), 0.,
        0., 0., static_cast<double>(parameters_->acc_cov_list[2]) };

      tf2::Quaternion q;
      q.setRPY(angular[0], angular[1], angular[2]);
      imu_msg.orientation.x = q.getX();
      imu_msg.orientation.y = q.getY();
      imu_msg.orientation.z = q.getZ();
      imu_msg.orientation.w = q.getW();

      imu_msg.orientation_covariance = {
        static_cast<double>(parameters_->pose_cov_list[3]), 0., 0.,
        0., static_cast<double>(parameters_->pose_cov_list[4]), 0.,
        0., 0., static_cast<double>(parameters_->pose_cov_list[5]) };

      pubImu_.publish(imu_msg);
    }

    if (tf_buffer_.canTransform("map", "odom", ros::Time(0), ros::Duration(0.01))) {
      const auto map_to_odom = tf_buffer_.lookupTransform("map", "odom", ros::Time(0));
      const auto transform = map_to_odom.transform;
      std::vector<double> correctionLinear(3);
      correctionLinear[0] = transform.translation.x;
      correctionLinear[1] = transform.translation.y;
      correctionLinear[2] = 0;
      table_->linearCorrectionPub.Set(correctionLinear);

      tf2::Quaternion quat;
      tf2::fromMsg(transform.rotation, quat);
      tf2::Matrix3x3 m(quat);

      std::vector<double> correctionAngular(3);
      m.getRPY(correctionAngular[0], correctionAngular[1], correctionAngular[2]);
      table_->angularCorrectionPub.Set(correctionAngular);
    }


  }
}
