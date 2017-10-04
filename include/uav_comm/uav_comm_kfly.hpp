
#pragma once

// Ros includes
#include <ros/ros.h>

// KFly
#include <kfly_comm/kfly_comm.hpp>

// Serial
#include <serialpipe.h>

// Messages
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/RateThrust.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <mav_msgs/Status.h>
#include <mav_msgs/Actuators.h>

class uav_communication
{
private:
  // KFly
  kfly_comm::codec kfly_comm_;
  std::string vehicle_name_;
  std::string vehicle_type_;

  // Callback functions for KFly subscribed messages
  void kfly_status(kfly_comm::datagrams::SystemStatus msg);
  void kfly_strings(kfly_comm::datagrams::SystemStrings msg);
  void kfly_imu(kfly_comm::datagrams::IMUData msg);

  // ROS
  ros::NodeHandle priv_nh_;
  ros::NodeHandle public_nh_;

  ros::Subscriber ratethrust_sub_;
  ros::Subscriber actuator_sub_;
  ros::Subscriber rpyrt_sub_;

  ros::Publisher status_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher rc_pub_;

  // Callback functions for ROS subscribed messages
  void callback_roll_pitch_yawrate_thrust(
      const mav_msgs::RollPitchYawrateThrustConstPtr& msg);

  void callback_rollrate_pitchrate_yawrate_thrust(
      const mav_msgs::RateThrustConstPtr& msg);

  void callback_actuator_commanded(const mav_msgs::ActuatorsConstPtr& msg);

public:

  uav_communication(ros::NodeHandle &pub_nh, ros::NodeHandle &priv_nh);

  void ros_loop();

};
