
#pragma once

// Ros includes
#include <ros/ros.h>

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
  ros::NodeHandle priv_nh_;
  ros::NodeHandle public_nh_;

  ros::Subscriber ratethrust_sub_;
  ros::Subscriber actuator_sub_;
  ros::Subscriber rpyrt_sub_;

  ros::Publisher status_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher rc_pub_;

  // Callback functions for subscribed messages
  void callback_roll_pitch_yawrate_thrust(
      const mav_msgs::RollPitchYawrateThrustConstPtr& msg);

  void callback_rollrate_pitchrate_yawrate_thrust(
      const mav_msgs::RateThrustConstPtr& msg);

  void callback_actuator_commanded(const mav_msgs::ActuatorsConstPtr& msg);

public:

  uav_communication(ros::NodeHandle &pub_nh, ros::NodeHandle &priv_nh);

  void ros_loop();

};
