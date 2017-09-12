
// User includes
#include "uav_comm/uav_comm_template.hpp"

// Callback functions for subscribed messages
void uav_communication::callback_roll_pitch_yawrate_thrust(
    const mav_msgs::RollPitchYawrateThrustConstPtr& msg)
{
  ROS_INFO("Got Roll Pitch Yawrate Thrust command");
}

void uav_communication::callback_rollrate_pitchrate_yawrate_thrust(
    const mav_msgs::RateThrustConstPtr& msg)
{
  ROS_INFO("Got Rate Thrust command");
}

void uav_communication::callback_actuator_commanded(
    const mav_msgs::ActuatorsConstPtr& msg)
{
  ROS_INFO("Got Actuator command");
}


uav_communication::uav_communication(ros::NodeHandle& pub_nh,
                                     ros::NodeHandle& priv_nh)
    : public_nh_(pub_nh), priv_nh_(priv_nh)
{
  ratethrust_sub_ = public_nh_.subscribe(
      mav_msgs::default_topics::COMMAND_RATE_THRUST, 5,
      &uav_communication::callback_rollrate_pitchrate_yawrate_thrust, this,
      ros::TransportHints().tcpNoDelay());

  actuator_sub_ =
      public_nh_.subscribe(mav_msgs::default_topics::COMMAND_ACTUATORS, 5,
                           &uav_communication::callback_actuator_commanded,
                           this, ros::TransportHints().tcpNoDelay());

  rpyrt_sub_ = public_nh_.subscribe(
      mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 5,
      &uav_communication::callback_roll_pitch_yawrate_thrust, this,
      ros::TransportHints().tcpNoDelay());

  status_pub_ = public_nh_.advertise< mav_msgs::Status >(
      mav_msgs::default_topics::STATUS, 1);

  imu_pub_ = public_nh_.advertise< sensor_msgs::Imu >(
      mav_msgs::default_topics::IMU, 1);

  rc_pub_ =
      public_nh_.advertise< sensor_msgs::Joy >(mav_msgs::default_topics::RC, 1);
}

void uav_communication::ros_loop()
{
  ros::Rate loop(10);

  while (ros::ok())
  {
    sensor_msgs::Imu imu_msg;
    mav_msgs::Status status_msg;

    status_pub_.publish(status_msg);
    imu_pub_.publish(imu_msg);

    ros::spinOnce();
    loop.sleep();
  }
}

