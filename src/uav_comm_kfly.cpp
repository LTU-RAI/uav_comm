
// User includes
#include "uav_comm/uav_comm_kfly.hpp"

//
// Callback functions for KFly subscribed messages
//
void uav_communication::kfly_status(kfly_comm::datagrams::SystemStatus msg)
{
  mav_msgs::Status out;

  out.vehicle_name = vehicle_name_;
  out.vehicle_name = vehicle_name_;

  out.battery_voltage           = msg.battery_voltage;
  out.command_interface_enabled = msg.serial_interface_enabled;
  out.flight_time               = msg.flight_time;
  out.system_uptime             = msg.up_time;
  out.cpu_load                  = msg.cpu_usage;
  out.in_air                    = msg.in_air;

  if (msg.motors_armed)
    out.motor_status = mav_msgs::Status::MOTOR_STATUS_RUNNING;
  else
    out.motor_status = mav_msgs::Status::MOTOR_STATUS_STOPPED;

  status_pub_.publish(out);
}

void uav_communication::kfly_strings(kfly_comm::datagrams::SystemStrings msg)
{
  vehicle_name_ = msg.vehicle_name;
  vehicle_type_ = msg.vehicle_type;
  ROS_INFO("KFly Version: %s", msg.kfly_version);
}

void uav_communication::kfly_imu(kfly_comm::datagrams::IMUData msg)
{
  sensor_msgs::Imu out;

  out.orientation_covariance[0] = -1;
  out.orientation.w             = 1;

  out.angular_velocity.x    = msg.gyroscope[0];
  out.angular_velocity.y    = msg.gyroscope[1];
  out.angular_velocity.z    = msg.gyroscope[2];
  out.linear_acceleration.x = msg.accelerometer[0];
  out.linear_acceleration.y = msg.accelerometer[1];
  out.linear_acceleration.z = msg.accelerometer[2];

  out.angular_velocity_covariance[0]    = 0.00016900;
  out.angular_velocity_covariance[4]    = 0.00016900;
  out.angular_velocity_covariance[8]    = 0.00016900;
  out.linear_acceleration_covariance[0] = 0.00688900;
  out.linear_acceleration_covariance[4] = 0.00688900;
  out.linear_acceleration_covariance[8] = 0.00688900;

  imu_pub_.publish(out);
}

//
// Callback functions for ROS subscribed messages
//
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
  // ROS
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

  // KFly
  kfly_comm_.register_callback(this, &uav_communication::kfly_status);
  kfly_comm_.register_callback(this, &uav_communication::kfly_strings);
  kfly_comm_.register_callback(this, &uav_communication::kfly_imu);

  // Generate KFly subscriptions

  //_communication->send(codec::generate_command(commands::GetSystemStrings));
  //_communication->subscribe(kfly_comm::commands::GetSystemStatus, 100);
  //_communication->subscribe(kfly_comm::commands::GetIMUData, 10);
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
