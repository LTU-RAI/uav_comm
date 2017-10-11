
// User includes
#include "uav_comm/uav_comm_kfly.hpp"
#include <thread>

//
// Callback functions for KFly subscribed messages
//
void uav_communication::kfly_status(kfly_comm::datagrams::SystemStatus msg)
{
  mav_msgs::Status out;

  out.header.stamp = ros::Time::now();

  out.vehicle_name = vehicle_name_;
  out.vehicle_type = vehicle_type_;

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
  ROS_INFO("     Name:    %s", msg.vehicle_name);
  ROS_INFO("     Type:    %s", msg.vehicle_type);
}

void uav_communication::kfly_imu(kfly_comm::datagrams::IMUData msg)
{
  sensor_msgs::Imu out;

  out.header.stamp    = ros::Time::now();
  out.header.frame_id = "body";

  out.orientation_covariance[0] = -1;
  out.orientation.w             = 1;
  out.orientation.x             = 0;
  out.orientation.y             = 0;
  out.orientation.z             = 0;

  out.angular_velocity.x    = msg.gyroscope[0];
  out.angular_velocity.y    = msg.gyroscope[1];
  out.angular_velocity.z    = msg.gyroscope[2];
  out.linear_acceleration.x = msg.accelerometer[0] * gravity_;
  out.linear_acceleration.y = msg.accelerometer[1] * gravity_;
  out.linear_acceleration.z = msg.accelerometer[2] * gravity_;

  out.angular_velocity_covariance[0]    = 0.00016900;
  out.angular_velocity_covariance[4]    = 0.00016900;
  out.angular_velocity_covariance[8]    = 0.00016900;
  out.linear_acceleration_covariance[0] = 0.00688900;
  out.linear_acceleration_covariance[4] = 0.00688900;
  out.linear_acceleration_covariance[8] = 0.00688900;

  imu_pub_.publish(out);
}

void uav_communication::kfly_raw_imu(kfly_comm::datagrams::RawIMUData msg)
{
  sensor_msgs::Imu out;

  out.header.stamp    = ros::Time::now();
  out.header.frame_id = "body";

  out.orientation_covariance[0] = -1;
  out.orientation.w             = 1;
  out.orientation.x             = 0;
  out.orientation.y             = 0;
  out.orientation.z             = 0;

  out.angular_velocity.x    = msg.gyroscope[0];
  out.angular_velocity.y    = msg.gyroscope[1];
  out.angular_velocity.z    = msg.gyroscope[2];
  out.linear_acceleration.x = msg.accelerometer[0];
  out.linear_acceleration.y = msg.accelerometer[1];
  out.linear_acceleration.z = msg.accelerometer[2];

  out.angular_velocity_covariance[0]    = 10;
  out.angular_velocity_covariance[4]    = 10;
  out.angular_velocity_covariance[8]    = 10;
  out.linear_acceleration_covariance[0] = 78;
  out.linear_acceleration_covariance[4] = 78;
  out.linear_acceleration_covariance[8] = 78;

  raw_imu_pub_.publish(out);
}

//
// Callback functions for ROS subscribed messages
//
void uav_communication::callback_roll_pitch_yawrate_thrust(
    const mav_msgs::RollPitchYawrateThrustConstPtr& msg)
{
  ROS_INFO("Got Roll Pitch Yawrate Thrust command");

  kfly_comm::datagrams::ComputerControlReference ref;

  ref.mode = kfly_comm::enums::FlightMode::ATTITUDE_EULER_MODE;

  if (msg->thrust.z > 1 || msg->thrust.z < 0)
  {
    ROS_ERROR(
        "Got roll_pitch_yawrate_thrust command with throttle outside of "
        "bounds, "
        "KFly accepts only normalized thrust on [0 .. 1].");

    return;
  }

}

void uav_communication::callback_rollrate_pitchrate_yawrate_thrust(
    const mav_msgs::RateThrustConstPtr& msg)
{
  ROS_INFO("Got Rate Thrust command");

  kfly_comm::datagrams::ComputerControlReference ref;

  ref.mode = kfly_comm::enums::FlightMode::RATE_MODE;

  if (msg->thrust.z > 1 || msg->thrust.z < 0)
  {
    ROS_ERROR(
        "Got rate_thrust command with throttle outside of bounds, "
        "KFly accepts only normalized thrust on [0 .. 1].");

    return;
  }

}

void uav_communication::callback_actuator_commanded(
    const mav_msgs::ActuatorsConstPtr& msg)
{
  ROS_INFO("Got Actuator command");

  if (msg->angles.size() > 0)
    ROS_ERROR(
        "Got Actuator command with \"angles\" defined, which is not "
        "supported.");

  if (msg->angular_velocities.size() > 0)
    ROS_ERROR(
        "Got Actuator command with \"angular_velocities\" defined, which is "
        "not "
        "supported.");

  if (msg->normalized.size() >= 9)
    ROS_ERROR(
        "Got Actuator command with \"normalized\" larger than 8, which is not "
        "supported.");

  bool valid_command = true;
  for (auto val : msg->normalized)
    valid_command = valid_command && !(val > 1 || val < 0);

  if (!valid_command)
  {
    ROS_ERROR(
        "Got Actuator command with \"normalized\" outside of bounds, which is "
        "not supported. Normalized command must be within [0 .. 1]");

    // Commands must be valid, abort!
    return;
  }

  // Fill the KFly data structure for direct motor commands
  kfly_comm::datagrams::ComputerControlReference ref;

  // Limit to max 8 motors
  auto size = msg->normalized.size();
  if (size > 8)
    size = 8;

  // Set correct mode for this topic
  ref.mode = kfly_comm::enums::FlightMode::MOTOR_DIRECT_MODE;

  // Fill the motor commands to the data structure
  for (auto i             = 0; i < size; i++)
    ref.direct_control[i] = msg->normalized[i] * 65535;  // KFly internal format

  // Send to KFly
  serial_->serialTransmit(kfly_comm::codec::generate_packet(ref));
}

uav_communication::uav_communication(ros::NodeHandle& pub_nh,
                                     ros::NodeHandle& priv_nh)
    : public_nh_(pub_nh), priv_nh_(priv_nh)
{
  // Get parameters
  std::string port;
  int baudrate;
  double imu_rate;
  bool publish_raw_imu;

  if (!priv_nh_.getParam("port", port))
  {
    ROS_ERROR("No port specified, aborting!");
    return;
  }
  ROS_INFO("KFly port:     %s", port.c_str());

  if (!priv_nh_.getParam("baudrate", baudrate))
  {
    ROS_ERROR("No baudrate specified, aborting!");
    return;
  }
  ROS_INFO("KFly baudrate: %d", baudrate);
  if (baudrate < 9600)
  {
    ROS_ERROR("The baudrate must be 9600 or more, aborting!");
    return;
  }

  priv_nh_.param("imu_hz", imu_rate, 100.0);
  if (imu_rate > 200)
  {
    ROS_WARN("The IMU sampling rate is too high, reducing to 200 Hz.");
    imu_rate = 200;
  }
  else if (imu_rate < 10)
  {
    ROS_WARN("The IMU sampling rate is too low, increasing to 10 Hz.");
    imu_rate = 10;
  }

  if (!priv_nh_.getParam("gravity_constant", gravity_))
  {
    ROS_WARN("No gravitational constant specified, assuming 9.81 m/s^2.");
    return;
  }

  priv_nh_.param("publish_raw_imu", publish_raw_imu, false);
  if (!publish_raw_imu)
  {
    ROS_INFO(
        "NOT publishing raw IMU measurements, set \"publish_raw_imu\" to true "
        "to enable.");
  }

  // Start the serial port
  serial_ = std::unique_ptr< SerialPipe::SerialBridge >(
      new SerialPipe::SerialBridge(port, baudrate, 1, false));

  // Allow data to start flowing by registering the parsing of serial data
  serial_->registerCallback(
      [&](const std::vector< uint8_t >& data) { kfly_comm_.parse(data); });

  // Open port
  try
  {
    serial_->openPort();
  }
  catch (serial::IOException& e)
  {
    ROS_ERROR("Port not found, aborting!");
  }

  if (!serial_->isOpen())
  {
    ROS_ERROR("Port not open, aborting!");
    return;
  }

  // Just for safety, unsubscribe from everything
  serial_->serialTransmit(kfly_comm::codec::generate_unsubscribe_all());
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

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
  serial_->serialTransmit(kfly_comm::codec::generate_command(
      kfly_comm::commands::GetSystemStrings));
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  serial_->serialTransmit(kfly_comm::codec::generate_subscribe(
      kfly_comm::commands::GetSystemStatus, 100));
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  serial_->serialTransmit(kfly_comm::codec::generate_subscribe(
      kfly_comm::commands::GetIMUData, 1000.0 / imu_rate + 0.5));
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  if (publish_raw_imu)
  {
    // Generate raw IMU subscription if requested
    raw_imu_pub_ = public_nh_.advertise< sensor_msgs::Imu >(
        std::string(mav_msgs::default_topics::IMU) + "_raw", 1);

    kfly_comm_.register_callback(this, &uav_communication::kfly_raw_imu);

    serial_->serialTransmit(kfly_comm::codec::generate_subscribe(
        kfly_comm::commands::GetRawIMUData, 1000.0 / imu_rate + 0.5));
  }

  // All ok
  ROS_INFO("Started KFly communication on port \"%s\"", port.c_str());
}

void uav_communication::ros_loop()
{
  ros::spin();
}
