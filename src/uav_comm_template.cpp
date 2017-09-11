// System includes
#include <iostream>
#include <sstream>
#include <boost/lexical_cast.hpp>

// Ros includes
#include <ros/ros.h>
#include <ros/transport_hints.h>
#include <ros/subscribe_options.h>

// Messages
#include "std_msgs/String.h"

#include "sensor_msgs/Imu.h"
#include "mav_msgs/RateThrust.h"
#include "mav_msgs/RollPitchYawrateThrust.h"
#include "mav_msgs/Status.h"
#include "mav_msgs/Actuators.h"

// User includes
#include "uav_comm/uav_comm_template.hpp"

using namespace std;

sensor_msgs::Imu empty_imu()
{
  sensor_msgs::Imu msg;
  msg.header.stamp = ros::Time::now();

  msg.orientation.w = 1;
  msg.orientation.x = 0;
  msg.orientation.y = 0;
  msg.orientation.z = 0;

  for(int i=0; i<9;++i)
    msg.orientation_covariance[i]=0;

  msg.angular_velocity.x = 0;
  msg.angular_velocity.y = 0;
  msg.angular_velocity.z = 0;

  for(int i=0; i<9;++i)
    msg.angular_velocity_covariance[i]=0;

  msg.linear_acceleration.x = 0;
  msg.linear_acceleration.y = 0;
  msg.linear_acceleration.z = 0;

  for(int i=0; i<9;++i)
    msg.linear_acceleration_covariance[i]=0;

  return msg;
}

void callback_RPY(const mav_msgs::RollPitchYawrateThrustConstPtr& msg)
{
}

void callback_RT(const mav_msgs::RateThrustConstPtr& msg)
{
}

void callback_A(const mav_msgs::ActuatorsConstPtr& msg)
{
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "uav_comm_node");
  ros::NodeHandle n;




  ros::Subscriber RateT_sub = n.subscribe("/RateT",10,callback_RT, ros::TransportHints().tcpNoDelay());
  ros::Subscriber Act_sub = n.subscribe("/Act",10,callback_A, ros::TransportHints().tcpNoDelay());
  ros::Subscriber RPYrT_sub = n.subscribe("/RPYrT",10,callback_RPY, ros::TransportHints().tcpNoDelay());
  ros::Publisher status_pub = n.advertise<mav_msgs::Status>("/status", 5);
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/imu", 5);

  ros::Rate loop (10);
  int count = 0;
  while (ros::ok())
  {
    sensor_msgs::Imu imu_msg;
    mav_msgs::Status status_msg;
    imu_msg = empty_imu();
    status_pub.publish(status_msg);
    imu_pub.publish(imu_msg);
    ros::spinOnce;
    loop.sleep();
  }

  return 0;
}
