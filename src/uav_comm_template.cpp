#include "uav_comm/uav_comm_template.hpp"
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "sensor_msgs/Imu.h"
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "uav_comm_node");
  ros::NodeHandle n;
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/imu", 5);
  ros::Rate loop (10);
  int count = 0;
  while (ros::ok())
  {
    sensor_msgs::Imu imu_msg;
    imu_msg = empty_imu();
    imu_pub.publish(imu_msg);
    ros::spinOnce;
    loop.sleep();
  }
  ROS_INFO ("test");
  return 0;
}
