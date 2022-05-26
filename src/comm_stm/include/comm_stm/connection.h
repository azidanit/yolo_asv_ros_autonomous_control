#ifndef CONNECTION_H
#define CONNECTION_H
#include <comm_stm/drive_system.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <string>
#include <algorithm>
#include <comm_stm/stm_status.h>
class Connection
{
protected:
  int motorL;
  int motorR;
  int servoL;
  int servoR;
  int tekinL;
  int tekinR;

  ros::NodeHandle node;
  ros::Subscriber sub_drive_system;
  ros::Publisher pub_stm_data;
  ros::Publisher pub_stm_state, pub_stm_conn;
  ros::Publisher pub_remote_auto_state;


public:
  Connection(){};
  ~Connection(){};
  virtual void start() = 0;
};

#endif