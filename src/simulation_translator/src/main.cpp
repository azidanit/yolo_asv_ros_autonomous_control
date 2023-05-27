#include "ros/ros.h"
#include "std_msgs/String.h"

#include "simulation_translator/GpsSensor.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle nh;
  GpsSensor gps_sensor(nh);
  ros::spin();

  return 0;
}