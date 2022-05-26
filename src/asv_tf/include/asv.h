//
// Created by azidanit on 5/26/22.
//

#ifndef SRC_ASV_H
#define SRC_ASV_H

#include "ros/ros.h"
#include <ros/package.h>
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <iostream>
#include "math.h"
#include "vector"


#define LAT_TO_METER 111000
#define LON_TO_METER 113321

class ASV {
private:
    ros::NodeHandle nh;
    ros::Subscriber gps_sub, compass_sub, mag_sub, imu_sub;

    void loadFileParam(char* filename_);
    void globalToLocal(double lat_, double long_);

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void compassCallback(std_msgs::Float64 msg);
    void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    double lat_center, long_center;
    double lat_top, lat_bottom, long_left, long_right;
    double local_x, local_y, local_yaw, local_pitch, local_roll;

public:
    ASV(char* venue_name);

    void update();

};


#endif //SRC_ASV_H
