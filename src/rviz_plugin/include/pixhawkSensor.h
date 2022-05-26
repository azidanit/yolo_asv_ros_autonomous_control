//
// Created by azidanit on 09/07/20.
//

#ifndef RVIZ_PLUGIN_PIXHAWKSENSOR_H
#define RVIZ_PLUGIN_PIXHAWKSENSOR_H

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/Imu.h>
#include <QString>
#include <qstring.h>
#include <iostream>
#include <string.h>
#include <stdlib.h>

#include <rviz/panel.h>
#include "ui_pixhawkSensor.h"

namespace Ui{
    class pixhawkSensor;
}

class pixhawkSensorPanel: public rviz::Panel{
    Q_OBJECT

public:
    pixhawkSensorPanel(QWidget* parent = 0);

public Q_SLOTS:

protected:
    Ui::pixhawkSensor *ui_;

private:
    void initSubscriber();
    void mavconnCallback(const std_msgs::Int8& msg);
    void diagnosticCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg_arr);
    void gpsVelCallback(const geometry_msgs::TwistStamped& msg);
    void localVelCallback(const geometry_msgs::TwistStamped& msg);
    void compassCallback(const std_msgs::Float64& msg);
    ros::NodeHandle nh_;
    ros::Subscriber subs_diagnotsic, subs_gps_vel, subs_local_vel, subs_compass, subs_mavconn;
};


#endif //RVIZ_PLUGIN_PIXHAWKSENSOR_H
