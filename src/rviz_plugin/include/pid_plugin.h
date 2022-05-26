#pragma once

# include <ros/ros.h>
# include <rviz/panel.h>
#include "rviz_plugin/Pid.h"

enum{
    SPEED       = 's',
    SUDUT       = 'a',
    P           = 'p',
    I           = 'i',
    D           = 'd',
    TIME        = 'm',
    TRIM        = 't',
    WAYPOINT    = 'w'
} VAR;

namespace Ui{
    class Pid;
}

class PIDPanel: public rviz::Panel{
        
Q_OBJECT
public:
    PIDPanel(QWidget* parent = 0);

    // virtual void load( const rviz::Config& config );
    // virtual void save( rviz::Config con


protected:
    Ui::Pid *ui;
    // The ROS node handle.
    ros::NodeHandle nh;
    ros::Publisher pid_publisher;
    rviz_plugin::Pid msg;

    void initPublisher();
    void initConnection();
    void connectTab0();
    void connectM1();
    void connectM2();
    void connectM3();
    void connectM4();
    void connectM5();
    void connectM6();
    void connectM7();
    void publish(const int misi_idx, const char var, const int index, const int value);
};
