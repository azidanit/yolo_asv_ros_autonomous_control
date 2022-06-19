//
// Created by azidanit on 10/07/20.
//

#ifndef SRC_ASVCOMMUNICATION_H
#define SRC_ASVCOMMUNICATION_H

#include <rviz/panel.h>
#include "ui_asv_communication.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>

namespace Ui{
    class asvCommunication;
}

class asvCommunicationPanel: public rviz::Panel{
Q_OBJECT

public:
    asvCommunicationPanel(QWidget* parent = 0);

public Q_SLOTS:

protected:
    Ui::asvCommunication *ui_;

private:
    ros::NodeHandle nh;
    ros::Subscriber send_ctrl_subs, get_feedback_subs;

    bool get_msg_before;

    void initSubscriber();
    void initUi();
    void sendControlCallback(const geometry_msgs::Twist& msg);
    void getFeedbackCallback(const geometry_msgs::Twist& msg);

};


#endif //SRC_ASVCOMMUNICATION_H
