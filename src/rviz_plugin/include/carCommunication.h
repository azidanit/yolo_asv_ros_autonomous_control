//
// Created by azidanit on 10/07/20.
//

#ifndef SRC_CARCOMMUNICATION_H
#define SRC_CARCOMMUNICATION_H

#include <rviz/panel.h>
#include "ui_car_communication.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>

namespace Ui{
    class carCommunication;
}

class carCommunicationPanel: public rviz::Panel{
Q_OBJECT

public:
    carCommunicationPanel(QWidget* parent = 0);

public Q_SLOTS:

protected:
    Ui::carCommunication *ui_;

private:
    ros::NodeHandle nh;
    ros::Subscriber send_ctrl_subs, get_feedback_subs;

    void initSubscriber();
    void initUi();
    void sendControlCallback(const geometry_msgs::Twist& msg);
    void getFeedbackCallback(const geometry_msgs::Twist& msg);

};


#endif //SRC_CARCOMMUNICATION_H
