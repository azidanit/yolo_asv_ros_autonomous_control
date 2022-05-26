//
// Created by azidanit on 10/07/20.
//

#include "carCommunication.h"

carCommunicationPanel::carCommunicationPanel(QWidget *parent):
        rviz::Panel(parent), ui_(new Ui::carCommunication()){

    ui_->setupUi(this);
    initSubscriber();
    initUi();
}

void carCommunicationPanel::initUi(){
    ui_->steer_bar_left->setTextVisible(false);
    ui_->steer_bar_left->setMaximum(500);
    ui_->steer_bar_right->setMaximum(500);
    ui_->throtle_bar->setMaximum(500);
    ui_->brake_bar->setMaximum(500);
}

void carCommunicationPanel::initSubscriber() {
    send_ctrl_subs = nh.subscribe("/data_to_stm", 10, &carCommunicationPanel::sendControlCallback, this);
//    get_feedback_subs = nh.subscribe()
}

void carCommunicationPanel::sendControlCallback(const geometry_msgs::Twist& msg) {
    if (msg.angular.z <= 0){
        ui_->steer_bar_left->setValue((int)-1*msg.angular.z);
        ui_->steer_bar_right->setValue(0);
    }else{
        ui_->steer_bar_right->setValue( (int)(msg.angular.z));
        ui_->steer_bar_left->setValue(0);
    }


    ui_->throtle_bar->setValue((int)msg.linear.y);
    ui_->throtle_label->setText(QString(std::to_string((int)msg.linear.y).c_str()));

    ui_->brake_bar->setValue(int(msg.linear.z));
    ui_->brake_label->setText(QString(std::to_string((int)msg.linear.z).c_str()));

    ui_->steering_label->setText(QString(std::to_string((int)msg.angular.z).c_str()));
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(carCommunicationPanel,rviz::Panel )