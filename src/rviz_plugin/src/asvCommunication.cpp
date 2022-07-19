//
// Created by azidanit on 10/07/20.
//

#include "asvCommunication.h"

asvCommunicationPanel::asvCommunicationPanel(QWidget *parent):
        rviz::Panel(parent), ui_(new Ui::asvCommunication()){

    ui_->setupUi(this);

    get_msg_before = false;

    initSubscriber();
    initUi();

    
}

void asvCommunicationPanel::initUi(){
    ui_->steer_bar_left->setTextVisible(false);
    ui_->steer_bar_left->setMaximum(1000);
    ui_->steer_bar_right->setMaximum(1000);
    ui_->throtle_bar->setMaximum(1000);
    ui_->brake_bar->setMaximum(1000);
}

void asvCommunicationPanel::initSubscriber() {
    send_ctrl_subs = nh.subscribe("/asv/cmd_vel", 10, &asvCommunicationPanel::sendControlCallback, this);
//    get_feedback_subs = nh.subscribe()
}

void asvCommunicationPanel::sendControlCallback(const geometry_msgs::Twist& msg) {
    if (msg.angular.z >= 0){
        ui_->steer_bar_left->setValue((int)(msg.angular.z * 1000.0));
        ui_->steer_bar_right->setValue(0);
    }else{
        ui_->steer_bar_right->setValue( -1 *(int)(msg.angular.z * 1000.0));
        ui_->steer_bar_left->setValue(0);
    }

    // std::cout << "STEER RVIZ" << (int)(msg.angular.z * 1000) << std::endl;

    ui_->throtle_bar->setValue((int)(msg.linear.x * 1000));
    ui_->throtle_label->setText(QString(std::to_string(msg.linear.x).c_str()));

    
    ui_->brake_bar->setValue((int)(msg.linear.x * 1000 * -1));
    ui_->brake_label->setText(QString(std::to_string(msg.linear.x).c_str()));

    ui_->steering_label->setText(QString(std::to_string(msg.angular.z).c_str()));

    // background-color: green;\ncolor: white;
    if(get_msg_before){
        ui_->label->setStyleSheet("background-color: green;\ncolor: white;");
        get_msg_before = false;
    }else{
        ui_->label->setStyleSheet("background-color: white;\ncolor: black;");
        get_msg_before = true;
    }
    

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(asvCommunicationPanel,rviz::Panel )