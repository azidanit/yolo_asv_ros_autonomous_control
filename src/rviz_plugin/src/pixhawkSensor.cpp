//
// Created by azidanit on 09/07/20.
//

#include "pixhawkSensor.h"

pixhawkSensorPanel::pixhawkSensorPanel(QWidget *parent):
    rviz::Panel(parent), ui_(new Ui::pixhawkSensor()){

    ui_->setupUi(this);
    initSubscriber();
}

void pixhawkSensorPanel::initSubscriber() {
    subs_diagnotsic = nh_.subscribe("/diagnostics", 4, &pixhawkSensorPanel::diagnosticCallback, this);
    subs_compass = nh_.subscribe("/mavros/global_position/compass_hdg", 4, &pixhawkSensorPanel::compassCallback, this);
    subs_gps_vel = nh_.subscribe("/mavros/global_position/raw/gps_vel", 4, &pixhawkSensorPanel::gpsVelCallback, this);
    subs_local_vel = nh_.subscribe("/mavros/local_position/velocity_body", 4, &pixhawkSensorPanel::localVelCallback, this);
    subs_mavconn = nh_.subscribe("/mavros/conn", 4, &pixhawkSensorPanel::mavconnCallback, this);
}

void pixhawkSensorPanel::gpsVelCallback(const geometry_msgs::TwistStamped &msg) {
    double y_vec = double(msg.twist.linear.y);
    double x_vec = double(msg.twist.linear.x);
    double velocity_resultant = sqrt(pow(y_vec,2) + pow(x_vec,2));
    ui_->speed_gps_label->setText(QString(std::to_string(velocity_resultant).c_str()));
}

void pixhawkSensorPanel::localVelCallback(const geometry_msgs::TwistStamped &msg) {
    double y_vec = double(msg.twist.linear.y);
    double x_vec = double(msg.twist.linear.x);
    double velocity_resultant = sqrt(pow(y_vec,2) + pow(x_vec,2));
    ui_->speed_local_label->setText(QString(std::to_string(velocity_resultant).c_str()));
}

void pixhawkSensorPanel::compassCallback(const std_msgs::Float64& msg){
    ui_->compass_label->setText(QString(std::to_string(msg.data).c_str()));
}

void pixhawkSensorPanel::diagnosticCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg_arr){
    for (auto a : msg_arr->status){
        if (strcmp(a.name.c_str(), "mavros: GPS") == 0){
            for (auto i : a.values){
                if (strcmp(i.key.c_str(), "Satellites visible")==0)
                    ui_->gps_count_label->setText(i.value.c_str());
//                    std::cout << "DATA SATALLITE " << i.value << std::endl;
                if (strcmp(i.key.c_str(), "Fix type")==0)
                    ui_->gps_status_label->setText(i.value.c_str());
//                    std::cout << "DATA fix type " << i.value << std::endl;
                if (strcmp(i.key.c_str(), "EPH (m)")==0){
                    char tmp_kk[22];
                    strcpy(tmp_kk, i.value.c_str());
                    strcat(tmp_kk, " m");
                    ui_->gps_accuracy_label->setText(tmp_kk);
                }
//                    std::cout << "DATA EPH " << i.value << std::endl;
//                std::cout << i.key << i.value <<std::endl;
            }
        }

        if (strcmp(a.name.c_str(), "mavros: System") == 0){
            for (auto i : a.values){
                if (strcmp(i.key.c_str(), "3D magnetometer")==0)
                    ui_->magnetometer_label->setText(i.value.c_str());
//                    std::cout << "DATA 3D magnetometer " << i.value << std::endl;
                if (strcmp(i.key.c_str(), "AHRS subsystem health")==0)
                    ui_->ahrs_label->setText(i.value.c_str());
//                    std::cout << "DATA AHRS subsystem health " << i.value << std::endl;
                if (strcmp(i.key.c_str(), "CPU Load (%)")==0){
                    char tmp_cpu[22];
                    strcpy(tmp_cpu, i.value.c_str());
                    strcat(tmp_cpu, " %");
                    ui_->cpu_load_label->setText(tmp_cpu);
//                    std::cout << "DATA CPU Load " << i.value << std::endl;
                }
                if (strcmp(i.key.c_str(), "3D gyro")==0)
                    ui_->gyro_label->setText(i.value.c_str());
//                    std::cout << "DATA 3D gyro " << i.value << std::endl;
//                std::cout << i.key << i.value <<std::endl;
            }
        }

    }
}

void pixhawkSensorPanel::mavconnCallback(const std_msgs::Int8& msg){
    if(msg.data==1){
        ui_->connectionStatus->setText("Connected");
        ui_->connectionStatus->setStyleSheet("QLabel { color : green; }");
    }
    else{
        ui_->connectionStatus->setText("Disconnected");
        ui_->connectionStatus->setStyleSheet("QLabel { color : red; }");

    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pixhawkSensorPanel,rviz::Panel )
