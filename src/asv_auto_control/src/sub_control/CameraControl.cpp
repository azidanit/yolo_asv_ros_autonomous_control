#include "sub_control/CameraControl.h"

CameraControl::CameraControl(Control* ct_, Misi* ms_, PIDController* pid_x_, PIDController* pid_y_){
    this->ct = ct_;
    this->pid_x = pid_x_;
    this->pid_y = pid_y_;

    is_person_detected = false;

    confident_threshold = 10;

    initSub();
}


CameraControl::~CameraControl(){

}

void CameraControl::initSub(){
    critline_sub = ct->nh.subscribe("/rviz_plugin/camera/critline",2,&CameraControl::critLineCallback, this);
    obj_vision_sub = ct->nh.subscribe("/vision/objects",2,&CameraControl::personDetectionCallback, this);
}

bool CameraControl::isPersonDetected(){
    is_person_detected = obj_person_detected.boxes.size();
    return is_person_detected;
}

geometry_msgs::Twist CameraControl::calculateOut(){
    geometry_msgs::Twist out_cmd;

    double error_cam_x = 0.5 - obj_person_detected.boxes[0].center.x;
    double error_cam_y = crit_line - obj_person_detected.boxes[0].center.y;

    out_cmd.angular.z = pid_x->updateError(error_cam_x);
    out_cmd.linear.x = pid_y->updateError(error_cam_y);        

    return out_cmd;

}


void CameraControl::critLineCallback(std_msgs::Int32MultiArray msg){
    if(msg.data.size() > 1){
        crit_line = msg.data[0];
        horizon = msg.data[1];

        std::cout << "SET CRITLINE AND HORIZON\n";
    }
}

void CameraControl::personDetectionCallback(vision_msgs::BoundingBox2DArray msg){
    obj_person_detected = msg;
}

