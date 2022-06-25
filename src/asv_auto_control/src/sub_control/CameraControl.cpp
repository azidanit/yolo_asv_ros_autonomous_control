#include "sub_control/CameraControl.h"

CameraControl::CameraControl(Control* ct_, Misi* ms_, PIDController* pid_x_, PIDController* pid_y_){
    this->ct = ct_;
    this->pid_x = pid_x_;
    this->pid_y = pid_y_;

    is_person_detected = false;

    confident_threshold = 10;
    confident_counter = 0;

    tracking_counter = 0;
    tracking_threshold = 30 * 8;

    try_rotate_in_position = 1;

    crit_line = 0.5;

    initSub();
}


CameraControl::~CameraControl(){

}

void CameraControl::initSub(){
    critline_sub = ct->nh.subscribe("/rviz_plugin/camera/critline",2,&CameraControl::critLineCallback, this);
    obj_vision_sub = ct->nh.subscribe("/vision/objects",2,&CameraControl::personDetectionCallback, this);
}

bool CameraControl::isPersonDetected(){
    return is_person_detected;
}

geometry_msgs::Twist CameraControl::calculateOut(){
    geometry_msgs::Twist out_cmd;

    if(obj_person_detected.boxes.size()){
        double error_cam_x = 0.5 - obj_person_detected.boxes[0].center.x;
        double error_cam_y = crit_line - obj_person_detected.boxes[0].center.y;

        out_cmd.angular.z = pid_x->updateError(error_cam_x);
        out_cmd.linear.x = pid_y->updateError(error_cam_y); 

        tracking_counter++;   
    }else if(confident_counter > -confident_threshold){//sebelumnya ada korban
        out_cmd.angular.z = 0;
        out_cmd.linear.x = 0;
        std::cout << "MENUNGGU KORBBAN GLITC " << "\n";

        

    }else if(try_rotate_in_position && tracking_counter > tracking_threshold){ //try rotate to find korban
        out_cmd.angular.z = 0.3;
        out_cmd.linear.x = 0;
        if(start_angle == NOT_CONDUCTED)
            start_angle = ct->getRobotTf().yaw;
        if(start_angle > ct->getRobotTf().yaw  && ct->getRobotTf().yaw > (start_angle - (M_PI/18)) ){
            try_rotate_in_position--;
        }
        // try_rotate_in_position--;
        std::cout << "ROTATING UNTIL " << start_angle << "\n";
    }else{

    }


    return out_cmd;

}


void CameraControl::critLineCallback(std_msgs::Int32MultiArray msg){
    if(msg.data.size() > 1){
        crit_line = msg.data[0] / 480.0;
        horizon = msg.data[1] / 480.0;

        std::cout << "SET CRITLINE AND HORIZON\n";
    }
}

void CameraControl::personDetectionCallback(vision_msgs::BoundingBox2DArray msg){
    obj_person_detected = msg;
    std::cout << "CONFIDENT COUNTER " << confident_counter << "\n";
    if(obj_person_detected.boxes.size()){
        last_obj_person_detected = obj_person_detected;
        if(confident_counter > confident_threshold){
            is_person_detected = true;
            try_rotate_in_position = 1;
            start_angle = NOT_CONDUCTED;
        }
        else{
            if(confident_counter <= confident_threshold)
                confident_counter++;
        }


    }else{
        if(confident_counter < -confident_threshold && (!try_rotate_in_position || tracking_counter < tracking_threshold)){
            is_person_detected = false;
            tracking_counter = 0;
        }else{
            if(confident_counter >= -confident_threshold)
                confident_counter--;
        }
    }
}

