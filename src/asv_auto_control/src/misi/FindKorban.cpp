#include "misi/FindKorban.h"

FindKorban::FindKorban(Control *ct_){
    this->ct = ct_;
    
    initSub();
    initVar();

    mission_name = "Find Korban";

    wp_control = new WaypointControl(ct_, this, pid_distance_wp, pid_angle_wp);
    camera_control = new CameraControl(ct_, this, pid_x_cam, pid_y_cam);

    std::cout << "FIND KORBAN CREATED\n";
    
}

FindKorban::~FindKorban(){

}

void FindKorban::initVar(){
    pid_angle_wp = ct->get_pid_angle_wp_find_korban();
    pid_distance_wp = ct->get_pid_distance_wp_find_korban();

    pid_angle_wp->setP(0.1);
    pid_distance_wp->setP(0.15);

    pid_x_cam = ct->get_pid_x_cam_find_korban();
    pid_y_cam = ct->get_pid_y_cam_find_korban();

    pid_x_cam->setP(0.1);
    pid_y_cam->setP(0.15);

    std::cout << "PID " << pid_distance_wp->getD();
    
}

geometry_msgs::Twist FindKorban::calculateOut(){
    geometry_msgs::Twist out;

    if(camera_control->isPersonDetected()){
        out = camera_control->calculateOut();
        out.linear.z = 1; //feedback control Camera
        std::cout << "CONTROLLING CAMERA\n";
    }else{
        out = wp_control->calculateOut();
        out.linear.x = ct->speedControlCalculate(0.5);
        out.linear.z = 2; //feedback control WP

        std::cout << "CONTROLLING WP\n";


    }   
    
    return out;
}


void FindKorban::initSub(){
    path_sub = ct->nh.subscribe("/waypoints", 1, &FindKorban::wpCallback, this);

}


void FindKorban::wpCallback(rviz_plugin::Selectedwp msg_wp){
    if (msg_wp.missionSelected == 1){
        std::cout << "FIND KORBAN GOT WP\n";
        wp_control->setPath(msg_wp.wp);

    }
}

void FindKorban::stop(){
    wp_control->stopAndReset();
}

