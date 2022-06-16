//
// Created by azidanit on 5/30/22.
//

#include "Control.h"

Control::Control(){
    // kapal belok kiri Z = POSITIF
    // kapal belok kanan Z = NEGATIF

    //init tf var
    tfListener = new tf2_ros::TransformListener(tfBuffer);

    
    // misisons[0] = new FindKorban(this);
    find_korban = new FindKorban(this);
    
    
    initVar();
    initSub();
    initPub();
}

Control::~Control(){

}

void Control::run(){
    ros::Rate rr(50);
    while (ros::ok())
    {
        
        listenASVTF();

        if(mission_state.data[0] == 1){
            //start mission
            out_cmd = find_korban->calculateOut();
            std::cout << out_cmd << std::endl;
        }else if(mission_state.data[0] == 0){
            //stop mission
            find_korban->stop();
        }
        
        if(mission_state.data[0] != 0){
            out_cmd.angular.z += steer_trim;
            asv_cmd_vel_pub.publish(out_cmd);
        }
        
        rr.sleep();
        ros::spinOnce();
    }
    
}

void Control::initVar(){
    mission_state.data.push_back(0);
    mission_state.data.push_back(0);

    use_speed_control = false;
    target_constant_thrust = 0;
    steer_trim = 0;
    // find_korban->setPIDWpAngle(&pid_angle_wp_find_korban);
    // find_korban->setPIDWpDistance(&pid_distance_wp_find_korban);
}

void Control::initSub(){

    gps_raw_sub = nh.subscribe("/mavros/global_position/global",2,&Control::GPSRawCallback, this);

    mission_state_control_sub = nh.subscribe("/rviz_plugin/stateMission",2,&Control::stateMissionCallback, this);

}

void Control::initPub(){
    asv_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/asv/cmd_vel",1);

    path_pub = nh.advertise<nav_msgs::Path>("/path_asv", 1);
    global_path_pub = nh.advertise<nav_msgs::Path>("/path_asv_global", 1);
    loaded_path_pub = nh.advertise<nav_msgs::Path>("/path_asv_loaded", 1);
}

void Control::GPSRawCallback(sensor_msgs::NavSatFix data_gps){
    current_gps = data_gps;
}

void Control::listenASVTF(){
    double roll, pitch, yaw;

    try{
        transformStamped = tfBuffer.lookupTransform("map", "asv/base_link",
                               ros::Time(0));
        ASV_TF.pos.y = transformStamped.transform.translation.y;
        ASV_TF.pos.x = transformStamped.transform.translation.x;
        ASV_TF.pos.z = 0;

        tf2::Quaternion q(
            transformStamped.transform.rotation.x,
            transformStamped.transform.rotation.y,
            transformStamped.transform.rotation.z,
            transformStamped.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(ASV_TF.roll, ASV_TF.pitch, ASV_TF.yaw);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
    //   ros::Duration(1.0).sleep();
    }

    // std::cout << "ASV TF COTNROL " << ASV_TF.pos.x << " " << ASV_TF.pos.y << " YAW " << ASV_TF.yaw << std::endl;

}

TF_simplified Control::getRobotTf(){
    return ASV_TF;
}

void Control::startMission() {

}

void Control::stopMission() {

}

void Control::resumeMission() {

}

void Control::changeSteerTrim(int val) {
    param_qt_mtx.lock();
    steer_trim = (float)val/1000;
    param_qt_mtx.unlock();
}

void Control::changeSpeedTrim(int val) {
    param_qt_mtx.lock();
    // thrust_trim = val;
    param_qt_mtx.unlock();
}

void Control::useSpeedControl(bool val) {
    param_qt_mtx.lock();
    use_speed_control = val;
    param_qt_mtx.unlock();
}

void Control::useAccError(bool) {

}

void Control::changeUseObstacleAvoidance(bool val) {
    param_qt_mtx.lock();
    // use_obstacle_avoidance = val;
    param_qt_mtx.unlock();
}

void Control::changeUseObstacleStop(bool val) {
    param_qt_mtx.lock();
    // use_obstacle_stop = val;
    param_qt_mtx.unlock();
}

void Control::changeP(int idx, double val) {
    std::cout << "CHANGED P IN CONTROL " << idx << " " << val << std::endl;
    param_qt_mtx.lock();
    if(idx == 0){
        pid_angle_wp_find_korban.setP(val);
    }
    else if(idx == 1){
        pid_distance_wp_find_korban.setP(val);
    }
    else if(idx == 2){
        pid_speed_control.setP(val);
    }
    //else if(idx == 3){
    //     pid_obstacle_avoidance_srf.p = val;
    // }else if(idx == 4){
    //     pid_camera_tracking.p = val;
    // }else if(idx == 5){
    //     pid_camera_tracking_thrust.p = val;
    // }
    param_qt_mtx.unlock();
}

void Control::changeI(int idx, double val) {
    param_qt_mtx.lock();
    if(idx == 0){
        pid_angle_wp_find_korban.setI(val);
    }
    else if(idx == 1){
        pid_distance_wp_find_korban.setI(val);
    }
    else if(idx == 2){
        pid_speed_control.setI(val);
    }
    // }else if(idx == 3){
    //     pid_obstacle_avoidance_srf.i = val;
    // }else if(idx == 4){
    //     pid_camera_tracking.i = val;
    // }else if(idx == 5){
    //     pid_camera_tracking_thrust.i = val;
    // }
    param_qt_mtx.unlock();
}

void Control::changeD(int idx, double val) {
    param_qt_mtx.lock();
    if(idx == 0){
        pid_angle_wp_find_korban.setD(val);
    }
    else if(idx == 1){
        pid_distance_wp_find_korban.setD(val);
    }
    else if(idx == 2){
        pid_speed_control.setD(val);
    }
    // }else if(idx == 3){
    //     pid_obstacle_avoidance_srf.d = val;
    // }else if(idx == 4){
    //     pid_camera_tracking.d = val;
    // }else if(idx == 5){
    //     pid_camera_tracking_thrust.d = val;
    // }
    param_qt_mtx.unlock();
}

void Control::changeSpeed(int idx, int val) {
    // target_constant_thrust = val;
}

void Control::changeSpeedDouble(int idx, double val) {
    target_constant_thrust = val;
}

void Control::changeDistance(int idx, double val) {
    param_qt_mtx.lock();
    // save_front_distance = val;
    param_qt_mtx.unlock();
}

void Control::changeASVWidth(double) {

}

void Control::changeASVOffset(double) {

}

void Control::changeRTHAfterDone(bool val) {
    std::cout << "RTH " << val << "\n";
    // is_rth_after_done = val;
    // rth_called = false;
}

void Control::changePatrolLap(int val) {
    param_qt_mtx.lock();
    // patrol_lap = val;
    param_qt_mtx.unlock();
    std::cout << "PATROL LAP TO " << val << "\n";
}

void Control::calibrateCompass(double val){
 
}


void Control::changeUseCompass(bool val) {
    // std_msgs::UInt8 msg;
    // msg.data = (int)val;
    // using_compass_pub.publish(msg);
}

void Control::changeUseFindKorban(bool val) {
    param_qt_mtx.lock();
    // use_find_korban = val;
    param_qt_mtx.unlock();
    // std::cout << "FIND KORBAN " << use_find_korban << "\n";
}

void Control::pitchChanged(int idx, double val) {
    // sensor_control.setPitchThreshold(val);
}


void Control::startRecordPath(){
    // std::cout << "Start record path" << std::endl;
    if (is_path_recorded){
        // std::cout << "is recording... " << std::endl;
        return;
    }

    path_mtx.lock();
    is_path_recorded = true;
    path_mtx.unlock();
    path_thread = std::thread(&Control::recordPath, this);
}

void Control::stopRecordPath(){
    // std::cout << "Stop record path" << std::endl;
    if (!is_path_recorded){
        return;
    }

    path_mtx.lock();
    is_path_recorded = false;
    path_mtx.unlock();
    path_thread.join();
    std::cout << "Stopped record path" << std::endl;
}

void Control::recordPath(){
    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped global_pose;
    pose.header.frame_id = "map";
    path_msg.header.frame_id = "map";
    is_path_recorded = true;
    // printf("masuk path record\n");

    while(is_path_recorded){
        // printf("masuk path record\n");
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = transformStamped.transform.translation.x;
        pose.pose.position.y = transformStamped.transform.translation.y;
        pose.pose.position.z = transformStamped.transform.translation.z;
        pose.pose.orientation.x = transformStamped.transform.rotation.x;
        pose.pose.orientation.y = transformStamped.transform.rotation.y;
        pose.pose.orientation.z = transformStamped.transform.rotation.z;
        pose.pose.orientation.w = transformStamped.transform.rotation.w;

        path_msg.header.stamp = ros::Time::now();
        path_msg.poses.push_back(pose);
        path_pub.publish(path_msg);

        // Record GPS
        if(current_gps.latitude != 0.0000 && current_gps.longitude != 0.0000){
            global_path.push_back(current_gps);

            global_pose = globalToPose(current_gps.latitude, current_gps.longitude);
            global_pose.header.stamp = ros::Time::now();
            // global_pose.pose.position.z = 0;
            global_pose.pose.orientation.x = currentBoatTF.getRotation().getX();
            global_pose.pose.orientation.y = currentBoatTF.getRotation().getY();
            global_pose.pose.orientation.z = currentBoatTF.getRotation().getZ();
            global_pose.pose.orientation.w = currentBoatTF.getRotation().getW();

            global_path_msg.header.stamp = ros::Time::now();
            global_path_msg.poses.push_back(global_pose);
            global_path_pub.publish(path_msg);
        }
        usleep(200000);
    }
    std::cout << "Stopped record path" << std::endl;
}

void Control::clearPath(){
    std::cout << "clearing path " << path_msg.poses.size() << std::endl;
    path_mtx.lock();
    path_msg.poses.clear();
    global_path_msg.poses.clear();
    loaded_path_msg.poses.clear();
    path_mtx.unlock();

    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();
    path_pub.publish(path_msg);

    loaded_path_msg.header.frame_id = "map";
    loaded_path_msg.header.stamp = ros::Time::now();
    loaded_path_pub.publish(loaded_path_msg);

    global_path_msg.header.frame_id = "map";
    global_path_msg.header.stamp = ros::Time::now();
    global_path_pub.publish(global_path_msg);

    global_path.clear();

    std::cout << "cleared path " << global_path.size() << std::endl;
}

void Control::savePath(QString filename){
    std::cout << "save path to " << filename.toStdString() << std::endl;
    if (global_path.size()==0){
        return;
    }
    std::cout << "save path" << std::endl;
    sensor_msgs::NavSatFix path_to_save;
    QFile file(filename);

    if (file.open(QIODevice::ReadWrite | QIODevice::Truncate)) {
        QTextStream stream(&file);
        char str[1024];
        auto pit=path_msg.poses.begin();
        for (auto it=++(global_path.begin()); it != global_path.end(); ++it){
            memset(str, 0, sizeof(str));
            sprintf(str, "%.8f,%.8f,%.8f,%.8f,%.8f,%.8f",
                    it->latitude, it->longitude,
                    pit->pose.orientation.w, pit->pose.orientation.x, pit->pose.orientation.y, pit->pose.orientation.z);
            stream << str << endl;
            std::cout << str << std::endl;
            ++pit;
        }
        file.close();
    }

    // save local path
    QString filename_local = filename + QString("local");
    QFile file_loc(filename_local);

    if (file_loc.open(QIODevice::ReadWrite | QIODevice::Truncate)) {
        QTextStream stream(&file_loc);
        char str[1024];
        for (auto pit=++(path_msg.poses.begin()); pit != path_msg.poses.end(); ++pit){
            memset(str, 0, sizeof(str));
            sprintf(str, "%.8f,%.8f",
                    pit->pose.position.x, pit->pose.position.y);
            stream << str << endl;
            std::cout << str << std::endl;
        }
        file_loc.close();
    }
}

void Control::loadPath(QString filename){
    std::cout << "load path" << std::endl;
    geometry_msgs::PoseStamped pose;
    double lat, lon;

    loaded_path_msg.poses.clear();
    loaded_path_msg.header.frame_id = "map";

    QFile file(filename);

    if(file.open(QIODevice::ReadWrite)) {

        QTextStream in(&file);

        while(!in.atEnd()) {
            QString line = in.readLine();
            std::cout << line.toStdString() << std::endl;
            QStringList fields = line.split(",");
            lat = fields[0].toDouble();
            lon = fields[1].toDouble();
            pose = globalToPose(lat, lon);
            pose.pose.orientation.w = fields[2].toDouble();
            pose.pose.orientation.x = fields[3].toDouble();
            pose.pose.orientation.y = fields[4].toDouble();
            pose.pose.orientation.z = fields[5].toDouble();
            pose.header.stamp = ros::Time::now();
            // std::cout << LongitudeCenter << "***" << LatitudeCenter <<  std::endl;
            loaded_path_msg.poses.push_back(pose);
        }
        file.close();

        loaded_path_msg.header.stamp = ros::Time::now();
        loaded_path_pub.publish(loaded_path_msg);

    }
}


void Control::loadLocalPath(QString filename){
    std::cout << "load path" << std::endl;
    geometry_msgs::PoseStamped pose;
    double lat, lon;

    loaded_path_msg.poses.clear();
    loaded_path_msg.header.frame_id = "map";

    QFile file(filename);

    if(file.open(QIODevice::ReadWrite)) {

        QTextStream in(&file);

        while(!in.atEnd()) {
            QString line = in.readLine();
            // std::cout << line.toStdString() << std::endl;
            QStringList fields = line.split(",");
            pose.pose.position.x = fields[0].toDouble();
            pose.pose.position.y = fields[1].toDouble();
            pose.pose.position.z = 0.5;
            pose.pose.orientation.w = 0;
            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = 0;
            pose.header.stamp = ros::Time::now();
            loaded_path_msg.poses.push_back(pose);
        }
        file.close();

        loaded_path_msg.header.stamp = ros::Time::now();
        loaded_path_pub.publish(loaded_path_msg);

    }
}

inline geometry_msgs::PoseStamped Control::globalToPose(double lat, double lon){
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";

    pose.pose.position.x = LON_TO_METER * (LongitudeCenter - lon);
    pose.pose.position.y = LAT_TO_METER * (LatitudeCenter - lat);
    pose.pose.position.z = .5;

    return pose;
}

void Control::changeCritLine(int i, int cl) {
    param_qt_mtx.lock();
    // if (i) crit_line = cl;
    // else horizon = cl;
    param_qt_mtx.unlock();

    // sensor_control.setCurrentCritLineCamera(cl);
    // std::cout << "CRIT LINE " << crit_line << " " << horizon << "\n";
}

void Control::changeBoatSide(QPoint **) {

}

void Control::stateMissionCallback(std_msgs::UInt16MultiArray msgl){
    mission_state_mtx.lock();
    mission_state = msgl;
    mission_state_mtx.unlock();


}

PIDController* Control::get_pid_angle_wp_find_korban(){
    return &pid_angle_wp_find_korban;
}

PIDController* Control::get_pid_distance_wp_find_korban(){
    return &pid_distance_wp_find_korban;
}

double Control::speedControlCalculate(double target){
    if (use_speed_control)
        return out_cmd.linear.x + pid_speed_control.updateError(target - current_asv_speed);
    else
        return target_constant_thrust;
}

double Control::getCurrentASVSpeed(){
    return current_asv_speed;
}