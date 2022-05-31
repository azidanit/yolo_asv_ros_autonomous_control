//
// Created by azidanit on 7/23/21.
//
#include "SensorControl.h"

SensorControl::SensorControl(){
    sma_element = 20;

    for(int i = 0; i < sizeof(srf_data_raw)/sizeof(srf_data_raw[0]); i++){
        srf_data_raw[i] = srf_data[i] = 0;
    }

    initSubscriber();

    run_thread = std::thread(&SensorControl::run, this);

    choosen_channel_to_pass = -1;
    error_camera = distance_est_korban_camera = -9999;
    critline = 240 ;
}

SensorControl::~SensorControl() {

}

void SensorControl::run() {
    ros::Rate rr(100);
    while(ros::ok()){
        ros::spinOnce();
//        calculateSRFError();
//        calculateCameraError();
        rr.sleep();

    }
}

void SensorControl::initSubscriber() {
    srf_sub = nh_.subscribe("/sensors/srf/data", 10, &SensorControl::srfDataCallback, this);
    korban_camera_sub = nh_.subscribe("/object_image/data/detection", 10, &SensorControl::korbanDataCameraCallback, this);
}

void SensorControl::calculateCameraError() {
    if(korban_camera_sub.getNumPublishers() == 0){
        std::cout << "DETECTION MATI\n";
    }

    if(detection_objets.detections.size()<=0 || korban_camera_sub.getNumPublishers() == 0){
        error_camera = -9999;
        distance_est_korban_camera = -9999;
    }else{
        int error_x = detection_objets.detections[0].bbox.center.x - 320;
        error_camera = error_x;

        int y_max = detection_objets.detections[0].bbox.center.y +
                (detection_objets.detections[0].bbox.size_y / 2);
        distance_est_korban_camera = critline - y_max;
    }
    std::cout << "CALCULATE CAMERA " <<error_camera << "\n";

}

double SensorControl::korbanDistanceEstimationCamera() {
    calculateCameraError();
    return distance_est_korban_camera;
}

double SensorControl::korbanDistanceSRF(){
    double distance = 9999;

    for (auto i : srf_data){
        if(i < distance)
            distance = i;
    }

    return distance;
}

bool SensorControl::isThereIsObstableSRF() {
    short number_of_srf = sizeof(srf_data)/sizeof(srf_data[0]);

    for(int i = 0; i < number_of_srf; i++){
        if (srf_data[i] <= save_distance_srf)
            direction_free_obs[i] = false;
        else
            direction_free_obs[i] = true;
    }

    return direction_free_obs[whichSRFChannelASVLead()];
}

void SensorControl::calculateSRFError() {
    short number_of_srf = sizeof(srf_data)/sizeof(srf_data[0]);

    short kanan_free_obs_channel = -1, kiri_free_obs_channel = -1;
    short channel_asv_lead = whichSRFChannelASVLead();

    if(!direction_free_obs[channel_asv_lead]){
        //ke kanan
        for(int q = channel_asv_lead+1; q < number_of_srf; q++){
            if(direction_free_obs[q]){
                kanan_free_obs_channel = q;
                break;
            }
        }

        //ke kiri
        for(int p = channel_asv_lead-1; p >= 0; p--){
            if(direction_free_obs[p]){
                kiri_free_obs_channel = p;
                break;
            }
        }
    }

    decission_mtx.lock();
    if (kanan_free_obs_channel == -1 and kiri_free_obs_channel == -1){
        //berhenti
        choosen_channel_to_pass = -1;
    }else if(kanan_free_obs_channel != -1 and kiri_free_obs_channel != -1){
        //pilih yang paling jauh
        if(srf_data[kanan_free_obs_channel] >= srf_data[kiri_free_obs_channel])
            choosen_channel_to_pass = kanan_free_obs_channel;
        else
            choosen_channel_to_pass = kiri_free_obs_channel;
    }else{
        if(kanan_free_obs_channel != -1)
            choosen_channel_to_pass = kanan_free_obs_channel;
        else
            choosen_channel_to_pass = kiri_free_obs_channel;
    }
    decission_mtx.unlock();
}

short SensorControl::whichSRFChannelASVLead(){
    double asv_lead = ((float)current_steer / 600) * 90;

    for (int i = 0; i < sizeof(threshold_deg)/sizeof(threshold_deg[0]) - 1; i++){
        if(threshold_deg[i] <= asv_lead <= threshold_deg[i+1])
            return i;
    }

    return -1;
}

float SensorControl::frontObstacleDistanceSRF() {
    return std::min(srf_data[1], srf_data[2]);
}

int SensorControl::steeringDegreeToAvoidObstacle() {
    calculateSRFError();
    if(choosen_channel_to_pass == -1)
        return -9999; //berhenti cuy
    else
        return (double)((threshold_deg[choosen_channel_to_pass] +
                    threshold_deg[choosen_channel_to_pass+1])) / 2;
}

int SensorControl::frontKorbanPositionErrorCamera() {
    calculateCameraError();
    return error_camera;
}

void SensorControl::srfFilterSMA() {
    data_sensor_mtx.lock();
    for (int i = 0; i < sizeof(srf_data_raw)/sizeof(srf_data_raw[0]); i++){
        srf_data[i] = (srf_data[i] * (sma_element-1) + srf_data_raw[i]) / sma_element;
    }
    data_sensor_mtx.unlock();

}

void SensorControl::srfDataCallback(const std_msgs::Float32MultiArray &msg) {
    data_sensor_mtx.lock();
    for(int i = 0; i < msg.data.size(); i++){
        srf_data_raw[i] = msg.data[i];
    }
    data_sensor_mtx.unlock();

    srfFilterSMA();
}

void SensorControl::setCurrentSteer(int steer) {
    data_asv_mtx.lock();
    current_steer = steer;
    data_asv_mtx.unlock();
}

void SensorControl::korbanDataCameraCallback(const vision_msgs::Detection2DArray &msg) {
    data_sensor_mtx.lock();
    detection_objets = msg;
    data_sensor_mtx.unlock();
//    std::cout << "CALBACK DATA KAMERA " << detection_objets.detections.size() << "\n";
}

void SensorControl::setCurrentSaveDistance(int save) {
    data_asv_mtx.lock();
    save_distance_srf = save;
    data_asv_mtx.unlock();

}

void SensorControl::setCurrentCritLineCamera(int cl) {
    data_asv_mtx.lock();
    critline = cl;
    data_asv_mtx.unlock();

}

void SensorControl::setCurrentASVPitch(float pitch) {
    data_asv_mtx.lock();
    asv_pitch = pitch;
    data_asv_mtx.unlock();
//    std::cout << "PITCH " << asv_pitch << std::endl;
}

void SensorControl::setPitchThreshold(float deg) {
    data_asv_mtx.lock();
    pitch_angle_threshold = deg;
    data_asv_mtx.unlock();

}