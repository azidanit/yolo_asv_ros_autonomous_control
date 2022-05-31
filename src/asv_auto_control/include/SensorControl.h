#ifndef SRC_SENSORCONTROL_H
#define SRC_SENSORCONTROL_H

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <vision_msgs/Detection2DArray.h>

#include <mutex>
#include <thread>


class SensorControl{
public:
    SensorControl();
    ~SensorControl();

    void setCurrentSteer(int steer);
    void setCurrentASVPitch(float pitch);

    void setCurrentSaveDistance(int save);
    void setCurrentCritLineCamera(int cl);
    void setPitchThreshold(float deg);

    //decission result
    bool isThereIsObstableSRF();
    int steeringDegreeToAvoidObstacle();
    float frontObstacleDistanceSRF();

    //calculation result
    int frontKorbanPositionErrorCamera();
    double korbanDistanceEstimationCamera();
    double korbanDistanceSRF();

private:
    const float STEER_TO_DEGREE = 0.15; // 90 DEGREE / 600

    ros::NodeHandle nh_;
    ros::Subscriber srf_sub, korban_camera_sub;

    std::thread run_thread;
    void run();

    int current_steer;
    float asv_pitch;

    void initSubscriber();

    //SRF PARAM
    double save_distance_srf;
    float pitch_angle_threshold;
    //SRF VARIABLE
    double srf_data_raw[4], srf_data[4];
    bool direction_free_obs[4];
    int sma_element;
    std::mutex data_sensor_mtx, data_asv_mtx;
    //SRF FUCNITON
    void srfFilterSMA();
    //callback topic
    void srfDataCallback(const std_msgs::Float32MultiArray &msg);
    //-------CONTROL SRF-----------'
    std::mutex decission_mtx;
    int threshold_deg[5] = {-90,30,0,30,90};
    int choosen_channel_to_pass;
    void calculateSRFError();
    short whichSRFChannelASVLead();



    //CONTROL CAMERA
    geometry_msgs::Point controlled_korban_pos;
    int critline;
    vision_msgs::Detection2DArray detection_objets;
    void calculateCameraError();

    int error_camera;
    double distance_est_korban_camera;
    //CALBACK CAMERA
    void korbanDataCameraCallback(const vision_msgs::Detection2DArray &msg);
};

#endif //SRC_SENSORCONTROL_H
