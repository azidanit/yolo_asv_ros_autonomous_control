//
// Created by azidanit on 5/30/22.
//

#ifndef SRC_CONTROL_H
#define SRC_CONTROL_H

#include "ros/ros.h"

#include <nav_msgs/Path.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt16MultiArray.h>

#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "QThread"
#include <QPoint>
#include <thread>
#include <QFile>
#include <QTextStream>

#include "utils_class/PIDController.h"

#include <misi/FindKorban.h>
#include <misi/misi.h>
#include <rviz_plugin/Selectedwp.h>

#define LAT_TO_METER 111000
#define LON_TO_METER 113321

class FindKorban;

typedef struct {
    geometry_msgs::Point pos;
    double roll, pitch, yaw;
} TF_simplified;

enum MissionName{
    FIND_KORBAN = 0,
    HOLD_KORBAN = 1,
    RTH = 2,
    QUICK_RTH = 21,
};

class Control : public QThread{
    Q_OBJECT

public:
    Control();
    ~Control();
    void run();

    TF_simplified getRobotTf();
    double getCurrentASVSpeed();

    ros::NodeHandle nh;

    PIDController* get_pid_angle_wp_find_korban();
    PIDController* get_pid_distance_wp_find_korban();

    double speedControlCalculate(double target);

private:
    void dummyFunction();

    std::mutex param_qt_mtx, mission_state_mtx;

    Misi* misisons[1];
    FindKorban *find_korban;

    //FIND KORBAN PIDS
    PIDController pid_angle_wp_find_korban, pid_distance_wp_find_korban;

    //General PIDS
    PIDController pid_speed_control;

    //ASV General Cmd
    geometry_msgs::Twist out_cmd;

    //Mission Var
    std_msgs::UInt16MultiArray mission_state;

    ros::Subscriber gps_raw_sub, mission_state_control_sub;
    ros::Publisher asv_cmd_vel_pub;

    //----ASV TF variable----
    tf::StampedTransform currentBoatTF;
    geometry_msgs::TransformStamped transformStamped;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener *tfListener;
    TF_simplified ASV_TF;
    std::string world_frame, robot_frame;
    std::mutex tf_mutex, sensor_mtx;
    // ----ASV TF Function-----
    void listenASVTF();
    // ASV RELATED VAR
    double current_asv_speed;



    //PATH VARIABLE
    ros::Publisher path_pub, loaded_path_pub, global_path_pub;
    nav_msgs::Path path_msg, loaded_path_msg, global_path_msg;
    std::vector<sensor_msgs::NavSatFix> global_path;
    sensor_msgs::NavSatFix current_gps;
    bool is_path_recorded;
    std::mutex path_mtx, gps_mtx;
    double LatitudeTop, LatitudeBot, LongitudeRight, LongitudeLeft, LongitudeCenter, LatitudeCenter;
    std::thread path_thread;
    //PATH FUNCTION
    void recordPath();
    geometry_msgs::PoseStamped globalToPose(double lat, double lon);

    void initVar();
    void initSub();
    void initPub();

    void GPSRawCallback(sensor_msgs::NavSatFix data_gps);
    void stateMissionCallback(std_msgs::UInt16MultiArray msgl);


public slots:
    //DARI UI
    void startMission();
    void stopMission();
    void resumeMission();

    void changeSteerTrim(int);
    void changeSpeedTrim(int);
    void calibrateCompass(double);
    void changeUseCompass(bool val);

    void useSpeedControl(bool);
    void useAccError(bool);
    void changeUseObstacleAvoidance(bool);
    void changeUseObstacleStop(bool);
    void changeUseFindKorban(bool);
    void changeRTHAfterDone(bool);

    void changePatrolLap(int val);

    void changeP(int, double);
    void changeI(int, double);
    void changeD(int, double);

    void changeSpeed(int, int);
    void changeDistance(int, double);
    void pitchChanged(int, double val);

    void changeASVWidth(double);
    void changeASVOffset(double);

    void startRecordPath();
    void stopRecordPath();
    void clearPath();  // Clear recorded path
    void savePath(QString filename);
    void loadPath(QString filename);
    void loadLocalPath(QString filename);

    void changeCritLine(int, int);
    void changeBoatSide(QPoint**);

signals:
    void setLogInformationDisplay(QString msg);
    void steer_trim_changed_from_remote(int val);

    void setCritLine(int idx, int val);
};

#endif //SRC_CONTROL_H
