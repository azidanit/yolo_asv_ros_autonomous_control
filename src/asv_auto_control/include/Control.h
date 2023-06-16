//
// Created by azidanit on 5/30/22.
//

#ifndef SRC_CONTROL_H
#define SRC_CONTROL_H

#include "ros/ros.h"
#include <ros/package.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/String.h>

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
#include <sub_control/ObstacleAvoidanceControl.h>
#include <rviz_plugin/Selectedwp.h>

#define LAT_TO_METER 111000
#define LON_TO_METER 113321

class FindKorban;
class ObstacleAvoidanceControl;

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
    PIDController* get_pid_x_cam_find_korban();
    PIDController* get_pid_y_cam_find_korban();
    PIDController* get_pid_angle_obs_avoid();
    PIDController* get_pid_thrust_obs_avoid();

    double speedControlCalculate(double target);
    std::vector<std::pair<int, int> > sortArr(int arr[], int n);

private:
    void dummyFunction();

    std::mutex param_qt_mtx, mission_state_mtx;

    Misi* misisons[1];
    FindKorban *find_korban;
    ObstacleAvoidanceControl *obstacle_avoid_control;

    //FIND KORBAN PIDS
    PIDController pid_angle_wp_find_korban, pid_distance_wp_find_korban;
    PIDController pid_x_cam_find_korban, pid_y_cam_find_korban;
    PIDController pid_angle_obs_avoid, pid_thrust_obs_avoid;

    //General PIDS
    PIDController pid_speed_control;

    //ASV General Cmd
    geometry_msgs::Twist out_cmd;
    bool is_test_motor;

    //EMA OUT THRUSTER
    double alpha_ema;
    geometry_msgs::Twist out_cmd_ema, out_cmd_ema_before;

    //PATH PATTERN SEARCH
    double start_lat_long[2];
    double end_lat_long[2];
    double track_specs[3];

    //Mission Var
    std_msgs::UInt16MultiArray mission_state;
    double target_constant_thrust, steer_trim, thrust_trim;
    bool use_speed_control;

    std_msgs::String mission_status_msg;

    ros::Subscriber gps_raw_sub, mission_state_control_sub;
    ros::Publisher asv_cmd_vel_pub, mission_status_string_pub;

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
    ros::Publisher track_path_pub, track_start_text_pub;
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
    void sendCmdVel();

    void getLocalFromGlobalPose(double lat_global, double lon_global, double *x_local, double *y_local);
    void offsetPathFromGlobalPose(nav_msgs::Path &path, double lat, double lon);

public slots:
    //DARI UI
    void startMission();
    void stopMission();
    void resumeMission();
    void testMotor(bool);

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
    void changeSpeedDouble(int, double);
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

    void changeStartLatLong(double, double);
    void changeEndLatLong(double, double);
    void changeTrackSpecs(double, double, double);
    void generateTrackPath(double);

signals:
    void setLogInformationDisplay(QString msg);
    void steer_trim_changed_from_remote(int val);

    void setCritLine(int idx, int val);
};

#endif //SRC_CONTROL_H
