//
// Created by azidanit on 5/30/22.
//

#ifndef SRC_CONTROL_H
#define SRC_CONTROL_H

#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>

#include <tf/transform_listener.h>


#include "QThread"
#include <QPoint>
#include <thread>
#include <QFile>
#include <QTextStream>

#define LAT_TO_METER 111000
#define LON_TO_METER 113321


typedef struct {
    geometry_msgs::Point pos;
    double roll, pitch, yaw;
} TF_simplified;

class Control : public QThread{
    Q_OBJECT

public:
    Control();
    ~Control();
    void run();

    ros::NodeHandle nh;

private:
    void dummyFunction();

    std::mutex param_qt_mtx;


        //----ASV variable----
    tf::StampedTransform currentBoatTF;
    TF_simplified ASV_TF;
    std::string world_frame, robot_frame;
    std::mutex tf_mutex, sensor_mtx;
    tf::TransformListener boatTF_sub;


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
};

#endif //SRC_CONTROL_H
