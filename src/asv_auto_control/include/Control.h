//
// Created by azidanit on 5/30/22.
//

#ifndef SRC_CONTROL_H
#define SRC_CONTROL_H

#include "ros/ros.h"

#include "QThread"

class Control : public QThread{
    Q_OBJECT

public:
    Control();
    ~Control();
    void run();

    ros::NodeHandle nh;

private:
    void dummyFunction();

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
