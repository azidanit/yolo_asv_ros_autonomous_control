//
// Created by lumpia on 15/07/21.
//

#ifndef SRC_MAINWINDOW_H
#define SRC_MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QThread>
#include <QMutex>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMessageBox>
#include <QFileDialog>
#include <dirent.h>
#include <QKeyEvent>

#include <QDebug>

#include <iostream>

#include "imgreceiver.h"

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    const short SPEEDSTEP = 10;
    const short STEERSTEP = 10;

    ImgReceiver *imgReceiver;

    Ui::MainWindow *ui;

    QMutex mtx_img;
    void initConnection();
    void initConnectionPID();
    void initConnectionTrim();
    void initConnectionMission();
    void initPathRecorder();
    void initUi();

    void saveParams(QString filename);
    void loadParams(QString filename);

public slots:
    void setInformationText(const char* info_text);
    void setLogText(QString log_text);
    void setSteerTrimFromRemote(int val);

    //Boatside view
    void setLeftBoat();
    void setRightBoat();
    void setLeftBoat2();
    void setRightBoat2();
    void setLeftBoat3();
    void setRightBoat3();
    void setCritLine();
    void setCritLine2();
    void changeCritLine(int, int);
    void changeCritLine2(int, int);

signals:
    void startMissionClicked();
    void stopMissionClicked();
    void resumeMissionClicked();

    void speedTrimChanged(int);
    void steerTrimChanged(int);
    void compassOffsetChanged(double);
    void changedUseCompass(bool);

    void useSpeedControl(bool);
    void useAccError(bool);
    void changeUseObstacleAvoidance(bool);
    void changeUseObstacleStop(bool);
    void changeUseFindKorban(bool);
    //mission
    void changedRTHAfterDone(bool);
    void changedPatrolLap(int);

    void pChanged(int, double);
    void iChanged(int, double);
    void dChanged(int, double);

    void speedChanged(int, int);
    void distanceChanged(int, double);
    void pitchChanged(int, double);

    void changedASVWidth(double);
    void changedASVOffset(double);

    void startRecordPath();
    void stopRecordPath();
    void clearPath();
    void savePath(QString fileName);
    void loadPath(QString fileName);
    void loadLocalPath(QString fileName);

    //Boatside view
    void critLineChanged(int, int);
    void critLineChanged2(int,int);
    void boatSideChanged(QPoint**);
    void boatSideChanged2(QPoint**);
    void boatSideChanged3(QPoint**);
};


#endif //SRC_MAINWINDOW_H
