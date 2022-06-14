//
// Created by lumpia on 15/07/21.
//

#ifndef SRC_APP_H
#define SRC_APP_H

#include <QApplication>
#include <QObject>
#include "MainWindow.h"
#include <typeinfo>
#include <ros/ros.h>

// #include "MissionWrapper.h"
#include <Control.h>

class App : public QApplication{
    Q_OBJECT
public:
    MainWindow *w;
    App(int &argc, char** argv) ;
    ~App();

private:
    ros::NodeHandle nh;
    // MissionWrapper *missionWrapper;
    Control *control;


    void initConnection();
};

#endif //SRC_APP_H
