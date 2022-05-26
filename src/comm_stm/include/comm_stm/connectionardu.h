//
// Created by dion on 27/10/2020.
//

#ifndef KKCTBN2020_CONNECTIONARDU_H
#define KKCTBN2020_CONNECTIONARDU_H
#include "serialconnection.h"

#include <ros/ros.h>
#include <mutex>
#include <thread>
//#include <QThread>
//#include <qt5/QtCore/QThread>

class connectionArdu{
private:
    std::thread thread_read;
    SerialConnection* ardu;
    ros::Publisher pub_mode,pub_param;
    ros::NodeHandle node;

    void threadRead();
public:

    connectionArdu(std::string,int);
    ~connectionArdu();
    void start();
};


#endif //KKCTBN2020_CONNECTIONARDU_H
