//
// Created by azidanit on 15/07/21.
//
#include <ros/ros.h>
#include "app.h"

#include <csignal>

void sigHandler(int s)
{
    std::signal(s, SIG_DFL);
    qApp->quit();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nala_rescuer");

    App app(argc, argv);

    std::signal(SIGINT,  sigHandler);
    std::signal(SIGTERM, sigHandler);

    return app.exec();
}