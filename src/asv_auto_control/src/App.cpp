//
// Created by lumpia on 15/07/21.
//

#include "app.h"

App::App(int &argc, char** argv)
        : QApplication(argc, argv)
{
    w = new MainWindow();
    ROS_INFO("Nala Rescuer App started");
    w->show();

    missionWrapper = new MissionWrapper(nh, argv[1]);

    initConnection();

    missionWrapper->start();
}

App::~App(){
    delete w;
    ROS_INFO("Nala Rescuer App Deleted");
}

void App::initConnection() {
    connect(w, &MainWindow::startMissionClicked, missionWrapper, &MissionWrapper::startMission, Qt::DirectConnection);
    connect(w, &MainWindow::resumeMissionClicked, missionWrapper, &MissionWrapper::resumeMission, Qt::DirectConnection);
    connect(w, &MainWindow::stopMissionClicked, missionWrapper, &MissionWrapper::stopMission, Qt::DirectConnection);

    connect(w, &MainWindow::steerTrimChanged, missionWrapper, &MissionWrapper::changeSteerTrim, Qt::DirectConnection);
    connect(w, &MainWindow::speedTrimChanged, missionWrapper, &MissionWrapper::changeSpeedTrim, Qt::DirectConnection);
    connect(w, &MainWindow::compassOffsetChanged, missionWrapper, &MissionWrapper::calibrateCompass, Qt::DirectConnection);
    connect(w, &MainWindow::changedUseCompass, missionWrapper, &MissionWrapper::changeUseCompass, Qt::DirectConnection);
    connect(w, &MainWindow::changeUseFindKorban, missionWrapper, &MissionWrapper::changeUseFindKorban, Qt::DirectConnection);

    connect(w, &MainWindow::useSpeedControl, missionWrapper, &MissionWrapper::useSpeedControl, Qt::DirectConnection);
    connect(w, &MainWindow::useAccError, missionWrapper, &MissionWrapper::useAccError, Qt::DirectConnection);
    connect(w, &MainWindow::changeUseObstacleAvoidance, missionWrapper, &MissionWrapper::changeUseObstacleAvoidance, Qt::DirectConnection);
    connect(w, &MainWindow::changeUseObstacleStop, missionWrapper, &MissionWrapper::changeUseObstacleStop, Qt::DirectConnection);
    connect(w, &MainWindow::changedRTHAfterDone, missionWrapper, &MissionWrapper::changeRTHAfterDone, Qt::DirectConnection);
    connect(w, &MainWindow::changedPatrolLap, missionWrapper, &MissionWrapper::changePatrolLap, Qt::DirectConnection);

    connect(w, &MainWindow::pChanged, missionWrapper, &MissionWrapper::changeP, Qt::DirectConnection);
    connect(w, &MainWindow::iChanged, missionWrapper, &MissionWrapper::changeI, Qt::DirectConnection);
    connect(w, &MainWindow::dChanged, missionWrapper, &MissionWrapper::changeD, Qt::DirectConnection);

    connect(w, &MainWindow::speedChanged, missionWrapper, &MissionWrapper::changeSpeed, Qt::DirectConnection);
    connect(w, &MainWindow::distanceChanged, missionWrapper, &MissionWrapper::changeDistance, Qt::DirectConnection);

    connect(w, &MainWindow::changedASVWidth, missionWrapper, &MissionWrapper::changeASVWidth, Qt::DirectConnection);
    connect(w, &MainWindow::changedASVOffset, missionWrapper, &MissionWrapper::changeASVOffset, Qt::DirectConnection);


    connect(missionWrapper, &MissionWrapper::setLogInformationDisplay, w, &MainWindow::setLogText, Qt::QueuedConnection);
    connect(missionWrapper, &MissionWrapper::steer_trim_changed_from_remote, w, &MainWindow::setSteerTrimFromRemote, Qt::QueuedConnection);
    connect(missionWrapper, &MissionWrapper::setCritLine, w, &MainWindow::changeCritLine, Qt::QueuedConnection);

    //PATH INIT
    connect(w, &MainWindow::startRecordPath, missionWrapper, &MissionWrapper::startRecordPath, Qt::DirectConnection);
    connect(w, &MainWindow::stopRecordPath, missionWrapper, &MissionWrapper::stopRecordPath, Qt::DirectConnection);
    connect(w, &MainWindow::clearPath, missionWrapper, &MissionWrapper::clearPath, Qt::DirectConnection);
    connect(w, &MainWindow::savePath, missionWrapper, &MissionWrapper::savePath, Qt::DirectConnection);
    connect(w, &MainWindow::loadPath, missionWrapper, &MissionWrapper::loadPath, Qt::DirectConnection);
    connect(w, &MainWindow::loadLocalPath, missionWrapper, &MissionWrapper::loadLocalPath, Qt::DirectConnection);

    connect(w, &MainWindow::critLineChanged, missionWrapper, &MissionWrapper::changeCritLine, Qt::DirectConnection);
    connect(w, &MainWindow::boatSideChanged, missionWrapper, &MissionWrapper::changeBoatSide, Qt::DirectConnection);

    //    connect(w, &MainWindow::critLineChanged2, misi, &Misi::changeCritLine2, Qt::DirectConnection);

}
