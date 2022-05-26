#pragma once

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/String.h>
#include <rviz/panel.h>
#include <rviz_plugin/NextDest.h>

#include <QMessageBox>

namespace Ui{
    class Mission;
}


class MissionPanel: public rviz::Panel{
Q_OBJECT
public:
    MissionPanel(QWidget* parent = 0);

public Q_SLOTS:
    void overrideButtonCallback();

protected:
    Ui::Mission *ui;

private:
    /* Initialize publishers */
    void initPublishers();

    /* Initialize subscribers */
    void initSubscribers();

    /* Connect each Ui to the method of this class. */
    void initConnection();

    /*
    * Start mission:
    * 0: stop
    * 1: start
    */
   
    void startMission(int);

    /* Publish waypoint navigation
     * -1 : previous
     * +1 : next
     */
    void publishWPNav(int8_t);

    /* Publish sequence after 'Set' button clicked */
    void setSequence(std::string);

    /* Callbakc for nextDest subscriber -> contain misi, wp_index */
    void nextDestCallback(const rviz_plugin::NextDest& msg);

    ros::NodeHandle nh;
    ros::Publisher statePub, sequencePub, wpnavPub;
    ros::Subscriber nextDestSub;

};