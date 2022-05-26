#pragma once

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif
#include <QPushButton>
#include <std_msgs/Int32.h>
#include "drive_widget.h"

#define MAX_THROTLE 500
#define MAX_STEERING 600
class QLineEdit;

    class DriveWidget;

class ManualControl: public rviz::Panel
{
Q_OBJECT
public:

    ManualControl( QWidget* parent = 0 );

    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;

    // Next come a couple of public Qt slots.
public Q_SLOTS:
    // The control area, DriveWidget, sends its output to a Qt signal
    // for ease of re-use, so here we declare a Qt slot to receive it.
    void setVel( float speed, float servo );
    void overrideButtonCallback();
    // In this example setTopic() does not get connected to any signal
    // (it is called directly), but it is easy to define it as a public
    // slot instead of a private function in case it would be useful to
    // some other user.
    void setTopic( const QString& topic );

    // Here we declare some internal slots.
protected Q_SLOTS:
    // sendvel() publishes the current velocity values to a ROS
    // topic.  Internally this is connected to a timer which calls it 10
    // times per second.
    void sendVel();

    // updateTopic() reads the topic name from the QLineEdit and calls
    // setTopic() with the result.
    void updateTopic();

    // Then we finish up with protected member variables.
public:
//    void trimSubsCallback(const std_msgs::Int32::ConstPtr&);

    // The control-area widget which turns mouse events into command
    // velocities.
    DriveWidget* drive_widget_;

    // One-line text editor for entering the outgoing ROS topic name.
    QLineEdit* output_topic_editor_;
    QLineEdit* vel_string;
    QPushButton* overrideButton;
    // The current name of the output topic.
    QString output_topic_;

    // The ROS publisher for the command velocity.
    ros::Publisher velocity_publisher_;
    ros::Subscriber sub;
    // The ROS node handle.
    ros::NodeHandle nh_;
//    void chatterCallback(const std_msgs::Int32::ConstPtr&);
    // The latest velocity values from the drive widget.
    float speed;
    float servo;
    int trim;
    bool isOverride;
    // END_TUTORIAL
//    void chatterCallback(const std_msgs::Int32_<std::allocator<void>>::ConstPtr &msg);
    void trimSubsCallback(const std_msgs::Int32_<std::allocator<void>>::ConstPtr &msg);
};


