#include <stdio.h>
#include <iostream>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>


//#include <geometry_msgs/Twist.h>
#include <rviz_plugin/ManualCtrl.h>
#include <std_msgs/Int32.h>
#include "drive_widget.h"
#include "ManualControl.h"




ManualControl::ManualControl( QWidget* parent )
        : rviz::Panel( parent )
        , speed(0 )
        , servo(0 )
{
    isOverride = false;
    // Then create the control widget.
    drive_widget_ = new DriveWidget;

    // Lay out the topic field above the control widget.
    QVBoxLayout* layout = new QVBoxLayout;
    overrideButton = new QPushButton;
    overrideButton->setText("OVERRIDE");
    overrideButton->setStyleSheet("background-color:green;");
    layout->addWidget(overrideButton);
    layout->addWidget( drive_widget_ );
    setLayout( layout );

    QTimer* output_timer = new QTimer( this );

    // Next we make signal/slot connections.
    connect( drive_widget_, SIGNAL( outputVelocity( float, float )), this, SLOT( setVel( float, float )));
    connect( overrideButton, SIGNAL(clicked()),this, SLOT(overrideButtonCallback()));

    connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));
    Q_EMIT setTopic("/manualctrl");
    // Start the timer.
    output_timer->start( 20 );
    // sub = nh_.subscribe("/trim", 10, &ManualControl::trimSubsCallback,this);
    // Make the control widget start disabled, since we don't start with an output topic.
    drive_widget_->setEnabled( false );
}

void ManualControl::trimSubsCallback(const std_msgs::Int32::ConstPtr& msg)
{
//    std::cout<<"MSG "<<msg->data<<std::endl;
    trim = msg->data;
}

void ManualControl::setVel( float spd, float srv )
{
//    std::cout<<" SET VEL "<<spd<<" "<<srv<<std::endl;
    speed = spd*8;
    servo = srv*30;
    if(speed > MAX_THROTLE)
        speed = MAX_THROTLE;
    else if(speed < -MAX_THROTLE)
        speed = -MAX_THROTLE;

    if(servo > MAX_STEERING)
        servo = MAX_STEERING;
    else if(servo < -MAX_STEERING)
        servo = -MAX_STEERING;


}


void ManualControl::setTopic( const QString& new_topic )
{
    // Only take action if the name has changed.
    if( new_topic != output_topic_ )
    {
        output_topic_ = new_topic;

        if( output_topic_ == "" )
        {
            velocity_publisher_.shutdown();
        }
        else
        {
            velocity_publisher_ = nh_.advertise<rviz_plugin::ManualCtrl>( output_topic_.toStdString(), 1 );
        }
        Q_EMIT configChanged();
    }

}

// Publish the control velocities if ROS is not shutting down and the
// publisher is ready with a valid topic name.
void ManualControl::sendVel()
{
    if( isOverride && ros::ok() && velocity_publisher_ )
    {

        rviz_plugin::ManualCtrl msg;
        msg.speed = speed;
        msg.servo = servo;

        velocity_publisher_.publish( msg );
    }
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void ManualControl::save( rviz::Config config ) const
{
    rviz::Panel::save( config );
}

// Load all configuration data for this panel from the given Config object.
void ManualControl::load( const rviz::Config& config )
{
    rviz::Panel::load( config );
    QString topic;
}

void ManualControl::overrideButtonCallback() {
    if(isOverride == false){
        isOverride = true;
        overrideButton->setStyleSheet("background-color:red;");
        overrideButton->setText("RELEASE");
        drive_widget_->setEnabled(true);
    }
    else{
        isOverride = false;
        overrideButton->setStyleSheet("background-color:green;");
        overrideButton->setText("OVERRIDE");
        drive_widget_->setEnabled(false);
    }
}


// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ManualControl,rviz::Panel )
// END_TUTORIAL
