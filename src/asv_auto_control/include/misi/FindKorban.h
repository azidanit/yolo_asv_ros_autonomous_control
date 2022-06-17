
#ifndef SRC_FINDKORBAN_H
#define SRC_FINDKORBAN_H

#include "Control.h"
#include <rviz_plugin/Selectedwp.h>
#include "misi/misi.h"

#include "sub_control/WaypointControl.h"
#include "sub_control/CameraControl.h"

class Control;
class WaypointControl;
class CameraControl;

class FindKorban : public Misi{
public:
    FindKorban(Control *ct_);
    ~FindKorban();

    geometry_msgs::Twist calculateOut();

    void stop();

private:
    Control* ct;
    WaypointControl *wp_control;
    CameraControl *camera_control;

    ros::Subscriber path_sub;

    PIDController *pid_distance_wp, *pid_angle_wp;
    PIDController *pid_x_cam, *pid_y_cam;

    bool use_camera_to_find_korban;

    void initSub();
    void initVar();
    void wpCallback(rviz_plugin::Selectedwp msg_wp);


};


#endif //SRC_FINDKORBAN_H
