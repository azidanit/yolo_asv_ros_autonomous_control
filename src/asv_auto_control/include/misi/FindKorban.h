
#ifndef SRC_FINDKORBAN_H
#define SRC_FINDKORBAN_H

#include "Control.h"
#include <rviz_plugin/Selectedwp.h>
#include "nav_msgs/Path.h"
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
    nav_msgs::Path track_path;

    ros::Subscriber path_sub;
    ros::Subscriber track_path_sub;

    PIDController *pid_distance_wp, *pid_angle_wp;
    PIDController *pid_x_cam, *pid_y_cam;
    // PIDController *pid_angle_obs, *pid_thrust_obs;

    bool use_camera_to_find_korban;

    void initSub();
    void initVar();
    void wpCallback(rviz_plugin::Selectedwp msg_wp);
    void trackPathCallback(nav_msgs::Path);

};


#endif //SRC_FINDKORBAN_H
