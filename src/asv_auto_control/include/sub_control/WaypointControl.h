//
// Created by azidanit on 5/30/22
//

#ifndef SRC_WAYPOINTCONTROL_H
#define SRC_WAYPOINTCONTROL_H

#include "Control.h"
#include "misi/misi.h"
#include "utils.hpp"

#include "geometry_msgs/Twist.h"
#include <visualization_msgs/Marker.h>

class Control;
class Misi;

class WaypointControl{
public:
    WaypointControl(Control*, Misi* ms_, PIDController* pid_d, PIDController* pid_a);
    ~WaypointControl();

    geometry_msgs::Twist calculateOut();

    void setPath(nav_msgs::Path path_);
    void setWpPIDAngle(PIDController pid_new);
    void setWpPIDDistance(PIDController pid_new);

private:
    Control* ct_;
    Misi* ms_;

    PIDController *pid_distance, *pid_angle;

    ros::Publisher targetMarker_pub;

    geometry_msgs::Twist error_before, error_now;
    nav_msgs::Path left_path, right_path, main_path;

    int left_path_idx, left_path_idx_before;
    int right_path_idx, right_path_idx_before;
    int path_idx, path_idx_before;

    int path_length_trace;
    double path_angle_limit, path_keep_distance;

    double path_error_angle, path_error_dist, error_acc_angle_path,
            error_acc_dist_path, path_error_angle_before, path_error_dist_before,
            path_controlDistanceOut, path_controlAngleOut;

    geometry_msgs::Point getNextPathPoint();
    geometry_msgs::Point getInitPathPoint();

    inline double distanceLineToPoint(const geometry_msgs::Point& init, const geometry_msgs::Point& dest, const geometry_msgs::Point& curr);

    bool isArrivedPath();

    void publishMarker(geometry_msgs::Point p0, geometry_msgs::Point p1, geometry_msgs::Point p2);

};

#endif //SRC_WAYPOINTCONTROL_H
