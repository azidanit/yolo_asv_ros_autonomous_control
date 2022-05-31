//
// Created by azidanit on 5/30/22.
//

#ifndef SRC_WAYPOINTCONTROL_H
#define SRC_WAYPOINTCONTROL_H

#include "MissionWrapper.h"
#include "geometry_msgs/Twist.h"
#include "utils.hpp"

class MissionWrapper;

class WaypointControl{
public:
    WaypointControl(MissionWrapper*);
    ~WaypointControl();

    geometry_msgs::Twist calculateOut();
private:
    MissionWrapper* mw_;

    bool is_using_left_path;

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
    bool isArrivedLeftPath();
    bool isArrivedRightPath();

};

#endif //SRC_WAYPOINTCONTROL_H
