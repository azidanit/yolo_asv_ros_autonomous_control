#ifndef SRC_CAMERACONTROL_H
#define SRC_CAMERACONTROL_H


#include "Control.h"
#include "misi/misi.h"

#include <std_msgs/Int32MultiArray.h>
#include <vision_msgs/BoundingBox2DArray.h>

#define NOT_CONDUCTED -999

class Control;
class Misi;

class CameraControl{
public:
    CameraControl(Control*, Misi* ms_, PIDController* pid_x, PIDController* pid_y);
    ~CameraControl();

    geometry_msgs::Twist calculateOut();
    bool isPersonDetected();

private:
    Control* ct;
    PIDController *pid_x, *pid_y;

    ros::Subscriber critline_sub, obj_vision_sub;

    double crit_line, horizon;
    bool is_person_detected;

    int confident_threshold, confident_counter;
    int try_rotate_in_position; //to find korban
    double start_angle;

    vision_msgs::BoundingBox2DArray obj_person_detected;
    vision_msgs::BoundingBox2DArray last_obj_person_detected;

    void initSub();

    void critLineCallback(std_msgs::Int32MultiArray msg);
    void personDetectionCallback(vision_msgs::BoundingBox2DArray msg);
};

#endif //SRC_CAMERACONTROL_Hs