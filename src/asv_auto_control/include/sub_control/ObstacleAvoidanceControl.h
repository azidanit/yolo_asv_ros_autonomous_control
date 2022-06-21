#ifndef SRC_OBSTACLEAVOIDANCECONTROL_H
#define SRC_OBSTACLEAVOIDANCECONTROL_H

#include "Control.h"
#include "misi/misi.h"

#include <std_msgs/Int32MultiArray.h>

#define SRF_NUMBER 3

class Control;
class Misi;

class ObstacleAvoidanceControl{
public:
    ObstacleAvoidanceControl(Control*, Misi* ms_, PIDController* pid_angle, PIDController* pid_thrust);
    ~ObstacleAvoidanceControl();

    geometry_msgs::Twist calculateOut();
    bool isObstacleDetected();

private:
    Control* ct;

    PIDController *pid_angle, *pid_thrust;

    ros::Subscriber srf_sub;
    ros::Publisher srf_obj_pub;

    int srf_angle[SRF_NUMBER];
    int srf_data[SRF_NUMBER];
    
    int obs_gap_confident [SRF_NUMBER];
    int early_obs_gap_confident[SRF_NUMBER];

    bool free_gap[SRF_NUMBER];
    bool early_free_gap[SRF_NUMBER];


    int early_threshold_distance, threshold_distance, threshold_confident;
    bool is_there_is_obstacle;

    void srfCallback(std_msgs::Int32MultiArray msg_);


}

#endif //SRC_OBSTACLEAVOIDANCECONTROL_H
