#include <sub_control/ObstacleAvoidanceControl.h>

ObstacleAvoidanceControl::ObstacleAvoidanceControl(Control* ct_, Misi* ms_, PIDController* pid_angle_, PIDController* pid_thrust_){
    this->ct = ct_;
    this->pid_angle = pid_angle_;
    this->pid_thrust = pid_thrust_;

    is_there_is_obstacle = false;

    early_threshold_distance = 160;
    threshold_distance = 130;
    threshold_confident = 2;

    srf_angle[0] = -20;
    srf_angle[1] = 0;
    srf_angle[2] = 20;

    for (int i = 0; i < SRF_NUMBER; i++){
        srf_data[i] = 9999;
        free_gap[i] = early_free_gap[i] = true;

    }    


    srf_sub = ct->nh.subscribe("/stm32/srf",2,&ObstacleAvoidanceControl::srfCallback, this);
}

ObstacleAvoidanceControl::~ObstacleAvoidanceControl(){

}

geometry_msgs::Twist ObstacleAvoidanceControl::calculateOut(){
    geometry_msgs::Twist out_cmd;

    double error = 0;

    for (int i = 0; i < SRF_NUMBER; i++){
        free_gap[i] = true;
        if(obs_gap_confident[i] > 0)
            free_gap[i] = false;

        early_free_gap[i] = true;
        if(early_obs_gap_confident[i] > 0)
            early_free_gap[i] = false;
    }

    bool any_gap = false;
    bool all_gap_free = true;
    bool all_early_gap_free = true;
    for (int i = 0; i < SRF_NUMBER; i++){
        any_gap |= free_gap[i];
        all_gap_free &= free_gap[i];
        all_early_gap_free &= early_free_gap[i];
    } 

    if (!any_gap){
        //mundur tidak ada jalan di depan
        out_cmd.linear.x = - ct->speedControlCalculate(-1);
        return out_cmd;
    }else if(all_gap_free){
        if(all_early_gap_free){
            is_there_is_obstacle = false;
            return out_cmd;
        }else{
            //early obstacle control
        }
    }else{ //obstacle dekat
        double error_mpx = 150 -  sorted_distance[0][1];
        int choosen_gap_idx = sorted_distance[-1][0];
        int avoided_gap_idx = sorted_distance[0][0];
        int steer_angle = srf_data_angle[choosen_gap_idx] - self.srf_data_angle[avoided_gap_idx]/2;
        std::cout << error_mpx << " " <<  choosen_gap_idx << " " << steer_angle << "\n" ;

        out_cmd.linear.x = ct->speedControlCalculate(1);
        out_cmd.angular.z = ((steer_angle * (error_mpx/4)) / 1000) * pid_angle->getP();
    }
}

bool ObstacleAvoidanceControl::isObstacleDetected(){
    return is_there_is_obstacle;
}
 
void ObstacleAvoidanceControl::srfCallback(std_msgs::Int32MultiArray msg_){
    if(msg_.data.size()>2){
        for (int i = 0; i < SRF_NUMBER; i++){
            srf_data[i] = msg_.data[i];
        }    

    }else{
        return;
    }

    for (int i = 0; i < SRF_NUMBER; i++){
        if(threshold_distance >= srf_data[i])
            obs_gap_confident[i] += 1;
        else
            obs_gap_confident[i] -= 1 ;

        if(obs_gap_confident[i] > threshold_confident)
            obs_gap_confident[i] = threshold_confident;
        if(obs_gap_confident[i] < -threshold_confident)
            obs_gap_confident[i] = -threshold_confident;

        //EARLY
        if(early_threshold_distance >= srf_data[i])
            early_obs_gap_confident[i] += 1;
        else
            early_obs_gap_confident[i] -= 1;

        if(early_obs_gap_confident[i] > early_threshold_distance)
            early_obs_gap_confident[i] = early_threshold_distance;
        if(early_obs_gap_confident[i] < -early_threshold_distance)
            early_obs_gap_confident[i] = -early_threshold_distance;
    }    



}