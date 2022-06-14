#include "misi/FindKorban.h"

FindKorban::FindKorban(Control *ct_){
    this->ct = ct_;
    
    initSub();

    std::cout << "FIND KORBAN CREATED\n";
    std::cout << "FIND KORBAN CREATED\n";
    std::cout << "FIND KORBAN CREATED\n";
    std::cout << "FIND KORBAN CREATED\n";
    // wp_control = new WaypointControl(ct_);


}

FindKorban::~FindKorban(){

}


void FindKorban::initSub(){
    path_sub = ct->nh.subscribe("/waypoints/mission_1/pre", 1, &FindKorban::wpCallback, this);

}

void FindKorban::wpCallback(nav_msgs::Path ms_path){
    std::cout << "FIND KORBAN GOT WP\n";
    wp_control->setPath(ms_path);
}