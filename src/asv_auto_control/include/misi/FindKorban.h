
#ifndef SRC_FINDKORBAN_H
#define SRC_FINDKORBAN_H

#include "Control.h"
#include "misi/misi.h"

#include "sub_control/WaypointControl.h"

class Control;

class FindKorban : public Misi{
public:
    FindKorban(Control *ct_);
    ~FindKorban();

private:
    Control* ct;
    WaypointControl *wp_control;

    void initSub();

    ros::Subscriber path_sub;

    void wpCallback(nav_msgs::Path ms_path);

};


#endif //SRC_FINDKORBAN_H
