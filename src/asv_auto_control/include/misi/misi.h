//
// Created by lumpia on 15/07/21.
//

#ifndef SRC_MISI_H
#define SRC_MISI_H

#include "Control.h"

class Control;

class Misi {
public:
    // Misi(Control *ct_);
    // ~Misi();
    std::string mission_name;
    virtual geometry_msgs::Twist calculateOut() {geometry_msgs::Twist tw; return tw;};

    // PIDController getPidDistanceWP();
    // PIDController getPidAngleWP();

private:
    // Control* ct;

};


#endif //SRC_MISI_H
