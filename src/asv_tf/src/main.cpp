#include <ros/ros.h>
#include "asv.h"
/** Main node entry point. */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "asv_tf_node");
    ros::NodeHandle node;
    ros::Rate r(50);

    ASV asv(argv[1]);

    while(ros::ok()){
        asv.update();
        r.sleep();
    }

    return 0;
}
