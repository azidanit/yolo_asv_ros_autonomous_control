#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include <ros/package.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

const double LAT_TO_METER = 111000;
const double LON_TO_METER = 113321;

const std::string FILENAME_MAP_TF = "map_danau8";

class GpsSensor{
public:
    GpsSensor(ros::NodeHandle nh);
    ~GpsSensor();
private:
    ros::NodeHandle* nh_;

    ros::Publisher gps_sensor_pub_;
    ros::Publisher compass_sensor_pub_;
    ros::Subscriber odom_sub_;

    sensor_msgs::NavSatFix gps_sensor_msg_;
    std_msgs::Float64 compass_hdg_;

    double map_origin_latlong_[4];
    double lat_center;
    double long_center;
    double map_origin_distance_[2];

    void convertOdomToGps(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void getMapOrigin();
    void setCompassHdgByOdom(const nav_msgs::Odometry::ConstPtr& odom_msg);

    void onOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
};