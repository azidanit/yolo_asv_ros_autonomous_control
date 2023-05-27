#include "simulation_translator/GpsSensor.h"

GpsSensor::GpsSensor(ros::NodeHandle nh) {
    nh_ = &nh;

    getMapOrigin();

    gps_sensor_pub_ = nh_->advertise<sensor_msgs::NavSatFix>("/mavros/global_position/global", 100);
    compass_sensor_pub_ = nh_->advertise<std_msgs::Float64>("/mavros/global_position/compass_hdg", 100);
    odom_sub_ = nh_->subscribe("/odom", 1000, &GpsSensor::onOdomCallback, this);
}

GpsSensor::~GpsSensor() {
    // TODO Auto-generated destructor stub
}

void GpsSensor::getMapOrigin(){
    std::string filename = ros::package::getPath("map_image") + "/resource/" + FILENAME_MAP_TF + ".txt";

    // parse file
    double dump;
    FILE* file = fopen(filename.c_str(), "r");
    fscanf(file, "%lf, %lf %lf, %lf", &map_origin_latlong_[0], &map_origin_latlong_[1], &map_origin_latlong_[2], &map_origin_latlong_[3]);
    fclose(file);

    lat_center = (map_origin_latlong_[0] + map_origin_latlong_[2]) / 2.0;
    long_center = (map_origin_latlong_[1] + map_origin_latlong_[3]) / 2.0;

    std::cout << "LEFT " << map_origin_latlong_[0] << " " << map_origin_latlong_[1] << "\n";
    printf("%lf %lf\n", map_origin_latlong_[0], map_origin_latlong_[1]);
    std::cout << "RIGHT " << map_origin_latlong_[2] << " " << map_origin_latlong_[3] << "\n";
    std::cout << "CENTER " << lat_center << " " << long_center << "\n";
    printf("%lf %lf\n", lat_center, long_center);

}

void GpsSensor::setCompassHdgByOdom(const nav_msgs::Odometry::ConstPtr& odom_msg){
    tf2::Quaternion q(
            odom_msg->pose.pose.orientation.x,
            odom_msg->pose.pose.orientation.y,
            odom_msg->pose.pose.orientation.z,
            odom_msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    compass_hdg_.data = -yaw * 180 / M_PI;
}

void GpsSensor::convertOdomToGps(const nav_msgs::Odometry::ConstPtr& odom_msg){
    double lat, lon, local_x, local_y;
    local_x = -odom_msg->pose.pose.position.x;
    local_y = -odom_msg->pose.pose.position.y;

    lon = long_center + (local_y / LON_TO_METER);
    lat = lat_center + (local_x / LAT_TO_METER * -1);

    gps_sensor_msg_.latitude = lat;
    gps_sensor_msg_.longitude = lon;
    gps_sensor_msg_.altitude = 0;
    gps_sensor_msg_.status.status = 0;
    gps_sensor_msg_.status.service = 0;
    gps_sensor_msg_.header.stamp = odom_msg->header.stamp;
}

void GpsSensor::onOdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){
    setCompassHdgByOdom(odom_msg); 
    convertOdomToGps(odom_msg);

    gps_sensor_pub_.publish(gps_sensor_msg_);
    compass_sensor_pub_.publish(compass_hdg_);
}
