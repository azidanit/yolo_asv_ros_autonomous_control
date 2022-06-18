//
// Created by azidanit on 5/26/22.
//

#include "asv.h"

ASV::ASV(char* filename_venue){
    std::cout << "ASV Created\n";
    loadFileParam(filename_venue);
    local_y = local_x = local_yaw = 0;
    gps_sub = nh.subscribe("/mavros/global_position/global", 1, &ASV::gpsCallback, this);
    compass_sub = nh.subscribe("/mavros/global_position/compass_hdg", 1, &ASV::compassCallback, this);
    // mag_sub = nh.subscribe("/mavros/imu/mag", 1, &ASV::magCallback, this);
    imu_sub = nh.subscribe("/mavros/imu/data", 1, &ASV::imuCallback, this);
}

void ASV::update() {
    ros::spinOnce();

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "asv/odom";
    transformStamped.transform.translation.x = local_x;
    transformStamped.transform.translation.y = local_y;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, local_yaw);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);

    ////////////////////////////
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "asv/odom";
    transformStamped.child_frame_id = "asv/base_link";
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;
    q.setRPY(local_roll, local_pitch, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);
}

void ASV::globalToLocal(double lat_, double long_){
    local_y = LON_TO_METER * (long_center - long_);
    local_x = LAT_TO_METER * (lat_center - lat_) * -1;
}

void ASV::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    globalToLocal(msg->latitude, msg->longitude);
}

void ASV::magCallback(const sensor_msgs::MagneticField::ConstPtr& msg){
    local_yaw = atan2(msg->magnetic_field.y, msg->magnetic_field.x);
    local_yaw *= 180/M_PI;
}

void ASV::compassCallback(std_msgs::Float64 msg) {
    local_yaw = -msg.data;
    local_yaw *= M_PI/180;
}

void ASV::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    tf2::Quaternion quart_pix;
    quart_pix.setX(msg->orientation.x);
    quart_pix.setY(msg->orientation.y);
    quart_pix.setZ(msg->orientation.z);
    quart_pix.setW(msg->orientation.w);

    tf2::Matrix3x3 m(quart_pix);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    local_pitch = pitch;
    local_roll = local_roll;
}

void ASV::loadFileParam(char *filename_) {
    std::string filename = ros::package::getPath("map_image") + "/resource/" + filename_ + ".txt";

    // parse file
    double dump;
    FILE* file = fopen(filename.c_str(), "r");
    fscanf(file, "%lf, %lf %lf, %lf", &lat_top, &long_left, &lat_bottom, &long_right);
    fclose(file);

    lat_center = (lat_top + lat_bottom) / 2.0;
    long_center = (long_left + long_right) / 2.0;

    std::cout << "LEFT " << lat_top << " " << long_left << "\n";
    printf("%lf %lf\n", lat_top, long_left);
    std::cout << "RIGHT " << lat_bottom << " " << long_right << "\n";
    std::cout << "CENTER " << lat_center << " " << long_center << "\n";
    printf("%lf %lf\n", lat_center, long_center);

}