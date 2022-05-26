#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <visualization_msgs/Marker.h>
#include <bits/stdc++.h>
#include <time.h>

using namespace std;

visualization_msgs::Marker convertImageToMarker(std::string frame_id, int id, cv::Mat src, std::pair<double, double> areaSize, double x_offset=0, double y_offset=0)
{
    visualization_msgs::Marker image;
    image.header.frame_id = frame_id;
    image.header.stamp = ros::Time::now();
    image.ns = "image";
    image.id = id;
    image.action = visualization_msgs::Marker::ADD;
    image.type = visualization_msgs::Marker::TRIANGLE_LIST;
    image.scale.x = 1;
    image.scale.y = 1;
    image.scale.z = 1;

    std::pair<double, double> LUCorner;
    LUCorner.first  = -areaSize.first / 2.0 + y_offset;
    LUCorner.second = areaSize.second / 2.0 + x_offset;
    double pix_row = areaSize.first / src.rows,
           pix_col = - areaSize.second / src.cols;

    geometry_msgs::Point p;
    std_msgs::ColorRGBA crgb;

    for(int r = 0; r < src.rows; ++r) {
        for(int c = 0; c < src.cols; ++c) {
        cv::Vec3b intensity = src.at<cv::Vec3b>(r, c);
        crgb.r = intensity.val[2] / 255.0;
        crgb.g = intensity.val[1] / 255.0;
        crgb.b = intensity.val[0] / 255.0;
        crgb.a = 1.0;

        p.z = -0.1;
        p.y = LUCorner.first + r * pix_row;
        p.x = LUCorner.second + c * pix_col;
        image.points.push_back(p);
        image.colors.push_back(crgb);
        p.y = LUCorner.first + (r + 1) * pix_row;
        p.x = LUCorner.second + c * pix_col;
        image.points.push_back(p);
        image.colors.push_back(crgb);
        p.y = LUCorner.first + r * pix_row;
        p.x = LUCorner.second + (c + 1) * pix_col;
        image.points.push_back(p);
        image.colors.push_back(crgb);
        p.y = LUCorner.first + (r + 1) * pix_row;
        p.x = LUCorner.second + c * pix_col;
        image.points.push_back(p);
        image.colors.push_back(crgb);
        p.y = LUCorner.first + (r + 1) * pix_row;
        p.x = LUCorner.second + (c + 1) * pix_col;
        image.points.push_back(p);
        image.colors.push_back(crgb);
        p.y = LUCorner.first + r * pix_row;
        p.x = LUCorner.second + (c + 1) * pix_col;
        image.points.push_back(p);
        image.colors.push_back(crgb);
        }
    }
    return image;
}

void parseFile(std::string filename, std::pair<double, double> *areaSize){
    // parse file
    double dump;
    FILE* file = fopen(filename.c_str(), "r");
    fscanf(file, "%lf, %lf %lf, %lf", &dump, &dump, &dump, &dump);
    fscanf(file, "%lf %lf", &areaSize->first, &areaSize->second);
    fclose(file);
    std::cout<< dump << " " << areaSize->first << " " << areaSize->second << std::endl;

}

int main(int argc, char** argv){
    ros::init(argc, argv, "map_image");
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("map_img", 1);
    cv::Mat image;
    std::pair<double, double> areaSize;
    ros::Rate r = ros::Rate(0.3);
    
    std::string filename = ros::package::getPath("map_image") + "/resource/" + argv[1] + ".txt";
    std::string imfile = ros::package::getPath("map_image") + "/resource/" + argv[1] +".png";
    std::cout<<filename<<std::endl;
    image = cv::imread(imfile);

    // offset center of image
    double x_offset = 0;
    double y_offset = 0;
    if (argc > 3){
        x_offset = std::atof(argv[2]);
        y_offset = std::atof(argv[3]);
    }
    
    parseFile(filename, &areaSize);
    
    int loop = 3;
    visualization_msgs::Marker marker = convertImageToMarker("map", 1, image, areaSize, x_offset, y_offset) ;
    while(loop--){
        marker_pub.publish(marker);
        printf("KIRIM GAMABR MAP\n");
        r.sleep();
    }
    return 0;
}