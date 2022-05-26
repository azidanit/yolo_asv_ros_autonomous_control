#ifndef IMGRECEIVER_H
#define IMGRECEIVER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <compressed_image_transport/compressed_subscriber.h>
#include <compressed_image_transport/compression_common.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <QThread>
#include <QMutex>
#include <QImage>

#include <time.h>

using namespace cv;

class ImgReceiver : public QThread {
Q_OBJECT;

public:
    ImgReceiver();
    ~ImgReceiver();

    ros::NodeHandle nh;
public Q_SLOTS:
    void kill();

Q_SIGNALS:
    void imageReceived(QImage);
    void cobasignal(int);

private:
    image_transport::ImageTransport itrans;
    ros::Subscriber  subs;

    Mat frame;
    QMutex mtx;
    bool isKilled;
    QImage image_copy;
    void run();
    void processImage(Mat &img);
    void receiveImage(const sensor_msgs::CompressedImageConstPtr& msg);
    QImage* matToQImage(const cv::Mat&);
};

#endif //IMGRECEIVER_H
