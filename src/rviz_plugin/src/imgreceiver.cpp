#include "imgreceiver.h"
#include <QDebug>

ImgReceiver::ImgReceiver(): itrans(nh) {
    // moveToThread(this);
    isKilled = false;
    // qDebug() << "imgr : " << QThread::currentThreadId();
    subs = nh.subscribe("/vision/image_raw/compressed", 1, &ImgReceiver::receiveImage, this);

}

ImgReceiver::~ImgReceiver(){

}

void ImgReceiver::run(){
    isKilled = false;
    sleep(3);
    ros::Rate r(50);
    while(isKilled == false){
        ros::spinOnce();
        r.sleep();
    }
}

void ImgReceiver::kill(){
    mtx.lock();
    qDebug()<<"killed";
    isKilled = true;
    mtx.unlock();
}

void ImgReceiver::receiveImage(const sensor_msgs::CompressedImageConstPtr& msg){
    // qDebug() << "img ros : " << QThread::currentThreadId();
    cv_bridge::CvImagePtr cvPtr;
//    std::cout << "DAPAT IMAGE\n";
    try{
        cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    }
    catch(cv_bridge::Exception& e){
        ROS_INFO("cv_bridge exception $s", e.what());
        return;
    }
    Q_EMIT imageReceived(*matToQImage(cvPtr->image));
}

QImage* ImgReceiver::matToQImage(const cv::Mat& mat){
    image_copy = QImage((uchar*) mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888).copy();
    return &image_copy;
}
