#include <camera_plugin.h>
#include <thread>
#include <iostream>


CameraPanel::CameraPanel(QWidget* parent):
        rviz::Panel( parent ), ui_(new Ui::cameraResponsive()){

    ui_->setupUi(this);
    img_receiver =  new ImgReceiver();

    std::cout <<"mari connect" << std::endl;

    connectWidget();
    initUi();

    critline_pub = nh.advertise<std_msgs::Int32MultiArray>("/rviz_plugin/camera/critline",1);
    boatside_pub = nh.advertise<geometry_msgs::PoseArray>("/rviz_plugin/camera/boatside",1);

    std::cout << "camera plugin" << std::endl;
}

void CameraPanel::initUi(){
    Q_EMIT boatSideChanged(ui_->camera->getBoatSide());
    Q_EMIT boatSideChanged2(ui_->camera->getBoatSide2());
    Q_EMIT critLineChanged(0, ui_->camera->getCritLine(0));
    Q_EMIT critLineChanged(1, ui_->camera->getCritLine(1));

}

void CameraPanel::connectWidget(){
    // Boat Side and CritLine
    connect(ui_->buttonLeftBoat, &QPushButton::clicked, this, &CameraPanel::setLeftBoat);
    connect(ui_->buttonRightBoat, &QPushButton::clicked, this, &CameraPanel::setRightBoat);
    connect(ui_->buttonLeftBoat_2, &QPushButton::clicked, this, &CameraPanel::setLeftBoat2);
    connect(ui_->buttonRightBoat_2, &QPushButton::clicked, this, &CameraPanel::setRightBoat2);
    connect(ui_->buttonCritLine, &QPushButton::clicked, this, &CameraPanel::setCritLine);


    connect(img_receiver, &ImgReceiver::imageReceived,
            this,
            [=](QImage image){
                mtx_img.lock();
                ui_->camera->drawPixmap(image);
//            delete image;
                mtx_img.unlock();
            },
            Qt::DirectConnection);
//    img_receiver->start();
}


void CameraPanel::setLeftBoat(){
    if(ui_->buttonLeftBoat->text() == "L"){
        ui_->camera->changeSetMode(3);
        ui_->buttonLeftBoat->setText("Done");
        ui_->buttonRightBoat->setDisabled(true);
    }
    else{
        ui_->camera->changeSetMode(0);
        ui_->buttonLeftBoat->setText("L");
        ui_->buttonRightBoat->setDisabled(false);
        Q_EMIT boatSideChanged(ui_->camera->getBoatSide());

        boatSide[0] = *(ui_->camera->getBoatSide())[0];
        boatSide[1] = *(ui_->camera->getBoatSide())[1];
        boatSide[2] = *(ui_->camera->getBoatSide())[2];
        boatSide[3] = *(ui_->camera->getBoatSide())[3];

        geometry_msgs::PoseArray msg_boatside;
        msg_boatside.header.stamp = ros::Time::now();
        for(int i = 0; i < 4; i++){
            geometry_msgs::Pose pose_;
            pose_.position.x = boatSide[i].x();
            pose_.position.y = boatSide[i].y();
            msg_boatside.poses.push_back(pose_);
        }

        boatside_pub.publish(msg_boatside);

    }
}

void CameraPanel::setRightBoat(){
    if(ui_->buttonRightBoat->text() == "R"){
        ui_->camera->changeSetMode(4);
        ui_->buttonRightBoat->setText("Done");
        ui_->buttonLeftBoat->setDisabled(true);
    }
    else{
        ui_->camera->changeSetMode(0);
        ui_->buttonRightBoat->setText("R");
        ui_->buttonLeftBoat->setDisabled(false);
        Q_EMIT boatSideChanged(ui_->camera->getBoatSide());
    }
}

void CameraPanel::setLeftBoat2(){
    if(ui_->buttonLeftBoat_2->text() == "L"){
        ui_->camera->changeSetMode(5);
        ui_->buttonLeftBoat_2->setText("Done");
        ui_->buttonRightBoat_2->setDisabled(true);
    }
    else{
        ui_->camera->changeSetMode(0);
        ui_->buttonLeftBoat_2->setText("L");
        ui_->buttonRightBoat_2->setDisabled(false);
        Q_EMIT boatSideChanged2(ui_->camera->getBoatSide2());
    }
}

void CameraPanel::setRightBoat2(){
    if(ui_->buttonRightBoat_2->text() == "R"){
        ui_->camera->changeSetMode(6);
        ui_->buttonRightBoat_2->setText("Done");
        ui_->buttonLeftBoat_2->setDisabled(true);
    }
    else{
        ui_->camera->changeSetMode(0);
        ui_->buttonRightBoat_2->setText("R");
        ui_->buttonLeftBoat_2->setDisabled(false);
        Q_EMIT boatSideChanged2(ui_->camera->getBoatSide2());
    }
}

void CameraPanel::setCritLine(){
    if(ui_->buttonCritLine->text() == "CritLine"){
        ui_->camera->changeSetMode(1);
        ui_->buttonCritLine->setText("Done");
    }
    else{
        ui_->camera->changeSetMode(0);
        ui_->buttonCritLine->setText("CritLine");
        Q_EMIT critLineChanged(0, ui_->camera->getCritLine(0));
        Q_EMIT critLineChanged(1, ui_->camera->getCritLine(1));

        std_msgs::Int32MultiArray msg_crt;
        // msg_crt.data.push_back(0);
        msg_crt.data.push_back(ui_->camera->getCritLine(1));
        msg_crt.data.push_back(ui_->camera->getCritLine(0));
        critline_pub.publish(msg_crt);
    }
}

void CameraPanel::changeCritLine(int i, int crit){
    ui_->camera->changeCritLine(i, crit);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(CameraPanel,rviz::Panel )