#include "mission_plugin.h"
#include "ui_mission.h"

MissionPanel::MissionPanel(QWidget* parent)
    : rviz::Panel(parent), ui(new Ui::Mission()){
        ui->setupUi(this);

        initPublishers();
        initSubscribers();
        initConnection();
}

void MissionPanel::initPublishers(){
    statePub    = nh.advertise<std_msgs::UInt16MultiArray>("/rviz_plugin/stateMission", 1);
    sequencePub = nh.advertise<std_msgs::String>("/sequenceMission", 1);
    wpnavPub    = nh.advertise<std_msgs::Int8>("/rviz_plugin/jumpWP", 1);
}

void MissionPanel::initSubscribers(){
    nextDestSub = nh.subscribe("/next_dest", 1, &MissionPanel::nextDestCallback, this);
}
    
void MissionPanel::initConnection(){
    connect(ui->startButton, &QPushButton::clicked, 
        [=](){
            ui->stopButton->setEnabled(true);
            ui->startButton->setDisabled(true);
            ui->setSeqButton->setDisabled(true);
            this->startMission(1);
        }
    );

    connect(ui->stopButton, &QPushButton::clicked, 
        [=](){
//            ui->stopButton->setDisabled(true);
            ui->startButton->setEnabled(true);
            ui->setSeqButton->setEnabled(true);
            this->startMission(0);
        }
    );

    connect(ui->continueButton, &QPushButton::clicked,
            [=](){
                ui->stopButton->setEnabled(true);
                ui->startButton->setDisabled(true);
                ui->setSeqButton->setDisabled(true);
                this->startMission(2);
            }
    );

    connect(ui->setSeqButton, &QPushButton::clicked,
        [=](){
            setSequence(ui->editMissionSeq->text().toUtf8().constData());
        });

    connect(ui->manual->overrideButton, &QPushButton::clicked, 
        [=](){
            ui->stopButton->setEnabled(true);
            // ui->startButton->setDisabled(true);
            // ui->setSeqButton->setDisabled(true);
            this->startMission(3);
        }
    );

    connect(ui->nextWPButton, &QPushButton::clicked,
            [=](){
                this->publishWPNav(1);
            }
    );

    connect(ui->prevWPButton, &QPushButton::clicked,
            [=](){
                this->publishWPNav(-1);
            }
    );

    connect(ui->rth_pushButton, &QPushButton::clicked, [=](){
        QMessageBox::StandardButton reply;
        reply = QMessageBox::question(this, "RTH CONFIRMATION", "RTH?",
                                      QMessageBox::Yes|QMessageBox::No);
        if (reply == QMessageBox::Yes) {
            std::cout << "Yes was clicked\n";
            this->startMission(11);
        } else {
            std::cout << "Yes was *not* clicked\n";
        }
    });
}

void MissionPanel::startMission(int state){
    std_msgs::UInt16MultiArray msg;
    msg.data.push_back(state);
    if(state==1) // if clicked button is 'START'
        msg.data.push_back(ui->startAtspinBox->value());
    else
        msg.data.push_back(-1);
    statePub.publish(msg);
}

void MissionPanel::setSequence(std::string seq){
    std_msgs::String msg;
    msg.data = seq;
    sequencePub.publish(msg);
}

void MissionPanel::publishWPNav(int8_t nav) {
    std_msgs::Int8 msg;
    msg.data = nav;
    wpnavPub.publish(msg);
}

void MissionPanel::nextDestCallback(const rviz_plugin::NextDest& msg){
    ui->missionLcd->display(msg.misi);
    ui->wpLcd->display(msg.wp_index);
    ui->actionLcd->display(msg.action_wp);
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(MissionPanel,rviz::Panel )