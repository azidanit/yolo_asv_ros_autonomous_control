//
// Created by lumpia on 15/07/21.
//

#include "MainWindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
        QMainWindow(parent),
        ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    imgReceiver = new ImgReceiver();


    initConnectionPID();
    initConnection();
    initConnectionTrim();
    initConnectionMission();
    initPathRecorder();
    initUi();

    imgReceiver->start();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::initUi() {
    setInformationText("NALA RESCUER CONTROL OPENED");
    setLogText(QString("app loaded succesfully"));

    ui->doubleSpinBox_P->setValue(8);
    ui->doubleSpinBox_PD->setValue(10);
    ui->spinBox_Speed->setValue(250);

    emit boatSideChanged(ui->camera->getBoatSide());
    emit boatSideChanged2(ui->camera->getBoatSide2());
    emit critLineChanged(0, ui->camera->getCritLine(0));
    emit critLineChanged(1, ui->camera->getCritLine(1));
}

void MainWindow::initConnection() {
    connect(ui->startmanualButton, &QPushButton::clicked,
            [=](){
                std::cout << "Start Mission clicked" << std::endl;

                ui->startmanualButton->setEnabled(false);
                ui->stopmanualButton->setEnabled(true);
                ui->resumemanualButton->setEnabled(false);
                emit startMissionClicked();
            });

    connect(ui->stopmanualButton, &QPushButton::clicked,
            [=](){
                std::cout << "Stop Mission clicked" << std::endl;

                ui->startmanualButton->setEnabled(true);
                ui->stopmanualButton->setEnabled(false);
                ui->resumemanualButton->setEnabled(true);
                emit stopMissionClicked();
            });

    connect(ui->resumemanualButton, &QPushButton::clicked,
            [=](){
                std::cout << "Stop Mission clicked" << std::endl;

                ui->startmanualButton->setEnabled(false);
                ui->stopmanualButton->setEnabled(true);
                ui->resumemanualButton->setEnabled(false);
                emit resumeMissionClicked();
            });

    connect(ui->actionSave, &QAction::triggered,
            [=](){
                QString fileName = QFileDialog::getSaveFileName(this,
                                                                ("Save All Setting"), QDir::homePath() + QString("/nala_rescuer/data/params/params.json"),
                                                                ("JSON (*.json)"));;

                if (fileName.isEmpty()) {
                    return;
                }else{
                    saveParams(fileName);
                }
            });

    connect(ui->actionOpen, &QAction::triggered,
            [=](){
                qDebug()<<"clicked load";
                QString fileName = QFileDialog::getOpenFileName(this,
                                                                tr("Open All Setting"), QDir::homePath() + QString("/nala_rescuer/data/params/"),
                                                                tr("JSON (*.json);;All Files (*)"));

                if (fileName.isEmpty()) {
                    return;
                }else{
                    loadParams(fileName);
                }
            });

//    connect(imgReceiver, &ImgReceiver::cobasignal, this, [=](int value){
//        std::cout << "DAPAT VALUE " << value << "\n";
//    },Qt::DirectConnection);
//
    connect(imgReceiver, &ImgReceiver::imageReceived,
            this,
            [=](QImage image){
                mtx_img.lock();
                ui->camera->drawPixmap(&image);
//            delete image;
                mtx_img.unlock();
//            emit sendCameraImage(image);
            },Qt::DirectConnection);

    // Boat Side and CritLine
    connect(ui->buttonLeftBoat, &QPushButton::clicked, this, &MainWindow::setLeftBoat);
    connect(ui->buttonRightBoat, &QPushButton::clicked, this, &MainWindow::setRightBoat);
    connect(ui->buttonLeftBoat_2, &QPushButton::clicked, this, &MainWindow::setLeftBoat2);
    connect(ui->buttonRightBoat_2, &QPushButton::clicked, this, &MainWindow::setRightBoat2);
    connect(ui->buttonCritLine, &QPushButton::clicked, this, &MainWindow::setCritLine);

    connect(ui->buttonCritLine2,&QPushButton::clicked,this,&MainWindow::setCritLine2);
    connect(ui->buttonLeftBoat_3, &QPushButton::clicked, this, &MainWindow::setLeftBoat3);
    connect(ui->buttonRightBoat_3, &QPushButton::clicked, this, &MainWindow::setRightBoat3);
}

void MainWindow::initPathRecorder(){
    connect(ui->startPathButton, &QPushButton::clicked,
            [=](){
                std::cout << "Start record path clicked" << std::endl;

                ui->startPathButton->setDisabled(true);
                ui->stopPathButton->setEnabled(true);
                emit startRecordPath();
            });

    connect(ui->stopPathButton, &QPushButton::clicked,
            [=](){
                std::cout << "Start record path clicked" << std::endl;
                ui->startPathButton->setEnabled(true);
                ui->stopPathButton->setDisabled(true);
                emit stopRecordPath();
            });

    connect(ui->clearPathButton, &QPushButton::clicked,
            [=](){
                emit clearPath();
            });

    connect(ui->savePathButton, &QPushButton::clicked,
            [=](){
                QString fileName = QFileDialog::getSaveFileName(this,
                                                                ("Save All Setting"), QDir::homePath() + QString("/nala_rescuer/data/gps/gps.csv"),
                                                                ("CSV (*.csv)"));;

                if (fileName.isEmpty()) {
                    return;
                }else{
                    emit savePath(fileName);
                }
            });

    connect(ui->loadPathButton, &QPushButton::clicked,
            [=](){
                qDebug()<<"clicked load";
                QString fileName = QFileDialog::getOpenFileName(this,
                                                                tr("Open All Setting"), QDir::homePath() + QString("/nala_rescuer/data/gps/gps.csv"),
                                                                tr("CSV (*.csv);;All Files (*)"));

                if (fileName.isEmpty()) {
                    return;
                }else{
                    emit loadPath(fileName);
                }
            });

    connect(ui->loadLocalPathButton, &QPushButton::clicked,
            [=](){
                qDebug()<<"clicked load";
                QString fileName = QFileDialog::getOpenFileName(this,
                                                                tr("Open All Setting"), QDir::homePath() + QString("/nala_rescuer/data/gps/gps.csv"),
                                                                tr("LOCALPATH (*.csvlocal);;All Files (*)"));

                if (fileName.isEmpty()) {
                    return;
                }else{
                    emit loadLocalPath(fileName);
                }
            });
}

void MainWindow::initConnectionMission() {
    connect(ui->RTH_after_done_checkBox, &QCheckBox::stateChanged, [=](bool val){ emit changedRTHAfterDone(val); });
    connect(ui->patrol_lap_spinBox, static_cast<void (QSpinBox::*)(int)> (&QSpinBox::valueChanged), [=](int value){ emit changedPatrolLap(value); });

}

void MainWindow::initConnectionPID(){
    connect(ui->doubleSpinBox_P, static_cast<void (QDoubleSpinBox::*)(double)> (&QDoubleSpinBox::valueChanged), [=](double value){ emit pChanged(0, value); });
    connect(ui->doubleSpinBox_I, static_cast<void (QDoubleSpinBox::*)(double)> (&QDoubleSpinBox::valueChanged), [=](double value){ emit iChanged(0, value); });
    connect(ui->doubleSpinBox_D, static_cast<void (QDoubleSpinBox::*)(double)> (&QDoubleSpinBox::valueChanged), [=](double value){ emit dChanged(0, value); });
    connect(ui->spinBox_Speed, static_cast<void (QSpinBox::*)(int)> (&QSpinBox::valueChanged), [=](int value){ emit speedChanged(0, value); });
    connect(ui->doubleSpinBox_PD, static_cast<void (QDoubleSpinBox::*)(double)> (&QDoubleSpinBox::valueChanged), [=](double value){ emit pChanged(1, value); });
    connect(ui->doubleSpinBox_ID, static_cast<void (QDoubleSpinBox::*)(double)> (&QDoubleSpinBox::valueChanged), [=](double value){ emit iChanged(1, value); });
    connect(ui->doubleSpinBox_DD, static_cast<void (QDoubleSpinBox::*)(double)> (&QDoubleSpinBox::valueChanged), [=](double value){ emit dChanged(1, value); });
    connect(ui->doubleSpinBox_Psp, static_cast<void (QDoubleSpinBox::*)(double)> (&QDoubleSpinBox::valueChanged), [=](double value){ emit pChanged(2, value); });
    connect(ui->doubleSpinBox_Isp, static_cast<void (QDoubleSpinBox::*)(double)> (&QDoubleSpinBox::valueChanged), [=](double value){ emit iChanged(2, value); });
    connect(ui->doubleSpinBox_Dsp, static_cast<void (QDoubleSpinBox::*)(double)> (&QDoubleSpinBox::valueChanged), [=](double value){ emit dChanged(2, value); });
    connect(ui->doubleSpinBox_Pld, static_cast<void (QDoubleSpinBox::*)(double)> (&QDoubleSpinBox::valueChanged), [=](double value){ emit pChanged(3, value); });
    connect(ui->doubleSpinBox_Ild, static_cast<void (QDoubleSpinBox::*)(double)> (&QDoubleSpinBox::valueChanged), [=](double value){ emit iChanged(3, value); });
    connect(ui->doubleSpinBox_Dld, static_cast<void (QDoubleSpinBox::*)(double)> (&QDoubleSpinBox::valueChanged), [=](double value){ emit dChanged(3, value); });
    connect(ui->doubleSpinBox_Pcam, static_cast<void (QDoubleSpinBox::*)(double)> (&QDoubleSpinBox::valueChanged), [=](double value){ emit pChanged(4, value); });
    connect(ui->doubleSpinBox_Icam, static_cast<void (QDoubleSpinBox::*)(double)> (&QDoubleSpinBox::valueChanged), [=](double value){ emit iChanged(4, value); });
    connect(ui->doubleSpinBox_Dcam, static_cast<void (QDoubleSpinBox::*)(double)> (&QDoubleSpinBox::valueChanged), [=](double value){ emit dChanged(4, value); });
    connect(ui->doubleSpinBox_Pcam_thrust, static_cast<void (QDoubleSpinBox::*)(double)> (&QDoubleSpinBox::valueChanged), [=](double value){ emit pChanged(5, value); });
    connect(ui->doubleSpinBox_Icam_thrust, static_cast<void (QDoubleSpinBox::*)(double)> (&QDoubleSpinBox::valueChanged), [=](double value){ emit iChanged(5, value); });
    connect(ui->doubleSpinBox_Dcam_thrust, static_cast<void (QDoubleSpinBox::*)(double)> (&QDoubleSpinBox::valueChanged), [=](double value){ emit dChanged(5, value); });
    connect(ui->doubleSpinBox_PBr, static_cast<void (QDoubleSpinBox::*)(double)> (&QDoubleSpinBox::valueChanged), [=](double value){ emit pChanged(-4, value); });
    connect(ui->doubleSpinBox_IBr, static_cast<void (QDoubleSpinBox::*)(double)> (&QDoubleSpinBox::valueChanged), [=](double value){ emit iChanged(-4, value); });
    connect(ui->doubleSpinBox_DBr, static_cast<void (QDoubleSpinBox::*)(double)> (&QDoubleSpinBox::valueChanged), [=](double value){ emit dChanged(-4, value); });

    connect(ui->speedControlCheckbox, &QCheckBox::stateChanged, [=](bool val){ emit useSpeedControl(val); });

    connect(ui->checkBoxUsingAccError, &QCheckBox::stateChanged,
            [=](bool val){
                emit useAccError(val);
            });


    connect(ui->doubleSpinBox_ASV_width, static_cast<void (QDoubleSpinBox::*)(double)> (&QDoubleSpinBox::valueChanged), this, &MainWindow::changedASVWidth);
    connect(ui->doubleSpinBox_ASV_offset, static_cast<void (QDoubleSpinBox::*)(double)> (&QDoubleSpinBox::valueChanged), this, &MainWindow::changedASVOffset);
    connect(ui->doubleSpinBox_Distance_save, static_cast<void (QDoubleSpinBox::*)(double)> (&QDoubleSpinBox::valueChanged), [=](double value){ emit distanceChanged(0, value); });
    connect(ui->pitch_threshold_doubleSpinBox, static_cast<void (QDoubleSpinBox::*)(double)> (&QDoubleSpinBox::valueChanged), [=](double value){ emit pitchChanged(0, value); });

    connect(ui->obstacleStopCheckbox, &QCheckBox::stateChanged, this, &MainWindow::changeUseObstacleStop);
    connect(ui->obstacleAvoidanceCheckbox, &QCheckBox::stateChanged, this, &MainWindow::changeUseObstacleAvoidance);
    connect(ui->find_korban_checkBox, &QCheckBox::stateChanged, this, &MainWindow::changeUseFindKorban);

    connect(ui->using_compass_yaw_checkbox, &QCheckBox::stateChanged, [=](bool val){ emit changedUseCompass(val); });


}

void MainWindow::initConnectionTrim() {
    connect(ui->compass_offset_doubleSpinBox, static_cast<void (QDoubleSpinBox::*)(double)> (&QDoubleSpinBox::valueChanged), this, &MainWindow::compassOffsetChanged);


    connect(ui->trim_servo_spinBox, static_cast<void (QSpinBox::*)(int)> (&QSpinBox::valueChanged),
            [=](int value){
                emit steerTrimChanged(value);
            }
    );

    connect(ui->trim_thrust_spinBox, static_cast<void (QSpinBox::*)(int)> (&QSpinBox::valueChanged),
            [=](int value){
                emit speedTrimChanged(value);
            }
    );

    connect(ui->trimleft_PushButton, &QPushButton::clicked,
            [=](){
                ui->trim_servo_spinBox->setValue(ui->trim_servo_spinBox->value() + STEERSTEP);
                std::cout << "steering trim " << ui->trim_servo_spinBox->value() << std::endl;
            }
    );

    connect(ui->trimright_PushButton, &QPushButton::clicked,
            [=](){
                ui->trim_servo_spinBox->setValue(ui->trim_servo_spinBox->value() - STEERSTEP);
                std::cout << "steering trim " << ui->trim_servo_spinBox->value() << std::endl;
            }
    );

    connect(ui->trimspeedup_PushButton, &QPushButton::clicked,
            [=](){
                ui->trim_thrust_spinBox->setValue(ui->trim_thrust_spinBox->value() + SPEEDSTEP);
                std::cout << "thruster trim " << ui->trim_thrust_spinBox->value() << std::endl;
            }
    );

    connect(ui->trimspeeddown_PushButton, &QPushButton::clicked,
            [=](){
                ui->trim_thrust_spinBox->setValue(ui->trim_thrust_spinBox->value() - SPEEDSTEP);
                std::cout << "thruster trim " << ui->trim_thrust_spinBox->value() << std::endl;
            }
    );

    connect(ui->trimreset_PushButton, &QPushButton::clicked,
            [=](){
                ui->trim_thrust_spinBox->setValue(0);
                ui->trim_servo_spinBox->setValue(0);
                std::cout << "thruster trim " << ui->trim_thrust_spinBox->value() << std::endl;
                std::cout << "steering trim " << ui->trim_servo_spinBox->value() << std::endl;
            }
    );
}

void MainWindow::setInformationText(const char* info_text) {
    ui->info_screen_TextEdit->setText(info_text);
}

void MainWindow::setLogText(QString log_text) {
    ui->log_screen_TextEdit->setText(log_text);
}

void MainWindow::setSteerTrimFromRemote(int val){
    ui->trim_servo_spinBox->setValue(val);
}

void MainWindow::saveParams(QString filename) {
    std::cout << "Save params" << std::endl;
    QFile file(filename);

    if(!file.open(QIODevice::ReadWrite | QIODevice::Truncate))
        QMessageBox::information(this, tr("Can't Open File"), file.errorString());

    QJsonObject json;

    // PID Speed
    json["P"] = ui->doubleSpinBox_P->value();
    json["I"] = ui->doubleSpinBox_I->value();
    json["D"] = ui->doubleSpinBox_D->value();
    json["speed"] = ui->spinBox_Speed->value();

    json["PD"] = ui->doubleSpinBox_PD->value();
    json["ID"] = ui->doubleSpinBox_ID->value();
    json["DD"] = ui->doubleSpinBox_DD->value();

    json["Psp"] = ui->doubleSpinBox_Psp->value();
    json["Isp"] = ui->doubleSpinBox_Isp->value();
    json["Dsp"] = ui->doubleSpinBox_Dsp->value();

    json["speedcontrol"] = (int) (ui->speedControlCheckbox->isChecked());

    json["Pld"] = ui->doubleSpinBox_Pld->value();
    json["Ild"] = ui->doubleSpinBox_Ild->value();
    json["Dld"] = ui->doubleSpinBox_Dld->value();
    json["DstLd"] = ui->doubleSpinBox_Distance_save->value();
    json["K_err_lidar"] = ui->doubleSpinBox_Mtp_error_ld->value();
    json["asvwidth"] = ui->doubleSpinBox_ASV_width->value();
    json["asvoffset"] = ui->doubleSpinBox_ASV_offset->value();

    // Trim
    json["trim_steer"] = ui->trim_servo_spinBox->value();
    json["trim_speed"] = ui->trim_thrust_spinBox->value();

    QJsonDocument document(json);

    qDebug() << "json : " << json;
    file.write(document.toJson());

    file.close();


}

void MainWindow::loadParams(QString filename) {
    std::cout << "Load params" << std::endl;

    QFile file(filename);

    if(!file.open(QIODevice::ReadWrite))
        QMessageBox::information(this, tr("Can't Open File"), file.errorString());


    QByteArray saveData = file.readAll();
    QJsonDocument document(QJsonDocument::fromJson(saveData));
    QJsonObject json = document.object();

    // PID Speed
    ui->doubleSpinBox_P->setValue(json["P"].toDouble());
    ui->doubleSpinBox_I->setValue(json["I"].toDouble());
    ui->doubleSpinBox_D->setValue(json["D"].toDouble());
    ui->spinBox_Speed->setValue(json["speed"].toInt());

    ui->doubleSpinBox_PD->setValue(json["PD"].toDouble());
    ui->doubleSpinBox_ID->setValue(json["ID"].toDouble());
    ui->doubleSpinBox_DD->setValue(json["DD"].toDouble());

    ui->doubleSpinBox_Psp->setValue(json["Psp"].toDouble());
    ui->doubleSpinBox_Isp->setValue(json["Isp"].toDouble());
    ui->doubleSpinBox_Dsp->setValue(json["Dsp"].toDouble());

    ui->doubleSpinBox_Pld->setValue(json["Pld"].toDouble());
    ui->doubleSpinBox_Ild->setValue(json["Ild"].toDouble());
    ui->doubleSpinBox_Dld->setValue(json["Dld"].toDouble());
    ui->doubleSpinBox_Distance_save->setValue(json["DstLd"].toDouble());
    ui->doubleSpinBox_Mtp_error_ld->setValue(json["K_err_lidar"].toDouble());
    ui->doubleSpinBox_ASV_width->setValue(json["asvwidth"].toDouble());
    ui->doubleSpinBox_ASV_offset->setValue(json["asvoffset"].toDouble());

    ui->speedControlCheckbox->setChecked(json["speedcontrol"].toInt());

    // Trim
    ui->trim_servo_spinBox->setValue(json["trim_steer"].toInt());
    ui->trim_thrust_spinBox->setValue(json["trim_speed"].toInt());


    file.close();
}

void MainWindow::setLeftBoat(){
    if(ui->buttonLeftBoat->text() == "L"){
        ui->camera->changeSetMode(3);
        ui->buttonLeftBoat->setText("Done");
        ui->buttonRightBoat->setDisabled(true);
    }
    else{
        ui->camera->changeSetMode(0);
        ui->buttonLeftBoat->setText("L");
        ui->buttonRightBoat->setDisabled(false);
        emit boatSideChanged(ui->camera->getBoatSide());
    }
}

void MainWindow::setRightBoat(){
    if(ui->buttonRightBoat->text() == "R"){
        ui->camera->changeSetMode(4);
        ui->buttonRightBoat->setText("Done");
        ui->buttonLeftBoat->setDisabled(true);
    }
    else{
        ui->camera->changeSetMode(0);
        ui->buttonRightBoat->setText("R");
        ui->buttonLeftBoat->setDisabled(false);
        emit boatSideChanged(ui->camera->getBoatSide());
    }
}

void MainWindow::setLeftBoat2(){
    if(ui->buttonLeftBoat_2->text() == "L"){
        ui->camera->changeSetMode(5);
        ui->buttonLeftBoat_2->setText("Done");
        ui->buttonRightBoat_2->setDisabled(true);
    }
    else{
        ui->camera->changeSetMode(0);
        ui->buttonLeftBoat_2->setText("L");
        ui->buttonRightBoat_2->setDisabled(false);
        emit boatSideChanged2(ui->camera->getBoatSide2());
    }
}

void MainWindow::setRightBoat2(){
    if(ui->buttonRightBoat_2->text() == "R"){
        ui->camera->changeSetMode(6);
        ui->buttonRightBoat_2->setText("Done");
        ui->buttonLeftBoat_2->setDisabled(true);
    }
    else{
        ui->camera->changeSetMode(0);
        ui->buttonRightBoat_2->setText("R");
        ui->buttonLeftBoat_2->setDisabled(false);
        emit boatSideChanged2(ui->camera->getBoatSide2());
    }
}
void MainWindow::setLeftBoat3(){
    if(ui->buttonLeftBoat_3->text() == "L"){
        ui->camera->changeSetMode(7);
        ui->buttonLeftBoat_3->setText("Done");
        ui->buttonRightBoat_3->setDisabled(true);
    }
    else{
        ui->camera->changeSetMode(0);
        ui->buttonLeftBoat_3->setText("L");
        ui->buttonRightBoat_3->setDisabled(false);
        emit boatSideChanged3(ui->camera->getBoatSide3());
    }
}

void MainWindow::setRightBoat3(){
    if(ui->buttonRightBoat_3->text() == "R"){
        ui->camera->changeSetMode(8);
        ui->buttonRightBoat_3->setText("Done");
        ui->buttonLeftBoat_3->setDisabled(true);
    }
    else{
        ui->camera->changeSetMode(0);
        ui->buttonRightBoat_3->setText("R");
        ui->buttonLeftBoat_3->setDisabled(false);
        emit boatSideChanged3(ui->camera->getBoatSide3());
    }
}

void MainWindow::setCritLine(){
    if(ui->buttonCritLine->text() == "Edit"){
        ui->camera->changeSetMode(1);
        ui->buttonCritLine->setText("Done");
    }
    else{
        ui->camera->changeSetMode(0);
        ui->buttonCritLine->setText("Edit");
        emit critLineChanged(0, ui->camera->getCritLine(0));
        emit critLineChanged(1, ui->camera->getCritLine(1));
    }
}
void MainWindow::setCritLine2(){
    if(ui->buttonCritLine2->text() == "Edit"){
        ui->camera->changeSetMode(2);
        ui->buttonCritLine2->setText("Done");
    }
    else{
        ui->camera->changeSetMode(0);
        ui->buttonCritLine2->setText("Edit");
        emit critLineChanged2(0, ui->camera->getCritLine2(0));
        emit critLineChanged2(1, ui->camera->getCritLine2(1));
    }
}

void MainWindow::changeCritLine(int i, int crit){
    ui->camera->changeCritLine(i, crit);
}
void MainWindow::changeCritLine2(int i, int crit){
    ui->camera->changeCritLine2(i, crit);
}