/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */
/* Author: Dinesh Thakur - Modified for waypoint navigation */

#include <OGRE/OgreSceneManager.h>
#include <rviz/display_context.h>
#include <interactive_markers/interactive_marker_server.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <json.hpp>
#include <qjsonarray.h>
#include <qjsonobject.h>
#include <qjsondocument.h>
#include <qjsonvalue.h>

#include "waypoint_nav_frame.h"
#include "waypoint_nav_tool.h"

#include <iostream>
#include <string>
//#include "waypoint_nav_frame.h"
//msg wp
#include "rviz_plugin/Selectedwp.h"

#include <tf/tf.h>

#include <QFileDialog>

#include <boost/foreach.hpp>
#include <QtWidgets/QMessageBox>

#define foreach BOOST_FOREACH

using json = nlohmann::json;

namespace waypoint_nav_plugin
{

    WaypointFrame::WaypointFrame(rviz::DisplayContext *context, std::map<int, Ogre::SceneNode* >*
    map_ptr, interactive_markers::InteractiveMarkerServer* server, int* unique_ind, QWidget *parent,
      WaypointNavTool* wp_tool)
  : QWidget(parent)
  , context_(context)
  , ui_(new Ui::WaypointNavigationWidget())
  , sn_map_ptr_(map_ptr)
  , unique_ind_(unique_ind)
  , server_(server)
  , frame_id_("map")
  , default_height_(0.0)
  , selected_marker_name_("waypoint1")
  , wp_nav_tool_(wp_tool)
{
  scene_manager_ = context_->getSceneManager();

  sub_loaded_path = nh_.subscribe("/loaded_path_car", 1, &WaypointFrame::loadedPathSubsCallback, this);
//  sub_topic_path = nh_.subscribe(ui_->topic_path_line_edit->text().toStdString(), 1, &WaypointFrame::loadedPathTopicSubsCallback, this);

  // set up the GUI
  ui_->setupUi(this);

  missionSelected = 0;
has_loaded = false;

        //connect the Qt signals and slots
  connect(ui_->publish_wp_button, SIGNAL(clicked()), this, SLOT(publishButtonClicked()));
  connect(ui_->publish_all_wp_button, SIGNAL(clicked()), this, SLOT(publishAllButtonClicked()));
  connect(ui_->topic_line_edit, SIGNAL(editingFinished()), this, SLOT(topicChanged()));
  connect(ui_->frame_line_edit, SIGNAL(editingFinished()), this, SLOT(frameChanged()));
  connect(ui_->wp_height_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(heightChanged(double)));
  connect(ui_->clear_all_button, SIGNAL(clicked()), this, SLOT(clearAllWaypoints()));

  // connect(ui->missionComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
    // [=](int index), SLOT(missionChanged(index)));
  connect(ui_->missionComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),[=](int index){setMission(index);});
  connect(ui_->speed_spinBox, SIGNAL(valueChanged(double)), this, SLOT(changeSavedSpeed(double)));


  connect(ui_->x_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(poseChanged(double)));
  connect(ui_->y_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(poseChanged(double)));
  connect(ui_->z_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(poseChanged(double)));
  connect(ui_->yaw_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(speedChanged(int)));

  connect(ui_->save_wp_button, SIGNAL(clicked()), this, SLOT(saveButtonClicked()));
  connect(ui_->load_wp_button, SIGNAL(clicked()), this, SLOT(loadButtonClicked()));

  connect(ui_->make_wp_button, SIGNAL(clicked()), this, SLOT(makeWPButtonClicked()));

  connect(ui_->set_selected_sped_button, SIGNAL(clicked()), this, SLOT(setSelectedWPSpeed()));
  connect(ui_->set_speed_all_wp_button, SIGNAL(clicked()), this, SLOT(setAllWPSpeed()));

  connect(ui_->delete_wp_button, SIGNAL(clicked()), this, SLOT(deleteSelectedWP()));

  connect(ui_->set_servo_wp, SIGNAL(clicked()), this, SLOT(setServoWPClicked()));
  connect(ui_->set_action_wp, SIGNAL(clicked()), this, SLOT(setActionButtonClicked()));

    connect(ui_->generate_road_button, SIGNAL(clicked()), this, SLOT(generateRoadClicked()));
    connect(ui_->generate_calib_button,SIGNAL(clicked()), this, SLOT(generateMarkerCalibClicked()));
    connect(ui_->generate_calib_local_button,SIGNAL(clicked()), this, SLOT(generateMarkerCalibLocalClicked()));
    connect(ui_->venue_line_edit, SIGNAL(textChanged(const QString &)), this, SLOT(checkVenue(const QString &)));

    connect(ui_->make_wp_topic_button,SIGNAL(clicked()), this, SLOT(makeWPTopicButtonClicked()));
    connect(ui_->topic_path_line_edit, SIGNAL(editingFinished()), this, SLOT(topicPathChanged()));

    connect(ui_->insert_wp_pushButton, SIGNAL(clicked()), this, SLOT(insertWPClicked()));
    connect(ui_->open_default_pushButton, SIGNAL(clicked()), this, SLOT(loadLastSetting()));

    //init paths & publisher
  for(int i =0;i<4;i++){
    paths.push_back(nav_msgs::Path());
  }

  ui_->missionComboBox->setCurrentIndex(1);
  missionSelected = 1;
  setMission(1);
  // sn_maps_saved.push_back(std::map<int, Ogre::SceneNode>());
  // sn_maps_saved.push_back(std::map<int, Ogre::SceneNode>());
  // sn_maps_saved.push_back(std::map<int, Ogre::SceneNode>());
  ui_->status_venue->setText("");

  //load last config
//  loadLastSetting();
  has_loaded = true;

}

WaypointFrame::~WaypointFrame()
{
  delete ui_;
  sn_map_ptr_ = NULL;
}

void WaypointFrame::enable()
{
  // activate the frame
  show();
}

void WaypointFrame::disable()
{
  wp_pub_.shutdown();
  hide();
}

void WaypointFrame::saveButtonClicked() {
    setMission(missionSelected);

    QString filename;
    filename = QFileDialog::getSaveFileName(this,
                                            "Save Waypoints",                           // caption
                                            QDir::homePath() + "/nala_rescuer/data/wp",                                    // directory
                                            "JSON (*.json)"                             // options
    );

    saveSetting(filename);
}

void WaypointFrame::saveSetting(QString filename) {
    if (filename.isEmpty()) {
        return;
    } else {
        std::ofstream file(filename.toStdString());

        json allPathsMission;
        for (int i = 1; i < paths.size(); i++) { //save wp per mission
            nav_msgs::Path *tempPath = &paths[i];

//            QJsonObject pathObjectJSON;
            json pathDetail;
            for (int a = 0; a < tempPath->poses.size(); a++) { //one path
                json poseDetail;
                poseDetail["x"] = tempPath->poses[a].pose.position.x;
                poseDetail["y"] = tempPath->poses[a].pose.position.y;
                poseDetail["z"] = tempPath->poses[a].pose.position.z;
                poseDetail["ox"] = tempPath->poses[a].pose.orientation.x;
                poseDetail["oy"] = tempPath->poses[a].pose.orientation.y;
                poseDetail["oz"] = tempPath->poses[a].pose.orientation.z;
                poseDetail["ow"] = tempPath->poses[a].pose.orientation.w;
                poseDetail["speed"] = speed[i][a];
                poseDetail["action"] = action_wp[i][a];
                poseDetail["servo"] = servo_action_wp[i][a];
                //std::cout << "SPEED SAVE " << speed[i][a] << std::endl;
                //std::cout << "XX " << tempPath->poses[a].pose.position.x << std::endl;
//                QJsonObject poseObjectJSON;
//                pathObjectJSON["x"] =  tempPath->poses->pose
                pathDetail.push_back(poseDetail);
            }
            allPathsMission.push_back(pathDetail);
        }

        //std::cout << "JSON " << allPathsMission << std::endl;

        file << allPathsMission;

    }
}

void WaypointFrame::setServoWPClicked() {
    int from_wp = ui_->servo_from_wp_spinbox->value();
    int to_wp = ui_->servo_to_wp_spinbox->value();
    for (int i = from_wp-1; i < to_wp; i++){
        servo_action_wp[missionSelected][i] = ui_->servo_wp_spinbox->value();
    }
}

void WaypointFrame::loadButtonClicked(){
    QString filename;
    QString buffer;
    filename = QFileDialog::getOpenFileName(this,
                                            "Load Waypoint",
                                            QDir::homePath() + "/nala_rescuer/data/wp",
                                            "JSON(*.json)"
    );

    loadSetting(filename);
}

void WaypointFrame::loadLastSetting() {
//        sleep(3);
    std::string path = ros::package::getPath("rviz_plugin");
    path += "/setting/default_setting.json";
    struct stat buffer;
    has_loaded = false;
    if(stat(path.c_str(), &buffer) == 0) {
        loadSetting(path.c_str());
    }

}

void WaypointFrame::saveLastSetting() {
    std::string path = ros::package::getPath("rviz_plugin");
    path += "/setting/default_setting.json";
    saveSetting(path.c_str());
}

void WaypointFrame::loadSetting(QString filename){
//        std::cout << "LOAD SETTING " << has_loaded <<"\n";
    QString buffer;
    if (filename .isEmpty()) {
        return;
    }
    else {
        has_loaded = false;
        ui_->missionComboBox->setCurrentIndex(0);
        setMission(0);
        missionSelected = 0;

        std::ifstream openFile(filename.toStdString());
        json allPathMissionLoad;
        openFile >> allPathMissionLoad;
        paths[0].poses.clear();
        action_wp[0].clear();
//        missionSelected = 1;
        for(int i = 0; i < allPathMissionLoad.size(); i++){
            if(!allPathMissionLoad[i].is_null()){
//                std::cout<<"LOAD CUY "<<allPathMissionLoad[i]<<std::endl;
                json pathDetail = allPathMissionLoad[i];
//                missionSelected = i+1;
                paths[i+1].poses.clear();
                action_wp[i+1].clear();

                speed[i+1].clear();
                // missionSelected = (i+1);
                for(int a = 0; a <pathDetail.size(); a++){
                    json poseDetail = pathDetail[a];

//                    std::cout<<"POSE "<<a<<" "<<poseDetail["x"]<<std::endl;

                    geometry_msgs::PoseStamped pos;
                    pos.pose.position.x = poseDetail["x"];
                    pos.pose.position.y = poseDetail["y"];
                    pos.pose.position.z = poseDetail["z"];

//                    std::cout<<"mau Masuk ox\n";
                    pos.pose.orientation.x = poseDetail["ox"];
//                    std::cout<<"Masuk ox\n";
                    pos.pose.orientation.y = poseDetail["oy"];
//                    std::cout<<"Masuk oy\n";
                    pos.pose.orientation.z = poseDetail["oz"];
                    pos.pose.orientation.w = poseDetail["ow"];
                    sel_wp = a+1;
                    paths[i+1].poses.push_back(pos);

                    speed[i+1].push_back(double(poseDetail["speed"]));
                    action_wp[i+1].push_back(double(poseDetail["action"]));
                    servo_action_wp[i+1].push_back(double(poseDetail["servo"]));
//                    ui_->speed_spinBox->setValue(int(poseDetail["speed"]));
//                    // changeSavedSpeed(a+1);
//
//                    //
//                    setSpeed(a+1);

//                    std::cout<<"i + 1 "<<i+1<<std::endl;
                }
            }
        }

    }

    for(int i =1; i<4;i++){
        drawPath(i);
    }

    has_loaded = true;
}

void WaypointFrame::insertWpFromNavTool(const Ogre::Vector3& pos, const Ogre::Quaternion& quart) {
    if (missionSelected == 0){
        clearAllWaypoints();
        if(ui_->road_selector_slider->value() == 1){ //left Road
//            setMission(1);
            int index_to_insert = ui_->inser_wp_spinBox->value();
            if (index_to_insert < paths[1].poses.size()){
                geometry_msgs::PoseStamped tmp_wp;
                tmp_wp.pose.position.x = pos.x;
                tmp_wp.pose.position.y = pos.y;
                tmp_wp.pose.position.z = pos.z;

                tmp_wp.pose.orientation.w = quart.w;
                tmp_wp.pose.orientation.x = quart.x;
                tmp_wp.pose.orientation.y = quart.y;
                tmp_wp.pose.orientation.z = quart.z;

                paths[1].poses.insert(paths[1].poses.begin() + index_to_insert, tmp_wp);
//                std::cout << "DIBUAT di" << 1 << " " << paths[1].poses.size() << "\n";
            }
        }else if(ui_->road_selector_slider->value() == 2){ //right road
//            setMission(2);
            int index_to_insert = ui_->inser_wp_spinBox->value();
            if (index_to_insert < paths[2].poses.size()){
                geometry_msgs::PoseStamped tmp_wp;
                tmp_wp.pose.position.x = pos.x;
                tmp_wp.pose.position.y = pos.y;
                tmp_wp.pose.position.z = pos.z;

                tmp_wp.pose.orientation.w = quart.w;
                tmp_wp.pose.orientation.x = quart.x;
                tmp_wp.pose.orientation.y = quart.y;
                tmp_wp.pose.orientation.z = quart.z;

                paths[2].poses.insert(paths[2].poses.begin() + index_to_insert, tmp_wp);
//                std::cout << "DIBUAT di" << missionSelected << " " << paths[2].poses.size() << "\n";
            }
        }
//        setMission(0);
//        missionSelected = 0;
        for(int i =1; i<4;i++){
            drawPath(i);
        }

    }else{
        int index_to_insert = ui_->inser_wp_spinBox->value();
//        std::cout << "MASUK INSERT WP CLICKED " << index_to_insert << "\n";
        if (index_to_insert < paths[missionSelected].poses.size()){
            geometry_msgs::PoseStamped tmp_wp;
            tmp_wp.pose.position.x = pos.x;
            tmp_wp.pose.position.y = pos.y;
            tmp_wp.pose.position.z = pos.z;

            tmp_wp.pose.orientation.w = quart.w;
            tmp_wp.pose.orientation.x = quart.x;
            tmp_wp.pose.orientation.y = quart.y;
            tmp_wp.pose.orientation.z = quart.z;

            paths[missionSelected].poses.insert(paths[missionSelected].poses.begin() + index_to_insert, tmp_wp);
//            std::cout << "DIBUAT di" << missionSelected << " " << paths[missionSelected].poses.size() << "\n";

        }
        clearAllWaypoints();
        drawPath(missionSelected);
    }



    ui_->inser_wp_spinBox->setValue(ui_->inser_wp_spinBox->value() + 1);
}

void WaypointFrame::insertWPClicked() {
    int index_to_insert = ui_->inser_wp_spinBox->value();
//    std::cout << "MASUK INSERT WP CLICKED " << index_to_insert << "\n";
    if (index_to_insert < paths[missionSelected].poses.size()){
        geometry_msgs::PoseStamped tmp_wp;
        tmp_wp.pose.position.x = paths[missionSelected].poses[index_to_insert].pose.position.x + 1;
        tmp_wp.pose.position.y = paths[missionSelected].poses[index_to_insert].pose.position.y + 1;
        tmp_wp.pose.position.z = 0;

        tmp_wp.pose.orientation.w = paths[missionSelected].poses[index_to_insert].pose.orientation.w;
        tmp_wp.pose.orientation.x = paths[missionSelected].poses[index_to_insert].pose.orientation.x;
        tmp_wp.pose.orientation.y = paths[missionSelected].poses[index_to_insert].pose.orientation.y;
        tmp_wp.pose.orientation.z = paths[missionSelected].poses[index_to_insert].pose.orientation.z;

        paths[missionSelected].poses.insert(paths[missionSelected].poses.begin() + index_to_insert, tmp_wp);
//        std::cout << "DIBUAT di" << missionSelected << " " << paths[missionSelected].poses.size() << "\n";

    }
    clearAllWaypoints();
    drawPath(missionSelected);
    //refresh screen
//    setMission(0);
//    setMission(missionSelected);
}

void WaypointFrame::generateMarkerCalibLocalClicked() {
    QString filename_save;
    filename_save = QFileDialog::getSaveFileName(this,
                                                 "Save Marker Callb Local",                           // caption
                                                 QDir::homePath() + "/nala_rescuer/src/roadgenerator/saves",                                    // directory
                                                 "txt (*.txt)"                             // options
    );

    int action;
    FILE *file_save = fopen(filename_save.toStdString().c_str(), "w");

    if (!file_save)
        return;

    for(int i = 0; i < paths[3].poses.size() ; i++){
        geometry_msgs::PoseStamped marker_callib = paths[3].poses[i];
        fprintf(file_save, "%lf %lf %d\n", marker_callib.pose.position.x, marker_callib.pose.position.y, action);
    }

    fclose(file_save);
}

void WaypointFrame::generateMarkerCalibClicked(){
    double LatitudeTop, LongitudeLeft, LatitudeBot, LongitudeRight;
    double lat_local_center, long_local_center;

    std::string filename = ros::package::getPath("map_image") + "/resource/" + ui_->venue_line_edit->text().toStdString() + ".txt";
    FILE *file = fopen(filename.c_str(), "r");
    if (!file)
        return;

    char temp[100];
    fscanf(file, "%lf, %lf %lf, %lf", &LatitudeTop, &LongitudeLeft, &LatitudeBot, &LongitudeRight);
    fscanf(file, "%s", temp);
    fclose(file);

    lat_local_center = (LatitudeBot + LatitudeTop) / 2.0;
    long_local_center = (LongitudeLeft + LongitudeRight) / 2.0;

    QString filename_save;
    filename_save = QFileDialog::getSaveFileName(this,
                                                 "Save Marker Callb",                           // caption
                                                 QDir::homePath() + "/nala_rescuer/src/roadgenerator/saves",                                    // directory
                                                 "txt (*.txt)"                             // options
    );

    double lat_marker, long_marker;
    int action;
    FILE *file_save = fopen(filename_save.toStdString().c_str(), "w");

    if (!file_save)
        return;

    for(int i = 0; i < paths[3].poses.size() ; i++){
        geometry_msgs::PoseStamped marker_callib = paths[3].poses[i];
        lat_marker = lat_local_center - (marker_callib.pose.position.y / LAT_TO_METER);
        long_marker = long_local_center - (marker_callib.pose.position.x / LON_TO_METER);
        action = action_wp[3][i];
        printf("%lf %lf %d\n", lat_marker, long_marker, action);
        fprintf(file_save, "%lf %lf %d\n", lat_marker, long_marker, action);
    }

    fclose(file_save);
}

void WaypointFrame::generateRoadClicked() {
    double LatitudeTop, LongitudeLeft, LatitudeBot, LongitudeRight;
    double lat_local_center, long_local_center;

    std::string filename = ros::package::getPath("map_image") + "/resource/" + ui_->venue_line_edit->text().toStdString() + ".txt";
    FILE *file = fopen(filename.c_str(), "r");
    if (!file)
        return;

    char temp[100];
    fscanf(file, "%lf, %lf %lf, %lf", &LatitudeTop, &LongitudeLeft, &LatitudeBot, &LongitudeRight);
    fscanf(file, "%s", temp);
    fclose(file);

    lat_local_center = (LatitudeBot + LatitudeTop) / 2.0;
    long_local_center = (LongitudeLeft + LongitudeRight) / 2.0;

    QString filename_save;
    filename_save = QFileDialog::getSaveFileName(this,
                                                 "Save Waypoints",                           // caption
                                                 QDir::homePath() + "/nala_rescuer/src/roadgenerator/saves",                                    // directory
                                                 "txt (*.txt)"                             // options
    );


    double left_lat,  left_long, right_lat,  right_long;
    int action;
    FILE *file_save = fopen(filename_save.toStdString().c_str(), "w");

    if (!file_save)
        return;

    for(int i = 0; i < paths[1].poses.size() ; i++){
        geometry_msgs::PoseStamped left_road = paths[1].poses[i];
        geometry_msgs::PoseStamped right_road = paths[2].poses[i];
        left_lat = lat_local_center - (left_road.pose.position.y / LAT_TO_METER);
        left_long = long_local_center - (left_road.pose.position.x / LON_TO_METER);
        right_lat = lat_local_center - (right_road.pose.position.y / LAT_TO_METER);
        right_long = long_local_center - (right_road.pose.position.x / LON_TO_METER);
        action = action_wp[1][i];
        printf("%lf %lf %lf %lf %d\n", left_lat, left_long, right_lat, right_long, action);
        fprintf(file_save, "%lf %lf %lf %lf %d\n", left_lat, left_long, right_lat, right_long, action);
    }

    fclose(file_save);
}

void WaypointFrame::checkVenue(QString venue_check) {
    std::string filename = ros::package::getPath("map_image") + "/resource/" + venue_check.toStdString() + ".txt";
    std::string filename_img = ros::package::getPath("map_image") + "/resource/" + venue_check.toStdString() + ".png";
    struct stat buffer;
    if(stat(filename.c_str(), &buffer) == 0){
        ui_->status_venue->setStyleSheet("color:green;");
        ui_->status_venue->setText("Venue Found!");
        int w = ui_->image_prev_venue->width();
        int h = ui_->image_prev_venue->height();
        ui_->image_prev_venue->setPixmap(QPixmap(filename_img.c_str()).scaled(w,h,Qt::KeepAspectRatio));

    }else{
        ui_->status_venue->setStyleSheet("color:red;");
        ui_->status_venue->setText("Venue Not Found!");
        ui_->image_prev_venue->setPixmap(QPixmap());
    }
}

void WaypointFrame::publishButtonClicked()
{
//  session(missionSelected);
    setPath();
  paths[missionSelected].header.
  frame_id = frame_id_.toStdString();
  rviz_plugin::Selectedwp msg;
  msg.missionSelected = missionSelected;
//  msg.speed.clear();

  for (int i = 0; i < paths[missionSelected].poses.size(); i++){
    msg.speed.push_back(speed[missionSelected][i]);
    msg.actionwp.push_back(action_wp[missionSelected][i]);
    msg.servowp.push_back(servo_action_wp[missionSelected][i]);
//    std::cout << "s: " << speed[missionSelected][i] << " ";
  }


//  std::cout << std::endl;
  msg.wp = paths[missionSelected];
  wp_pub_.publish(msg);
}

void WaypointFrame::publishAllButtonClicked(){
  int last_selected_mission = missionSelected;

  for (int i=1; i <=3; i++){
    setMission(i);
    usleep(100);
    publishButtonClicked();
    usleep(1000);
    //std::cout << "publish misi: " << missionSelected << std::endl;
  }

  missionSelected = last_selected_mission;
}

void WaypointFrame::makeWPTopicButtonClicked() {
    for (int i = 0; i < loaded_topic_path.poses.size(); i++){
        Ogre::Vector3 position;
        position.x = loaded_topic_path.poses[i].pose.position.x;
        position.y = loaded_topic_path.poses[i].pose.position.y;
        position.z = loaded_topic_path.poses[i].pose.position.z;

        Ogre::Quaternion quat;
        quat.x = loaded_topic_path.poses[i].pose.orientation.x;
        quat.y = loaded_topic_path.poses[i].pose.orientation.y;
        quat.z = loaded_topic_path.poses[i].pose.orientation.z;
        quat.w = loaded_topic_path.poses[i].pose.orientation.w;
        wp_nav_tool_->missionIndex = missionSelected;

        wp_nav_tool_->makeIm(position, quat, missionSelected);
        i+= ui_->make_wp_spinbox->value();
    }
//        geometry_msgs::PoseStamped pos = paths[1].poses[i];
//
}

void WaypointFrame::makeWPButtonClicked() {
    for (int i = 0; i < loaded_path.poses.size(); i++){
        Ogre::Vector3 position;
        position.x = loaded_path.poses[i].pose.position.x;
        position.y = loaded_path.poses[i].pose.position.y;
        position.z = loaded_path.poses[i].pose.position.z;

        Ogre::Quaternion quat;
        quat.x = loaded_path.poses[i].pose.orientation.x;
        quat.y = loaded_path.poses[i].pose.orientation.y;
        quat.z = loaded_path.poses[i].pose.orientation.z;
        quat.w = loaded_path.poses[i].pose.orientation.w;
        wp_nav_tool_->missionIndex = missionSelected;

        wp_nav_tool_->makeIm(position, quat, missionSelected);
        i+= ui_->make_wp_spinbox->value();
    }
//        geometry_msgs::PoseStamped pos = paths[1].poses[i];
//
}

void WaypointFrame::setSelectedWPSpeed(){
    int from_wp = ui_->wp_selected_from->value();
    int to_wp = ui_->wp_selected_to->value();
    for (int i = from_wp-1; i < to_wp; i++){
        speed[missionSelected][i] = ui_->speed_selected_wp->value();
    }
}

void WaypointFrame::setAllWPSpeed() {
    //std::cout << "SET SPEED ALL WP \n";
//    for (auto i : speed[missionSelected]){
    for (int i =0; i < speed[missionSelected].size(); i++){
        speed[missionSelected][i] += ui_->speed_all_wp->value();
    }
}

void WaypointFrame::deleteSelectedWP(){
    setMission(missionSelected);
    int from_wp = ui_->delete_wp_from->value() - 1;
    int to_wp = ui_->delete_wp_to->value();
    speed[missionSelected].erase(speed[missionSelected].begin()+from_wp , speed[missionSelected].begin() + to_wp);
    // for (int i = from_wp- 1; i < to_wp; i++){
    paths[missionSelected].poses.erase(paths[missionSelected].poses.begin()+ from_wp, paths[missionSelected].poses.begin() +to_wp);
    // }
    clearAllWaypoints();
    drawPath(missionSelected);
    //std::cout<< "DELETE WP " << from_wp<< " TO " << to_wp << " MIS " << missionSelected <<std::endl;
}

void WaypointFrame::topicPathChanged() {
    sub_topic_path.shutdown();
    sub_topic_path = nh_.subscribe(ui_->topic_path_line_edit->text().toStdString(), 1, &WaypointFrame::loadedPathTopicSubsCallback, this);

}

void WaypointFrame::loadedPathSubsCallback(const nav_msgs::Path& msg) {
//        paths[missionSelected] = msg;
    loaded_path = msg;
}

void WaypointFrame::loadedPathTopicSubsCallback(const nav_msgs::Path& msg) {
//        paths[missionSelected] = msg;
    loaded_topic_path = msg;
}

void WaypointFrame::clearAllWaypoints()
{
  //destroy the ogre scene nodes
//    speed[missionSelected].clear();
    std::map<int, Ogre::SceneNode* >::iterator sn_it;
    for (sn_it = sn_map_ptr_->begin(); sn_it != sn_map_ptr_->end(); sn_it++)
    {
        scene_manager_->destroySceneNode(sn_it->second);
    }

    //clear the waypoint map and reset index
    sn_map_ptr_->clear();
    *unique_ind_=0;

    //clear the interactive markers
    server_->clear();
    server_->applyChanges();
}

void WaypointFrame::heightChanged(double h)
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  default_height_ = h;
}

void WaypointFrame::setActionButtonClicked() {
    int from_wp = ui_->action_from_wp_spinbox->value();
    int to_wp = ui_->action_to_wp_spinbox->value();
    for (int i = from_wp-1; i < to_wp; i++){
        action_wp[missionSelected][i] = ui_->action_wp_spinbox->value();
    }
}

void WaypointFrame::deleteSavadSpeed(int index){
        //std::cout<< "HAPUS KE " << index << " " << speed[missionSelected][index] <<std::endl;
  speed[missionSelected].erase(speed[missionSelected].begin() + (index-1));
//  std::cout << speed[missionSelected] << std::endl;
}

void WaypointFrame::changeSavedSpeed(double value){
    //std::cout << "UBAH SPEED WP " << sel_wp << " " << value << std::endl;
  if (sel_wp != 0 && speed[missionSelected].size() >= sel_wp)
    speed[missionSelected][sel_wp-1] = value;
}

void WaypointFrame::setSpeed(int value){
    //std::cout << "SET SPEED DAPAT AVALUE : " << value <<std::endl;
    sel_wp = value;
    if (speed[missionSelected].size() < value){
      sel_wp = 0;
      speed[missionSelected].push_back(ui_->speed_spinBox->value());
      //std::cout << "INIT VALUE SPEED : " << double(speed[missionSelected][value-1]) << std::endl;

    }
    else{
//        speed[missionSelected][value-1] = ui_->speed_spinBox->value();
      ui_->speed_spinBox->setValue(double(speed[missionSelected][value-1]));
      //std::cout << "VALUE SPEED : " << double(speed[missionSelected][value-1]) << std::endl;
    }

    for (int i = 0; i < speed[1].size(); i++){
//      std::cout << speed[1][i] << std::endl;
    }
}

void WaypointFrame::setActionWP(int index) {
    if (action_wp[missionSelected].size() < index){
        action_wp[missionSelected].push_back(-1);
    }
}

void WaypointFrame::deleteActionWP(int index) {
    action_wp[missionSelected].erase(action_wp[missionSelected].begin() + (index-1));
}

void WaypointFrame::setServoWP(int index) {
    if (servo_action_wp[missionSelected].size() < index){
        servo_action_wp[missionSelected].push_back(0);
    }
}

void WaypointFrame::deleteServoWP(int index) {
    servo_action_wp[missionSelected].erase(servo_action_wp[missionSelected].begin() + (index-1));
}

void WaypointFrame::setSelectedMarkerName(std::string name)
{
  std::stringstream wp_label_name;
  wp_label_name << name <<std::endl<< "Mission "<<missionSelected;
  // std::string wp_name_str(wp_name.str());
  //std::cout << "SETSELECTEDMARKER NAME " << name << std::endl;
  //std::cout << "SETSELECTEDMARKER NAME " << name.substr(8,9) << "AS" << std::endl;
  setSpeed(stoi(name.substr(8,9)));

  selected_marker_name_ = wp_label_name.str();
//  selected_marker_name_ = "LOLOLO";
}

void WaypointFrame::poseChanged(double val)
{
  //std::cout << "POSE CHANGED\n";
  std::map<int, Ogre::SceneNode *>::iterator sn_entry =
      sn_map_ptr_->find(std::stoi(selected_marker_name_.substr(8)));

  if (sn_entry == sn_map_ptr_->end())
    ROS_ERROR("%s not found in map", selected_marker_name_.c_str());
  else
  {
    Ogre::Vector3 position;
    Ogre::Quaternion quat;
    getPose(position, quat);

    sn_entry->second->setPosition(position);
    sn_entry->second->setOrientation(quat);

    std::stringstream wp_name;
    wp_name << "waypoint" << sn_entry->first;
    std::string wp_name_str(wp_name.str());

    visualization_msgs::InteractiveMarker int_marker;
    if(server_->get(wp_name_str, int_marker))
    {
      int_marker.pose.position.x = position.x;
      int_marker.pose.position.y = position.y;
      int_marker.pose.position.z = position.z;

      int_marker.pose.orientation.x = quat.x;
      int_marker.pose.orientation.y = quat.y;
      int_marker.pose.orientation.z = quat.z;
      int_marker.pose.orientation.w = quat.w;

      server_->setPose(wp_name_str, int_marker.pose, int_marker.header);
    }
    server_->applyChanges();
  }
}

void WaypointFrame::frameChanged()
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  QString new_frame = ui_->frame_line_edit->text();

  // Only take action if the frame has changed.
  if((new_frame != frame_id_)  && (new_frame != ""))
  {
    frame_id_ = new_frame;
    ROS_INFO("new frame: %s", frame_id_.toStdString().c_str());

    // update the frames for all interactive markers
    std::map<int, Ogre::SceneNode *>::iterator sn_it;
    for (sn_it = sn_map_ptr_->begin(); sn_it != sn_map_ptr_->end(); sn_it++)
    {
      std::stringstream wp_name;
      wp_name << "waypoint" << sn_it->first;
      std::string wp_name_str(wp_name.str());

      visualization_msgs::InteractiveMarker int_marker;
      if(server_->get(wp_name_str, int_marker))
      {
        int_marker.header.frame_id = new_frame.toStdString();
        server_->setPose(wp_name_str, int_marker.pose, int_marker.header);
      }
    }
    server_->applyChanges();
  }
}

void WaypointFrame::topicChanged()
{
  QString new_topic = ui_->topic_line_edit->text();

  // Only take action if the name has changed.
  if(new_topic != output_topic_)
  {
    wp_pub_.shutdown();
    output_topic_ = new_topic;

    if((output_topic_ != "") && (output_topic_ != "/"))
    {
      wp_pub_ = nh_.advertise<rviz_plugin::Selectedwp>(output_topic_.toStdString(), 1);
    }
  }
}

void WaypointFrame::setWpCount(int size)
{
  std::ostringstream stringStream;
  stringStream << "num wp: " << size;
  std::string st = stringStream.str();
  //std::cout << "SET WP COUNT " << size << "\n";
  // setSpeed();
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  ui_->waypoint_count_label->setText(QString::fromStdString(st));
}

void WaypointFrame::setConfig(QString topic, QString frame, float height)
{
  {
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  ui_->topic_line_edit->blockSignals(true);
  ui_->frame_line_edit->blockSignals(true);
  ui_->wp_height_doubleSpinBox->blockSignals(true);

  ui_->topic_line_edit->setText(topic);
  ui_->frame_line_edit->setText(frame);
  ui_->wp_height_doubleSpinBox->setValue(height);

  ui_->topic_line_edit->blockSignals(false);
  ui_->frame_line_edit->blockSignals(false);
  ui_->wp_height_doubleSpinBox->blockSignals(false);

  }
  topicChanged();
  frameChanged();
  heightChanged(height);
}

void WaypointFrame::getPose(Ogre::Vector3& position, Ogre::Quaternion& quat)
{
  {
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  position.x = ui_->x_doubleSpinBox->value();
  position.y = ui_->y_doubleSpinBox->value();
  position.z = ui_->z_doubleSpinBox->value();
  double yaw = ui_->yaw_doubleSpinBox->value();

  tf::Quaternion qt = tf::createQuaternionFromYaw(yaw);
  quat.x = qt.x();
  quat.y = qt.y();
  quat.z = qt.z();
  quat.w = qt.w();

  }
}

void WaypointFrame::setPose(Ogre::Vector3& position, Ogre::Quaternion& quat)
{
  //std::cout << "SET POSE CALLED\n";
  //boost::mutex::scoped_lock lock(frame_updates_mutex_);
  //block spinbox signals
  ui_->x_doubleSpinBox->blockSignals(true);
  ui_->y_doubleSpinBox->blockSignals(true);
  ui_->z_doubleSpinBox->blockSignals(true);
  ui_->yaw_doubleSpinBox->blockSignals(true);

  ui_->x_doubleSpinBox->setValue(position.x);
  ui_->y_doubleSpinBox->setValue(position.y);
  ui_->z_doubleSpinBox->setValue(position.z);

  tf::Quaternion qt(quat.x, quat.y, quat.z, quat.w);
  ui_->yaw_doubleSpinBox->setValue(tf::getYaw(qt));

  //enable the signals
  ui_->x_doubleSpinBox->blockSignals(false);
  ui_->y_doubleSpinBox->blockSignals(false);
  ui_->z_doubleSpinBox->blockSignals(false);
  ui_->yaw_doubleSpinBox->blockSignals(false);
  
}

void WaypointFrame::setWpLabel(Ogre::Vector3 position)
{
  {
  //boost::mutex::scoped_lock lock(frame_updates_mutex_);
  std::ostringstream stringStream;
  stringStream.precision(2);
  stringStream << selected_marker_name_;
  //stringStream << " x: " << position.x << " y: " << position.y << " z: " << position.z;
  std::string label = stringStream.str();
  // speed[missionSelected][]
  ui_->sel_wp_label->setText(QString::fromStdString(label));
  }
}

double WaypointFrame::getDefaultHeight()
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  return default_height_;
}

QString WaypointFrame::getFrameId()
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
//  std::cout << "ID FRAME: " << frame_id_.toStdString() << std::endl;
  return frame_id_;
}

QString WaypointFrame::getOutputTopic()
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  return output_topic_;
}

void WaypointFrame::setMission(int index){
    //std::cout<<"Masuk ganti misi"<<std::endl;
    wp_nav_tool_->missionIndex = index;

    setPath();
    //std::cout<<"Masuk setpath"<<std::endl;

    missionSelected = index;
    //save last Topic Name
//  pathMissionIndexName[missionSelected] = ui_->topic_line_edit->text().toStdString();


  //open current Topic Name
//  ui_->topic_line_edit->setText(QString::fromStdString(pathMissionIndexName[missionSelected]));
//  topicChanged();

  clearAllWaypoints();
  
  if(missionSelected == 0){
    for(int i =1; i<4;i++){
      drawPath(i);
    }
  }else{
    drawPath(missionSelected);
  }
    //std::cout<<"Masuk drawpath"<<std::endl;

    if(index == 0){
        wp_nav_tool_->activate();
    }else{
        wp_nav_tool_->deactivate();
    }

    if (has_loaded){
        saveLastSetting();
        //std::cout << "SAVE LAST SETTING " << has_loaded << "\n";
    }
}

void WaypointFrame::drawPath(int missionNow){  

  for(int i = 0; i < paths[missionNow].poses.size();i++){
    
    geometry_msgs::PoseStamped pos = paths[missionNow].poses[i];
    Ogre::Vector3 position;
    position.x = pos.pose.position.x;
    position.y = pos.pose.position.y;
    position.z = pos.pose.position.z;

    Ogre::Quaternion quat;
    quat.x = pos.pose.orientation.x;
    quat.y = pos.pose.orientation.y;
    quat.z = pos.pose.orientation.z;
    quat.w = pos.pose.orientation.w;

    wp_nav_tool_->missionIndex = missionNow;

    wp_nav_tool_->makeIm(position, quat, missionNow);
//    setSpeed(i+1);

  }
  // sn_map_ptr_ = *sn_maps_saved[missionSelected]; 
}

void WaypointFrame::setPath(){
  nav_msgs::Path *path;
  paths[missionSelected].poses.clear();
  path = &paths[missionSelected];
  // path->clear();
  int i=0;
  //std::cout << "MASUK SET PATH\n";
  std::map<int, Ogre::SceneNode* >::iterator sn_it;
  // &sn_maps_saved[missionSelected] = sn_map_ptr_;
  int wp_size_tmp;
  if(missionSelected == 0){
      int cur_mis = 1, counter = 0;
      wp_size_tmp = paths[cur_mis].poses.size();
      paths[cur_mis].poses.clear();
      path = &paths[cur_mis];
      for (sn_it = sn_map_ptr_->begin(); sn_it != sn_map_ptr_->end(); sn_it++) {
          //std::cout << "COUNTER " << counter << " curmis " << cur_mis << " WP SIZE "<< wp_size_tmp << std::endl;
          counter++;


          i++;

          Ogre::Vector3 position;

          position = sn_it->second->getPosition();

          // if(path->poses.size() > i && position.x == path->poses[i].pose.position.x){
          //   // ROS_INFO("DILEWATI KARENA SAMA", filn.c_str());
          //   std::cout<<"DILEWATI KARENA SAMA"<<std::endl;
          //   continue;
          // }
          geometry_msgs::PoseStamped pos;
          pos.pose.position.x = position.x;
          pos.pose.position.y = position.y;
          pos.pose.position.z = position.z;

          Ogre::Quaternion quat;
          quat = sn_it->second->getOrientation();
          pos.pose.orientation.x = quat.x;
          pos.pose.orientation.y = quat.y;
          pos.pose.orientation.z = quat.z;
          pos.pose.orientation.w = quat.w;

          path->poses.push_back(pos);

          if (counter == wp_size_tmp) {
              counter = 0;
              cur_mis++;
              wp_size_tmp = paths[cur_mis].poses.size();
              paths[cur_mis].poses.clear();
              path = &paths[cur_mis];
          }
      }

  }else{
      for (sn_it = sn_map_ptr_->begin(); sn_it != sn_map_ptr_->end(); sn_it++)
      {
          i++;
          Ogre::Vector3 position;


          position = sn_it->second->getPosition();

          // if(path->poses.size() > i && position.x == path->poses[i].pose.position.x){
          //   // ROS_INFO("DILEWATI KARENA SAMA", filn.c_str());
          //   std::cout<<"DILEWATI KARENA SAMA"<<std::endl;
          //   continue;
          // }
          geometry_msgs::PoseStamped pos;
          pos.pose.position.x = position.x;
          pos.pose.position.y = position.y;
          pos.pose.position.z = position.z;

          Ogre::Quaternion quat;
          quat = sn_it->second->getOrientation();
          pos.pose.orientation.x = quat.x;
          pos.pose.orientation.y = quat.y;
          pos.pose.orientation.z = quat.z;
          pos.pose.orientation.w = quat.w;


          path->poses.push_back(pos);
      }
  }



}

    void WaypointFrame::keyPressEvent(QKeyEvent *event){
        if(event->key() == Qt::Key_A)
        {

            missionSelected = 1;
            setMission(missionSelected);
            ui_->missionComboBox->setCurrentIndex(1);
        }else if(event->key() == Qt::Key_D){
            missionSelected = 2;
            setMission(missionSelected);
            ui_->missionComboBox->setCurrentIndex(2);
        }else if(event->key() == Qt::Key_0){
            action_wp[missionSelected][sel_wp] = 0;
            //std::cout << "0 PRESSED\n";
        }else if(event->key() == Qt::Key_1){
            //std::cout << "1 PRESSED\n";
            action_wp[missionSelected][sel_wp] = 1;
        }else if(event->key() == Qt::Key_2){
            //std::cout << "2 PRESSED\n";
            action_wp[missionSelected][sel_wp] = 2;
        }else if(event->key() == Qt::Key_BracketLeft){
            //std::cout << "{ PRESSED\n";
            action_wp[missionSelected][sel_wp] = 2;
        }
    }

} // namespace
