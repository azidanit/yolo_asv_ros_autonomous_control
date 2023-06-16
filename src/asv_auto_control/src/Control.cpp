//
// Created by azidanit on 5/30/22.
//

#include "Control.h"

Control::Control()
{
    // kapal belok kiri Z = POSITIF
    // kapal belok kanan Z = NEGATIF

    // init tf var
    tfListener = new tf2_ros::TransformListener(tfBuffer);
    is_path_recorded = false;

    // misisons[0] = new FindKorban(this);
    find_korban = new FindKorban(this);

    initVar();
    initSub();
    initPub();
}

Control::~Control()
{
}

void Control::run()
{
    ros::Rate rr(50);
    while (ros::ok())
    {

        listenASVTF();

        if (mission_state.data[0] == 1)
        {
            // start mission
            out_cmd = find_korban->calculateOut();
            if (out_cmd.linear.z != 1)
            { // bukan control camera
                mission_status_msg.data = "Waypoint Control";
                mission_status_string_pub.publish(mission_status_msg);
                // std::cout << "bukan control camera\n";
                geometry_msgs::Twist obs_cmd;
                obs_cmd = obstacle_avoid_control->calculateOut();
                // std::cout << "bukan control camera " << obs_cmd << "\n";
                if ((obs_cmd.linear.x != 0 && obs_cmd.angular.z != 0) || obs_cmd.linear.x < 0)
                {
                    out_cmd = obs_cmd;
                    std::cout << "MENHINDARRR\n";
                    mission_status_msg.data = "SRF Control";
                    mission_status_string_pub.publish(mission_status_msg);
                }
            }
            else
            {
                mission_status_msg.data = "Camera Control";
                mission_status_string_pub.publish(mission_status_msg);
            }

            // std::cout << out_cmd << std::endl;
        }
        else if (mission_state.data[0] == 0)
        {
            // stop mission
            find_korban->stop();

            if (is_test_motor)
            {
                out_cmd.linear.x = thrust_trim;
                out_cmd.angular.z = steer_trim;
                sendCmdVel();
            }
        }

        if (mission_state.data[0] != 0)
        {
            out_cmd.angular.z += steer_trim;
            // std::cout << "SINIII\n";
            sendCmdVel();
        }

        rr.sleep();
        ros::spinOnce();
    }
}

void Control::initVar()
{
    mission_state.data.push_back(0);
    mission_state.data.push_back(0);

    use_speed_control = false;
    target_constant_thrust = 0;
    steer_trim = 0;
    // find_korban->setPIDWpAngle(&pid_angle_wp_find_korban);
    // find_korban->setPIDWpDistance(&pid_distance_wp_find_korban);
    is_test_motor = false;

    alpha_ema = 0.1;

    obstacle_avoid_control = new ObstacleAvoidanceControl(this, find_korban, &pid_angle_obs_avoid, &pid_thrust_obs_avoid);
}

void Control::initSub()
{

    gps_raw_sub = nh.subscribe("/mavros/global_position/global", 2, &Control::GPSRawCallback, this);

    mission_state_control_sub = nh.subscribe("/rviz_plugin/stateMission", 2, &Control::stateMissionCallback, this);
}

void Control::initPub()
{
    asv_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/asv/cmd_vel", 1);

    path_pub = nh.advertise<nav_msgs::Path>("/path_asv", 1);
    global_path_pub = nh.advertise<nav_msgs::Path>("/path_asv_global", 1);
    loaded_path_pub = nh.advertise<nav_msgs::Path>("/path_asv_loaded", 1);
    track_path_pub = nh.advertise<nav_msgs::Path>("/path_asv_track", 1);
    track_start_text_pub = nh.advertise<visualization_msgs::Marker>("/path_asv_track_start_text", 1);

    mission_status_string_pub = nh.advertise<std_msgs::String>("/rviz_plugin/missin_status", 1);
}

void Control::GPSRawCallback(sensor_msgs::NavSatFix data_gps)
{
    current_gps = data_gps;
}

void Control::listenASVTF()
{
    double roll, pitch, yaw;

    try
    {
        transformStamped = tfBuffer.lookupTransform("map", "asv/base_link",
                                                    ros::Time(0));
        ASV_TF.pos.y = transformStamped.transform.translation.y;
        ASV_TF.pos.x = transformStamped.transform.translation.x;
        ASV_TF.pos.z = 0;

        tf2::Quaternion q(
            transformStamped.transform.rotation.x,
            transformStamped.transform.rotation.y,
            transformStamped.transform.rotation.z,
            transformStamped.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(ASV_TF.roll, ASV_TF.pitch, ASV_TF.yaw);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        //   ros::Duration(1.0).sleep();
    }

    // std::cout << "ASV TF COTNROL " << ASV_TF.pos.x << " " << ASV_TF.pos.y << " YAW " << ASV_TF.yaw << std::endl;
}

TF_simplified Control::getRobotTf()
{
    return ASV_TF;
}

void Control::sendCmdVel()
{
    // val_out = (alpha_ema * value_cur) + (1-alpha_ema) * value_before;
    out_cmd_ema.angular.z = (alpha_ema * out_cmd.angular.z) + (1 - alpha_ema) * out_cmd_ema_before.angular.z;
    out_cmd_ema.linear.x = (alpha_ema * out_cmd.linear.x) + (1 - alpha_ema) * out_cmd_ema_before.linear.x;

    asv_cmd_vel_pub.publish(out_cmd_ema);

    out_cmd_ema_before.angular.z = out_cmd_ema.angular.z;
    out_cmd_ema_before.linear.x = out_cmd_ema.linear.x;
}

void Control::startMission()
{
}

void Control::stopMission()
{
}

void Control::resumeMission()
{
}

void Control::changeSteerTrim(int val)
{
    param_qt_mtx.lock();
    steer_trim = (float)val / 1000;
    param_qt_mtx.unlock();
}

void Control::changeSpeedTrim(int val)
{
    param_qt_mtx.lock();
    thrust_trim = (float)val / 1000;
    param_qt_mtx.unlock();
}

void Control::useSpeedControl(bool val)
{
    param_qt_mtx.lock();
    use_speed_control = val;
    param_qt_mtx.unlock();
}

void Control::useAccError(bool)
{
}

void Control::changeUseObstacleAvoidance(bool val)
{
    param_qt_mtx.lock();
    // use_obstacle_avoidance = val;
    param_qt_mtx.unlock();
}

void Control::changeUseObstacleStop(bool val)
{
    param_qt_mtx.lock();
    // use_obstacle_stop = val;
    param_qt_mtx.unlock();
}

void Control::changeP(int idx, double val)
{
    std::cout << "CHANGED P IN CONTROL " << idx << " " << val << std::endl;
    param_qt_mtx.lock();
    if (idx == 0)
    {
        pid_angle_wp_find_korban.setP(val);
    }
    else if (idx == 1)
    {
        pid_distance_wp_find_korban.setP(val);
    }
    else if (idx == 2)
    {
        pid_speed_control.setP(val);
    }
    else if (idx == 3)
    {
        pid_angle_obs_avoid.setP(val);
    }
    else if (idx == 4)
    {
        pid_x_cam_find_korban.setP(val);
    }
    else if (idx == 5)
    {
        pid_y_cam_find_korban.setP(val);
    }
    param_qt_mtx.unlock();
}

void Control::changeI(int idx, double val)
{
    param_qt_mtx.lock();
    if (idx == 0)
    {
        pid_angle_wp_find_korban.setI(val);
    }
    else if (idx == 1)
    {
        pid_distance_wp_find_korban.setI(val);
    }
    else if (idx == 2)
    {
        pid_speed_control.setI(val);
    }
    else if (idx == 3)
    {
        pid_angle_obs_avoid.setI(val);
    }
    else if (idx == 4)
    {
        pid_x_cam_find_korban.setI(val);
    }
    else if (idx == 5)
    {
        pid_y_cam_find_korban.setI(val);
    }
    param_qt_mtx.unlock();
}

void Control::changeD(int idx, double val)
{
    param_qt_mtx.lock();
    if (idx == 0)
    {
        pid_angle_wp_find_korban.setD(val);
    }
    else if (idx == 1)
    {
        pid_distance_wp_find_korban.setD(val);
    }
    else if (idx == 2)
    {
        pid_speed_control.setD(val);
    }
    else if (idx == 3)
    {
        pid_angle_obs_avoid.setD(val);
    }
    else if (idx == 4)
    {
        pid_x_cam_find_korban.setD(val);
    }
    else if (idx == 5)
    {
        pid_y_cam_find_korban.setD(val);
    }
    param_qt_mtx.unlock();
}

void Control::changeSpeed(int idx, int val)
{
    // target_constant_thrust = val;
}

void Control::changeSpeedDouble(int idx, double val)
{
    param_qt_mtx.lock();
    if (idx == 0)
        target_constant_thrust = val;
    else if (idx == -1)
        alpha_ema = val;

    param_qt_mtx.unlock();
}

void Control::changeDistance(int idx, double val)
{
    param_qt_mtx.lock();
    // save_front_distance = val;
    param_qt_mtx.unlock();
}

void Control::changeASVWidth(double)
{
}

void Control::changeASVOffset(double)
{
}

void Control::changeRTHAfterDone(bool val)
{
    std::cout << "RTH " << val << "\n";
    // is_rth_after_done = val;
    // rth_called = false;
}

void Control::changePatrolLap(int val)
{
    param_qt_mtx.lock();
    // patrol_lap = val;
    param_qt_mtx.unlock();
    std::cout << "PATROL LAP TO " << val << "\n";
}

void Control::calibrateCompass(double val)
{
}

void Control::changeUseCompass(bool val)
{
    // std_msgs::UInt8 msg;
    // msg.data = (int)val;
    // using_compass_pub.publish(msg);
}

void Control::changeUseFindKorban(bool val)
{
    param_qt_mtx.lock();
    // use_find_korban = val;
    param_qt_mtx.unlock();
    // std::cout << "FIND KORBAN " << use_find_korban << "\n";
}

void Control::pitchChanged(int idx, double val)
{
    // sensor_control.setPitchThreshold(val);
}

void Control::startRecordPath()
{
    std::cout << "Start record path" << std::endl;
    if (is_path_recorded)
    {
        std::cout << "is recording... " << std::endl;
        return;
    }

    path_mtx.lock();
    is_path_recorded = true;
    path_mtx.unlock();
    path_thread = std::thread(&Control::recordPath, this);
}

void Control::stopRecordPath()
{
    // std::cout << "Stop record path" << std::endl;
    if (!is_path_recorded)
    {
        return;
    }

    path_mtx.lock();
    is_path_recorded = false;
    path_mtx.unlock();
    path_thread.join();
    std::cout << "Stopped record path" << std::endl;
}

void Control::recordPath()
{
    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped global_pose;
    pose.header.frame_id = "map";
    path_msg.header.frame_id = "map";
    is_path_recorded = true;
    printf("masuk path record\n");

    while (is_path_recorded)
    {
        printf("masuk path record\n");
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = transformStamped.transform.translation.x;
        pose.pose.position.y = transformStamped.transform.translation.y;
        pose.pose.position.z = transformStamped.transform.translation.z + 0.15;
        pose.pose.orientation.x = transformStamped.transform.rotation.x;
        pose.pose.orientation.y = transformStamped.transform.rotation.y;
        pose.pose.orientation.z = transformStamped.transform.rotation.z;
        pose.pose.orientation.w = transformStamped.transform.rotation.w;

        path_msg.header.stamp = ros::Time::now();
        path_msg.poses.push_back(pose);
        path_pub.publish(path_msg);

        // Record GPS
        printf("recording path");
        if (current_gps.latitude != 0.0000 && current_gps.longitude != 0.0000)
        {
            global_path.push_back(current_gps);

            global_pose = globalToPose(current_gps.latitude, current_gps.longitude);
            global_pose.header.stamp = ros::Time::now();
            // global_pose.pose.position.z = 0;
            global_pose.pose.orientation.x = currentBoatTF.getRotation().getX();
            global_pose.pose.orientation.y = currentBoatTF.getRotation().getY();
            global_pose.pose.orientation.z = currentBoatTF.getRotation().getZ();
            global_pose.pose.orientation.w = currentBoatTF.getRotation().getW();

            global_path_msg.header.stamp = ros::Time::now();
            global_path_msg.poses.push_back(global_pose);
            global_path_pub.publish(path_msg);
        }
        usleep(200000);
    }
    std::cout << "Stopped record path" << std::endl;
}

void Control::clearPath()
{
    std::cout << "clearing path " << path_msg.poses.size() << std::endl;
    path_mtx.lock();
    path_msg.poses.clear();
    global_path_msg.poses.clear();
    loaded_path_msg.poses.clear();
    path_mtx.unlock();

    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();
    path_pub.publish(path_msg);

    loaded_path_msg.header.frame_id = "map";
    loaded_path_msg.header.stamp = ros::Time::now();
    loaded_path_pub.publish(loaded_path_msg);

    global_path_msg.header.frame_id = "map";
    global_path_msg.header.stamp = ros::Time::now();
    global_path_pub.publish(global_path_msg);

    global_path.clear();

    std::cout << "cleared path " << global_path.size() << std::endl;
}

void Control::savePath(QString filename)
{
    std::cout << "save path to " << filename.toStdString() << std::endl;
    if (global_path.size() == 0)
    {
        return;
    }
    std::cout << "save path" << std::endl;
    sensor_msgs::NavSatFix path_to_save;
    QFile file(filename);

    if (file.open(QIODevice::ReadWrite | QIODevice::Truncate))
    {
        QTextStream stream(&file);
        char str[1024];
        auto pit = path_msg.poses.begin();
        for (auto it = ++(global_path.begin()); it != global_path.end(); ++it)
        {
            memset(str, 0, sizeof(str));
            sprintf(str, "%.8f,%.8f,%.8f,%.8f,%.8f,%.8f",
                    it->latitude, it->longitude,
                    pit->pose.orientation.w, pit->pose.orientation.x, pit->pose.orientation.y, pit->pose.orientation.z);
            stream << str << endl;
            std::cout << str << std::endl;
            ++pit;
        }
        file.close();
    }

    // save local path
    QString filename_local = filename + QString("local");
    QFile file_loc(filename_local);

    if (file_loc.open(QIODevice::ReadWrite | QIODevice::Truncate))
    {
        QTextStream stream(&file_loc);
        char str[1024];
        for (auto pit = ++(path_msg.poses.begin()); pit != path_msg.poses.end(); ++pit)
        {
            memset(str, 0, sizeof(str));
            sprintf(str, "%.8f,%.8f",
                    pit->pose.position.x, pit->pose.position.y);
            stream << str << endl;
            std::cout << str << std::endl;
        }
        file_loc.close();
    }
}

void Control::loadPath(QString filename)
{
    std::cout << "load path" << std::endl;
    geometry_msgs::PoseStamped pose;
    double lat, lon;

    loaded_path_msg.poses.clear();
    loaded_path_msg.header.frame_id = "map";

    QFile file(filename);

    if (file.open(QIODevice::ReadWrite))
    {

        QTextStream in(&file);

        while (!in.atEnd())
        {
            QString line = in.readLine();
            std::cout << line.toStdString() << std::endl;
            QStringList fields = line.split(",");
            lat = fields[0].toDouble();
            lon = fields[1].toDouble();
            pose = globalToPose(lat, lon);
            pose.pose.orientation.w = fields[2].toDouble();
            pose.pose.orientation.x = fields[3].toDouble();
            pose.pose.orientation.y = fields[4].toDouble();
            pose.pose.orientation.z = fields[5].toDouble();
            pose.header.stamp = ros::Time::now();
            // std::cout << LongitudeCenter << "***" << LatitudeCenter <<  std::endl;
            loaded_path_msg.poses.push_back(pose);
        }
        file.close();

        loaded_path_msg.header.stamp = ros::Time::now();
        loaded_path_pub.publish(loaded_path_msg);
    }
}

void Control::loadLocalPath(QString filename)
{
    std::cout << "load path" << std::endl;
    geometry_msgs::PoseStamped pose;
    double lat, lon;

    loaded_path_msg.poses.clear();
    loaded_path_msg.header.frame_id = "map";

    QFile file(filename);

    if (file.open(QIODevice::ReadWrite))
    {

        QTextStream in(&file);

        while (!in.atEnd())
        {
            QString line = in.readLine();
            // std::cout << line.toStdString() << std::endl;
            QStringList fields = line.split(",");
            pose.pose.position.x = fields[0].toDouble();
            pose.pose.position.y = fields[1].toDouble();
            pose.pose.position.z = 0.5;
            pose.pose.orientation.w = 0;
            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = 0;
            pose.header.stamp = ros::Time::now();
            loaded_path_msg.poses.push_back(pose);
        }
        file.close();

        loaded_path_msg.header.stamp = ros::Time::now();
        loaded_path_pub.publish(loaded_path_msg);
    }
}

inline geometry_msgs::PoseStamped Control::globalToPose(double lat, double lon)
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";

    pose.pose.position.x = LON_TO_METER * (LongitudeCenter - lon);
    pose.pose.position.y = LAT_TO_METER * (LatitudeCenter - lat);
    pose.pose.position.z = .5;

    return pose;
}

void Control::changeCritLine(int i, int cl)
{
    param_qt_mtx.lock();
    // if (i) crit_line = cl;
    // else horizon = cl;
    param_qt_mtx.unlock();

    // sensor_control.setCurrentCritLineCamera(cl);
    // std::cout << "CRIT LINE " << crit_line << " " << horizon << "\n";
}

void Control::changeBoatSide(QPoint **)
{
}

void Control::stateMissionCallback(std_msgs::UInt16MultiArray msgl)
{
    mission_state_mtx.lock();
    mission_state = msgl;
    mission_state_mtx.unlock();
}

PIDController *Control::get_pid_angle_wp_find_korban()
{
    return &pid_angle_wp_find_korban;
}

PIDController *Control::get_pid_distance_wp_find_korban()
{
    return &pid_distance_wp_find_korban;
}

PIDController *Control::get_pid_x_cam_find_korban()
{
    return &pid_x_cam_find_korban;
}

PIDController *Control::get_pid_y_cam_find_korban()
{
    return &pid_y_cam_find_korban;
}

PIDController *Control::get_pid_angle_obs_avoid()
{
    return &pid_angle_obs_avoid;
}

PIDController *Control::get_pid_thrust_obs_avoid()
{
    return &pid_thrust_obs_avoid;
}

double Control::speedControlCalculate(double target)
{
    if (use_speed_control)
        return out_cmd.linear.x + pid_speed_control.updateError(target - current_asv_speed);
    else
        return target_constant_thrust;
}

double Control::getCurrentASVSpeed()
{
    return current_asv_speed;
}

void Control::testMotor(bool status)
{
    param_qt_mtx.lock();
    is_test_motor = status;
    param_qt_mtx.unlock();
}

std::vector<std::pair<int, int>> Control::sortArr(int arr[], int n)
{

    // Vector to store element
    // with respective present index
    std::vector<std::pair<int, int>> vp;

    // Inserting element in pair vector
    // to keep track of previous indexes
    for (int i = 0; i < n; ++i)
    {
        vp.push_back(std::make_pair(arr[i], i));
    }

    // Sorting pair vector
    sort(vp.begin(), vp.end());

    // Displaying sorted element
    // with previous indexes
    // corresponding to each element
    // cout << "Element\t"
    //     << "index" << endl;
    // for (int i = 0; i < vp.size(); i++) {
    //     cout << vp[i].first << "\t"
    //         << vp[i].second << endl;
    // }

    return vp;
}

void Control::changeStartLatLong(double lat, double lon)
{
    param_qt_mtx.lock();
    start_lat_long[0] = lat;
    start_lat_long[1] = lon;
    param_qt_mtx.unlock();
}

void Control::changeEndLatLong(double lat, double lon)
{
    param_qt_mtx.lock();
    end_lat_long[0] = lat;
    end_lat_long[1] = lon;
    param_qt_mtx.unlock();
}

void Control::changeTrackSpecs(double count, double lenght, double width)
{
    param_qt_mtx.lock();
    track_specs[0] = count;
    track_specs[1] = lenght;
    track_specs[2] = width;
    param_qt_mtx.unlock();
}

void pushBackPoseToPath(nav_msgs::Path *path, double x, double y)
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.1;
    pose.pose.orientation.w = 0;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.header.stamp = ros::Time::now();

    path->poses.push_back(pose);
}

void rotatePath(nav_msgs::Path &path, double angle, double anchorX, double anchorY)
{
    double radians = angle * M_PI / 180.0;
    double cosAngle = std::cos(radians);
    double sinAngle = std::sin(radians);

    for (auto &pose : path.poses)
    {
        double translatedX = pose.pose.position.x - anchorX;
        double translatedY = pose.pose.position.y - anchorY;

        double rotatedX = translatedX * cosAngle - translatedY * sinAngle;
        double rotatedY = translatedX * sinAngle + translatedY * cosAngle;

        pose.pose.position.x = rotatedX + anchorX;
        pose.pose.position.y = rotatedY + anchorY;
    }
}

void offsetPath(nav_msgs::Path &path, double offsetX, double offsetY)
{
    for (auto &pose : path.poses)
    {
        pose.pose.position.x += offsetX;
        pose.pose.position.y += offsetY;
    }
}

void Control::getLocalFromGlobalPose(double lat_global, double lon_global, double *x_local, double *y_local)
{
    std::string map_venue;
    nh.getParam("map_venue", map_venue);

    if (map_venue == ""){
        map_venue = "map_danau8";
    }

    std::cout << "SUCCESS GET PARAM " << map_venue << "\n";

    double dump;
    double map_origin_latlong_[4];
    std::string filename = ros::package::getPath("map_image") + "/resource/" + map_venue + ".txt";

    FILE *file = fopen(filename.c_str(), "r");
    fscanf(file, "%lf, %lf %lf, %lf", &map_origin_latlong_[0], &map_origin_latlong_[1], &map_origin_latlong_[2], &map_origin_latlong_[3]);
    fclose(file);

    // std::cout << "LEFT " << map_origin_latlong_[0] << " " << map_origin_latlong_[1] << "\n";
    // printf("%lf %lf\n", map_origin_latlong_[0], map_origin_latlong_[1]);
    // std::cout << "RIGHT " << map_origin_latlong_[2] << " " << map_origin_latlong_[3] << "\n";
    // std::cout << "CENTER " << lat_center << " " << long_center << "\n";
    // printf("%lf %lf\n", lat_center, long_center);

    double x_global = lon_global;
    double y_global = lat_global;
    double x_origin = (map_origin_latlong_[1] + map_origin_latlong_[3]) / 2.0;
    double y_origin = (map_origin_latlong_[0] + map_origin_latlong_[2]) / 2.0;

    double x_scale = 113321;
    double y_scale = 111000;
    *y_local = -(x_global - x_origin) * x_scale;
    *x_local = (y_global - y_origin) * y_scale;
}

void Control::offsetPathFromGlobalPose(nav_msgs::Path &path, double lat, double lon)
{
    double x, y;
    getLocalFromGlobalPose(lat, lon, &x, &y);
    offsetPath(path, x, y);
}

visualization_msgs::Marker makeTextMarker(std::string textMsg, double x, double y){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "text";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 1.0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;

    marker.scale.z = 0.5;

    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1;
    marker.color.g = 1;
    marker.color.b = 1;

    marker.text = textMsg;

    return marker;
}

void Control::generateTrackPath(double angle)
{
    // make path from start to end
    // using nav_msgs::Path
    // as parallel search pattern
    // and publish it to ros
    // short direction = 1;
    printf("GENERATE TRACK PATH a %f specs: %f %f %f\n", angle, track_specs[0], track_specs[1], track_specs[2]);
    nav_msgs::Path track_path_msg;
    track_path_msg.header.frame_id = "map";
    track_path_msg.header.stamp = ros::Time::now();

    for (int i = 0; i < track_specs[0]; i++)
    {
        if (i % 2 == 0)
        {
            for (float j = 0; j <= track_specs[1]; j += 0.5)
            {
                pushBackPoseToPath(&track_path_msg, j, i * track_specs[2]);
            }
        }
        else
        {
            for (float j = track_specs[1]; j >= 0; j -= 0.5)
            {
                pushBackPoseToPath(&track_path_msg, j, i * track_specs[2]);
            }
        }

        if (i == track_specs[0] - 1)
            break;

        for (float j = 0; j <= track_specs[2]; j += 0.5)
        {
            pushBackPoseToPath(&track_path_msg, (i % 2 == 0) ? track_specs[1] : 0, i * track_specs[2] + j);
        }
    }

    printf("GENERATE TRACK PATH a %f specs: %f %f %f DONEEEE\n", angle, track_specs[0], track_specs[1], track_specs[2]);


    double local_x, local_y;
    getLocalFromGlobalPose(start_lat_long[0], start_lat_long[1], &local_x, &local_y);
    track_start_text_pub.publish(makeTextMarker("START", local_x, local_y));

    rotatePath(track_path_msg, angle, 0, 0);
    offsetPathFromGlobalPose(track_path_msg, start_lat_long[0], start_lat_long[1]);
    
    getLocalFromGlobalPose(end_lat_long[0], end_lat_long[1], &local_x, &local_y);
    pushBackPoseToPath(&track_path_msg, local_x, local_y);

    track_path_pub.publish(track_path_msg);
}