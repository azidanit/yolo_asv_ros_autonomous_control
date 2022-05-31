#include "MissionWrapper.h"

MissionWrapper::MissionWrapper(ros::NodeHandle nh, char *venue){
    loadParams(nh);
    this->nh = nh;
    initVenue(venue);
    initVariable();

    ms = new Misi();

    initSubscriber();
    initPublisher();

}

MissionWrapper::~MissionWrapper(){
    std::cout << "Mission Wrapper destroyed" << std::endl;
}

void MissionWrapper::run(){

    marker_thread = std::thread(&MissionWrapper::publishMarkers, this);

    ros::Rate r_rate(50);
    while(ros::ok()){
        ros::spinOnce();
        listenBoatTF();
        r_rate.sleep();
    }
}

void MissionWrapper::initVariable() {
    is_started = false;
    start_waypoint = 0;
    selected_mission = -1;
    wp_idx_current = 0;

    quick_rth_called = hold_korban_called = false;
    is_korban_found = false;
    use_find_korban = use_obstacle_avoidance = use_obstacle_stop = false;
    lifebuoy_detached = false;

    target_constant_thrust = 0;
    steer_trim = thrust_trim = 0;
    steer_out = thrust_out = current_thrust = 0;
    asv_local_speed = 0;
    status_auto_from_remote = 2;

    patrol_lap = is_rth_after_done = is_quick_rth = 0;

    error_angle_wp = error_angle_wp_before = error_dist_wp = error_dist_wp_before = 0;
    error_speed = error_speed_before = 0;
    error_camera = error_camera_before = 0;

    pid_wp_angle.p = pid_wp_angle.i = pid_wp_angle.d = 0;
    pid_wp_dist.p = pid_wp_dist.i = pid_wp_dist.d = 0;
    pid_speed_control.p = pid_speed_control.i = pid_speed_control.d = 0;
    pid_obstacle_avoidance_srf.p = pid_obstacle_avoidance_srf.i = pid_obstacle_avoidance_srf.d = 0;
    pid_camera_tracking.p = pid_camera_tracking.i = pid_camera_tracking.d = 0;

}

void MissionWrapper::initSubscriber() {
    wp_sub = nh.subscribe("/waypoints", 10, &MissionWrapper::waypointsCallback, this);
    state_rviz_sub = nh.subscribe("/rviz_plugin/stateMission", 10, &MissionWrapper::missionStateCallback, this);
    marker_srf_sub = nh.subscribe("/asv_sensors/srf/markers", 10, &MissionWrapper::markerCallback, this);
    local_vel_sub = nh.subscribe("/mavros/local_position/velocity_body", 4, &MissionWrapper::localVelocityCallback, this);
    data_from_stm_sub = nh.subscribe("/data_from_stm", 5, &MissionWrapper::dataFromStmCallback, this);
    gps_sub = nh.subscribe("/mavros/global_position/global", 100, &MissionWrapper::gpsCallback, this);

    //camera param
    critline_sub = nh.subscribe("/camera/critline",1,&MissionWrapper::critlineCallback, this);
    boatside_sub = nh.subscribe("/camera/boatside",1,&MissionWrapper::boatsideCallback, this);
}

void MissionWrapper::initPublisher() {
    stm_pub = nh.advertise<geometry_msgs::Twist>("/data_to_stm", 1);

    cmpcal_pub = nh.advertise<std_msgs::Float32>("/autonomous_boat/cmpcal", 1);
    using_compass_pub = nh.advertise<std_msgs::UInt8>("/autonomous_boat/using_compass", 1);


    nextDest_wp_marker_pub = nh.advertise<rviz_plugin::NextDest>("/next_dest", 1);
    headingMarker_pub = nh.advertise<visualization_msgs::Marker>("/heading_marker", 1);
    targetMarker_pub = nh.advertise<visualization_msgs::Marker>("/target_marker", 1);
    compass_helper_pub = nh.advertise<visualization_msgs::Marker>("/autonomous_car/compass_helper",1);
    path_pub = nh.advertise<nav_msgs::Path>("/path_car", 1);
    global_path_pub = nh.advertise<nav_msgs::Path>("/global_path_car", 1);
    loaded_path_pub = nh.advertise<nav_msgs::Path>("/loaded_path_car", 1);

    orientation_TF_pub = nh.advertise<geometry_msgs::Twist>("/orientation_TF", 1);

}

void MissionWrapper::missionStateCallback(std_msgs::UInt16MultiArray msg) {
    int state = msg.data[0];
    if(state == 1) { //start mission from wp 0
        if(is_running)
            return;
        checkIsPatrol();
        thrust_out = steer_out = 0;
        std::cout << "starting\n";
        mission_wp_mtx.lock();
        selected_mission = 0;
        wp_start = 0;
        mission_wp_mtx.unlock();
        if(misi_thread.joinable()){
            misi_thread.join();
        }
        misi_thread = std::thread(&MissionWrapper::missionStart, this);
        std::cout << "started\n";
    }else if(state == 2){ //Resume mission from wp state val (msg.data[1])

    }else if(state == 3){ //run manual
        manual_control_sub = nh.subscribe("/manualctrl",1,&MissionWrapper::runManual, this);
    }else if(state == 10) { //called from another function recurse
        std::cout << "starting from recurse fn\n";
        selected_mission = msg.data[1];
        wp_start = 0;
        if(misi_thread.joinable()){
            misi_thread.join();
        }
        misi_thread = std::thread(&MissionWrapper::missionStart, this);
        std::cout << "started from recurse fn\n";
    }else if(state == 11){ //QUICK RTH
        quick_rth_called = true;
        initQuickRTH();
    }else{ //stop mission
        manual_control_sub.shutdown();
        std::cout << "stoping\n";
        run_mtx.lock();
        is_running = quick_rth_running = is_hold_korban_running = false;
        run_mtx.unlock();

//        if(misi_thread.joinable()){
//            misi_thread.join();
//        }
//        if(quick_rth_thread.joinable()){
//            quick_rth_thread.join();
//        }
        std::cout << "stoped\n";

    }

}

void MissionWrapper::missionStart() {
    if(waypoints[selected_mission].poses.size()<=0){
        wp_idx_current = -1;
        std::cout << "NO WAYPOINT AT ALL\n";
        return;
    }
    run_mtx.lock();
    is_running = true;
    run_mtx.unlock();
    int stopWP = waypoints[selected_mission].poses.size() - 1;
    mission_wp_mtx.lock();
    wp_idx_current = wp_start;
    mission_wp_mtx.unlock();

    error_angle_wp = error_angle_wp_before = error_dist_wp = error_dist_wp_before = 0;

    std::cout << "STARTING MISSION "<< selected_mission << " from wp "<< wp_start << " / " << stopWP << std::endl;

    setNextDest();

    ros::Rate r_rate(50);
    hold_korban_called = false;
    int is_overtaking = 0;
    int obstacle_position = 0;
    bool arrived;
    is_korban_found = false;
    while(wp_idx_current < stopWP && is_running && ros::ok()){
        arrived = false;

        while(!arrived && is_running && ros::ok()){
            avoiding_obstacle = false;

            if(is_korban_found){
                counter_braking_after_korban_found++;
                if(counter_braking_after_korban_found >= (50*3)){
//                    quick_rth_called = true;
//                    initQuickRTH();
//                    hold position tracking korban
                    hold_korban_called = true;
                    initHoldKorban();
//                    holdTrackingKorban();
                    is_korban_found = false;
                    break;
                }
            }

            if(use_obstacle_avoidance and sensor_control.isThereIsObstableSRF()){
                steer_out = steer_trim + controlObstacleAvoidanceSRF();
            }else if(use_find_korban and sensor_control.frontKorbanPositionErrorCamera() != -9999) {
                steer_out = steer_trim + controlFindKorbanCamera();
            }else{
                steer_out = steer_trim + controlWaypoint();
            }

            thrust_out = controlSpeed(-9999,-1, pid_speed_control);

            publishControlToSTM(steer_out, thrust_out);

            r_rate.sleep();
            arrived = isWaypointArrived();

            if(quick_rth_called or hold_korban_called)
                break;
        }

        setNextDest();
        usleep(1000);

        if(quick_rth_called or hold_korban_called)
            break;
    }
    publishControlToSTM(0, 0);

    if(quick_rth_called or hold_korban_called)
        return;

    std::cout << "MISSION " << selected_mission << " DONE\n";

    if(current_patrol_lap >= 0){
        std::cout << "STARTING PATROL " << current_patrol_lap << " \n";
        current_patrol_lap--;
        mission_wp_mtx.lock();
        if(patroling_back){
            selected_mission = MAIN_MISSION;
            wp_start = 0;
            patroling_back = false;
        }else{
            selected_mission = PATROL_BACK;
            wp_start = 0;
            patroling_back = true;
        }
        mission_wp_mtx.unlock();
        std::cout << "STARTING PATROL BAWAH " << current_patrol_lap << " \n";
        missionStart();
    }

    if(is_rth_after_done && !rth_called){
        mission_wp_mtx.lock();
        rth_called = true;
        selected_mission = GUIDED_RTH;
        wp_start = 0;
        mission_wp_mtx.unlock();
        missionStart();
    }else{
        run_mtx.lock();
        is_running = false;
        selected_mission = -1;
        run_mtx.unlock();
        publishControlToSTM(steer_trim,0);
    }
}

void MissionWrapper::initHoldKorban() {
    if(!hold_korban_called)
        return;
    run_mtx.lock();
    is_hold_korban_running = true;
    run_mtx.unlock();
//    selected_mission =;

    if(hold_korban_thread.joinable()){
        hold_korban_thread.join();
    }
    hold_korban_thread = std::thread(&MissionWrapper::holdTrackingKorban, this);
    std::cout << "started HOLD KORBAN\n";
}

void MissionWrapper::holdTrackingKorban() {
    ros::Rate rr(50);
    lifebuoy_detached = false;
    bool is_reversing = false;
    bool is_rotating = false;
    int counter_reversing = 0;
    bool detaching_lifebuoy = false;
    int counter_detaching_lifebuoy = 0;
    while(ros::ok() & is_hold_korban_running){
        std::cout << "HOLD TRACKING KORBAN\n";

        if(sensor_control.frontKorbanPositionErrorCamera() != -9999){ //ada korban
            steer_out = steer_trim + controlFindKorbanCamera();

            if(!lifebuoy_detached){
                steer_out = steer_trim + controlLifebuoyKorbanCamera();
                thrust_out = controlSpeed(0.5,-1, pid_camera_tracking_thrust);
                if(sensor_control.korbanDistanceEstimationCamera() <= 15){
                    detaching_lifebuoy = true;
                }
            }else{
                if(sensor_control.korbanDistanceEstimationCamera() >= 15){ //korban jauh, di gas
                    thrust_out = controlSpeed(0.5,-1, pid_camera_tracking_thrust);
                }else{
                    thrust_out = 0;
                }
            }

            is_reversing = is_rotating = false;
            counter_reversing = 0;
        }else{ //cari korban.. mundur terus putar 360
            if(counter_reversing >= 150){
//                is_reversing = false;
                is_rotating = true;
            }else{
                counter_reversing++;
            }

            if(is_rotating){
                steer_out = steer_trim+500;
                thrust_out = controlSpeed(1,-1, pid_speed_control);
            }else{
                steer_out = -steer_trim;
                thrust_out = controlSpeed(-0.75,-1, pid_speed_control);
            }
        }

        if(detaching_lifebuoy){
            if(sensor_control.frontKorbanPositionErrorCamera() != -9999)
                steer_out = steer_trim + controlLifebuoyKorbanCamera();
            else
                steer_out = steer_trim;
            thrust_out = controlSpeed(0.5,-1, pid_camera_tracking_thrust);
            counter_detaching_lifebuoy++;
            if(counter_detaching_lifebuoy >= 150){
                lifebuoy_detached = true;
                detaching_lifebuoy = false;
                counter_detaching_lifebuoy = 0;
            }
        }

        publishControlToSTM(steer_out, thrust_out);
        rr.sleep();
    }

    run_mtx.lock();
//    is_running = false;
    selected_mission = -1;
    run_mtx.unlock();

    publishControlToSTM(steer_trim, 0);

}

void MissionWrapper::initQuickRTH() {
    if(quick_rth_running)
        return;
    run_mtx.lock();
    quick_rth_called = true;
    run_mtx.unlock();
    selected_mission = MAIN_MISSION;

    if(quick_rth_thread.joinable()){
        quick_rth_thread.join();
    }
    quick_rth_thread = std::thread(&MissionWrapper::quickRTHMissionStart, this);
    std::cout << "started QUICK RTH\n";
}

void MissionWrapper::quickRTHMissionStart() {
    quick_rth_running = true;
    run_mtx.lock();
    is_running = true;
    run_mtx.unlock();

    if(waypoints[selected_mission].poses.size()<=0){
        wp_idx_current = -1;
        std::cout << "NO WAYPOINT AT ALL\n";
        return;
    }

    sleep(3);
    bool arrived;
    int stopWP = 1;
    mission_wp_mtx.lock();
    wp_idx_current = 0;
    mission_wp_mtx.unlock();

    ros::Rate r_rate(50);

    while(wp_idx_current < stopWP && is_running && quick_rth_running && ros::ok()){
        arrived = false;

        while(!arrived && is_running && quick_rth_running && ros::ok()){
            avoiding_obstacle = false;

            if(use_obstacle_avoidance and sensor_control.isThereIsObstableSRF()){
                steer_out = steer_trim + controlObstacleAvoidanceSRF();
            }else{
                steer_out = steer_trim + controlWaypoint();
            }

            thrust_out = controlSpeed(-9999,-1, pid_speed_control);

            publishControlToSTM(steer_out, thrust_out);

            r_rate.sleep();
            arrived = isWaypointArrived();
        }

        setNextDest();
        usleep(1000);
    }
    publishControlToSTM(0, 0);
    run_mtx.lock();
    is_running = false;
    selected_mission = -1;
    quick_rth_running = false;
    quick_rth_called = false;
    run_mtx.unlock();
    std::cout << "QUICK RTH DONE\n";
}

int MissionWrapper::controlObstacleAvoidanceSRF() {
    avoiding_obstacle = true;
    sensor_control.setCurrentSteer(controlWaypoint());
    double error_obs = sensor_control.steeringDegreeToAvoidObstacle();
    return calculateErrorPID(pid_obstacle_avoidance_srf, error_obs, error_obs,
                                  error_obs + error_obs);

}

int MissionWrapper::controlFindKorbanCamera() {
    int steer;
    error_camera_before = error_camera;
    error_camera = sensor_control.frontKorbanPositionErrorCamera();
    double distance_korban = sensor_control.korbanDistanceEstimationCamera();
    std::cout << "DISTANCE KORBAN" << distance_korban << "\n";
    if(distance_korban <= 0 && distance_korban != -9999) //braking for 10 sec then RTH
        is_korban_found = true;


    steer = calculateErrorPID(pid_camera_tracking, error_camera, error_camera_before,
                                               error_camera + error_camera_before);

    return steer;
}

int MissionWrapper::controlLifebuoyKorbanCamera() {
    int steer;
    error_camera_before = error_camera;
    int korban_position_x = 320 + sensor_control.frontKorbanPositionErrorCamera();
    error_camera = korban_position_x - boatside->x();
    double distance_korban = sensor_control.korbanDistanceEstimationCamera();
    std::cout << "DISTANCE KORBAN" << distance_korban << "\n";
//    if(distance_korban <= 0 && distance_korban != -9999) //braking for 10 sec then RTH
//        is_korban_found = true;


    steer = calculateErrorPID(pid_camera_tracking, error_camera, error_camera_before,
                              error_camera + error_camera_before);

    return steer;
}

int MissionWrapper::controlWaypoint() {
    double control_wp_distance_out = 0;
    double control_wp_angle_out = 0;
    if(wp_idx_current >= waypoints[selected_mission].poses.size()) return 0;

    error_dist_wp_before = error_dist_wp;
    error_dist_wp = distanceLineToPoint(waypoints[selected_mission].poses[wp_idx_current-1].pose.position, waypoints[selected_mission].poses[wp_idx_current].pose.position, ASV_TF.pos);

    control_wp_distance_out = calculateErrorPID(pid_wp_dist, error_dist_wp, error_dist_wp_before,
                                                error_dist_wp + error_dist_wp_before);

    error_angle_wp_before = error_angle_wp;
    if (wp_idx_current ==0 || (error_dist_wp > DIST_TO_WP_THRESHOLD)) //not using line to distance control, only use heading to WP
    {
        control_wp_distance_out = 0;
//        std::cout << "SINGLE ERROR ANGLE " << error_dist_wp << "\n";
        error_angle_wp = utils::radToDeg(getDestAngle(waypoints[selected_mission].poses[wp_idx_current].pose.position) - ASV_TF.yaw);
    }
    else {
        error_angle_wp = utils::radToDeg(angleOfTwoWaypoints(wp_idx_current-1, wp_idx_current) - ASV_TF.yaw);
    }

    if ( error_angle_wp < -180 ) error_angle_wp += 360; //waypoint behind robot heading
    else if (error_angle_wp > 180) error_angle_wp -= 360;

        // errorDist = distanceLineToPointLocal();

//       std::cout << "ERROR DIST: " << errorDist << " real : " << distanceLineToPoint(initPos, waypoints.poses[wpIdx].pose.position, boatTF.pos) <<  std::endl;

    control_wp_angle_out = calculateErrorPID(pid_wp_angle, error_angle_wp, error_angle_wp_before, error_angle_wp+error_angle_wp_before);
    // if(controlDistanceOut<-250)controlDistanceOut = -250;
    // else if(controlDistanceOut>250)controlDistanceOut = 250;

    if(isWaypointArrived()){
        setNextDest();
    }
    double result = control_wp_angle_out - control_wp_distance_out;
//    return result;
    if ((double)(steer_trim + result) >= MAX_STEERING){ //CLAMPING STEERING ACTUATOR
//        std::cout << "STEERING CLAMPING LEFT\n";
        return MAX_STEERING - steer_trim;
    }else if((double)(steer_trim + result) <= -MAX_STEERING){
//        std::cout << "STEERING CLAMPING RIGHT\n";
        return -(MAX_STEERING + steer_trim);
    }else{
        error_angle_wp_accumulative += error_angle_wp;
        error_dist_wp_accumulative += error_dist_wp;
        return result;
    }
}

double MissionWrapper::controlSpeed(double target_speed, int sensitivity, PID_t pid_speed) {
    if(!use_speed_control)
        return target_constant_thrust;

    if(status_auto_from_remote != 2){
        return 0;
    }

    double curr_target_speed;
    int multiplier_sensitivity;
    if(use_obstacle_stop and
            (save_front_distance >= sensor_control.frontObstacleDistanceSRF()) ){
        curr_target_speed = 0;
        multiplier_sensitivity = 5;
    }else if(avoiding_obstacle){
        curr_target_speed = 1;
        multiplier_sensitivity = 1;
    }else{
        curr_target_speed = waypoints_speed[selected_mission][wp_idx_current];
        multiplier_sensitivity = 1;
    }

    if(target_speed != -9999){
        curr_target_speed = target_speed;
    }

    error_speed_before = error_speed;
    error_speed = (curr_target_speed - asv_local_speed) * multiplier_sensitivity;

    double add_thrust_out = calculateErrorPID(pid_speed, error_speed, error_speed_before,
                                           error_speed  + error_speed_before);

    std::cout << "CUR SPEED " << asv_local_speed << " TARGET "
        << waypoints_speed[selected_mission][wp_idx_current]
        << " ADD " << add_thrust_out << "\n";

    if (thrust_out + add_thrust_out >= MAX_THRUST)
        return MAX_THRUST;
    else if(thrust_out + add_thrust_out <= -MAX_THRUST)
        return -MAX_THRUST;
    else
        return thrust_out + add_thrust_out;
}

void MissionWrapper::checkIsPatrol() { //invert waypoint
    if(patrol_lap > 0){
        std::cout << "INVERTING WP FOR PATROL\n";
        waypoints[PATROL_BACK].poses.clear();
        waypoints_action[PATROL_BACK].clear();
        for (int i = waypoints[MAIN_MISSION].poses.size() - 1; i >= 0 ; i--){
            waypoints[PATROL_BACK].poses.push_back(waypoints[MAIN_MISSION].poses[i]);
            waypoints_action[PATROL_BACK].push_back(waypoints_action[MAIN_MISSION][i]);
            waypoints_speed[PATROL_BACK].push_back(waypoints_speed[MAIN_MISSION][i]);
        }
        patroling_back = false;
        current_patrol_lap = patrol_lap;
    }else{
        current_patrol_lap = -1;
    }

}

void MissionWrapper::setNextDest(){
    wp_idx_current++;

    rviz_plugin::NextDest nextDest_msg;
    nextDest_msg.misi = selected_mission;
    nextDest_msg.wp_index = wp_idx_current;
    nextDest_msg.action_wp = waypoints_action[selected_mission][wp_idx_current];
    nextDest_wp_marker_pub.publish(nextDest_msg);
}

inline bool MissionWrapper::isWaypointArrived(){
    double delta_x = ASV_TF.pos.x - waypoints[selected_mission].poses[wp_idx_current].pose.position.x;
    double delta_y = ASV_TF.pos.y - waypoints[selected_mission].poses[wp_idx_current].pose.position.y;
    double angle = utils::radToDeg(
            getAngle(ASV_TF.pos, waypoints[selected_mission].poses[wp_idx_current].pose.position)
            - getAngle(waypoints[selected_mission].poses[wp_idx_current-1].pose.position,
                       waypoints[selected_mission].poses[wp_idx_current].pose.position)
    );
    if (fabs(angle) >= 180){
        angle = 360 - fabs(angle);
    }
    if(!(fabs(angle) < ANGLE_WP_THRESHOLD and pow(delta_x, 2) + pow(delta_y, 2) > pow(RADIUS_WP_THRESHOLD,2)))
//        std::cout << "Angle of init, boat, dest : " << angle << "  " << pow(delta_x, 2) + pow(delta_y, 2) <<" IDX: " << wp_idx_current << std::endl;
    return !((fabs(angle) < ANGLE_WP_THRESHOLD) and (pow(delta_x, 2) + pow(delta_y, 2) > pow(RADIUS_WP_THRESHOLD,2)))
                and (pow(delta_x, 2) + pow(delta_y, 2) < pow(DIST_TO_WP_THRESHOLD,2));
}

double MissionWrapper::getAngle(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2){
    /*
     * Angle of 2 vector in 2D, ignore z component
     */
    return atan2( -p1.x + p2.x, -p1.y + p2.y );
}

inline double MissionWrapper::distanceLineToPoint(const geometry_msgs::Point& init, const geometry_msgs::Point& dest, const geometry_msgs::Point& curr){
    double y2Miny1 = dest.y-init.y;
    double x2Minx1 = dest.x-init.x;
    return 10*((y2Miny1*curr.x - x2Minx1*curr.y + dest.x*init.y - dest.y*init.x)
               / sqrt(y2Miny1*y2Miny1 + x2Minx1*x2Minx1));
}

double MissionWrapper::getDestAngle(const geometry_msgs::Point& dest){
    /*
    * Since the coordinate of the map in RViz is : up (X-) and right (Y-),
    * and North (0 degree) is X- axis and East (90 degree) is Y-,
    * then, tan of the destination angle is
    * tan(theta) = (- delta_X ) / (- delta_Y )
    * Therefore, theta = arctan ( (- delta_X ) / (-delta_Y ) )
    */
    return atan2( -dest.x + ASV_TF.pos.x, -dest.y + ASV_TF.pos.y );
}

inline double MissionWrapper::angleOfTwoWaypoints(int wp1_idx, int wp2_idx) {
    double x1 = waypoints[selected_mission].poses[wp1_idx].pose.position.x;
    double x2 = waypoints[selected_mission].poses[wp2_idx].pose.position.x;
    double y1 = waypoints[selected_mission].poses[wp1_idx].pose.position.y;
    double y2 = waypoints[selected_mission].poses[wp2_idx].pose.position.y;
    double result = atan2((x2-x1), (y2-y1));

    if (result > 0){
        return (-1 * M_PI) + result;
    }else {
        return M_PI + result;
    }
}

double MissionWrapper::calculateErrorPID(PID_t pid, double error_now, double error_before, double error_acc) {
    return (double)(pid.p * error_now + pid.i * (error_acc) + pid.d * (error_now - error_before));
}

void MissionWrapper::markerCallback(const visualization_msgs::MarkerArray &msg) {
    std::cout << "JUMLAH MARKER "<< msg.markers.size() << "\n";
    std::cout << msg.markers[0].pose.position.x << "\n";
}

void MissionWrapper::waypointsCallback(const rviz_plugin::Selectedwp& msg) {
    short idx_miss = msg.missionSelected-1;

    mission_wp_mtx.lock();
    waypoints[idx_miss] = msg.wp;
    waypoints_speed[idx_miss].clear();
    waypoints_action[idx_miss].clear();

    for(int it=0; it<waypoints[idx_miss].poses.size(); ++it) {
        waypoints_speed[idx_miss].push_back(msg.speed[it]);
        waypoints_action[idx_miss].push_back(msg.actionwp[it]);

        std::cout << waypoints_speed[idx_miss][it] << " topic " << msg.speed[it] << "\n";
    }
    checkIsPatrol();

    mission_wp_mtx.unlock();
    std::stringstream ss;
    ss << "DAPAT WP CALLBACK\n" << idx_miss << " IDX\n" <<
        waypoints[idx_miss].poses.size() << " wp total\n";

    emit setLogInformationDisplay(QString(ss.str().c_str()));
//    std::cout << "DAPAT WP CALLABCK\n";
//    std::cout << waypoints[idx_miss];
}

void MissionWrapper::publishControlToSTM(int steer, int thrust) {
    geometry_msgs::Twist stm_msg;
    stm_msg.linear.y = thrust;
    stm_msg.angular.z = steer;
    stm_pub.publish(stm_msg);
}

void MissionWrapper::runManual(const rviz_plugin::ManualCtrl& msg){
    geometry_msgs::Twist stm_msg;

//    if (msg.speed >= 0){
//        stm_msg.linear.z = 0;
//
//    }
    stm_msg.linear.y = msg.speed;
    stm_msg.angular.z = -msg.servo;

    stm_pub.publish(stm_msg);
}

void MissionWrapper::loadParams(ros::NodeHandle nh){
    if (nh.getParam("/main_node/frame_id/world", world_frame))
    {
        ROS_INFO("Got param: %s", world_frame.c_str());
    }
    if (nh.getParam("/main_node/frame_id/robot", robot_frame))
    {
        ROS_INFO("Got param: %s", robot_frame.c_str());
    }
    std::cout <<"frame: " <<  world_frame << " " << robot_frame << std::endl;
}

void MissionWrapper::listenBoatTF(){
    ros::Rate rate(50);
    double pitch, roll, yaw;
    geometry_msgs::Point tf_point;

    int last;
    bool found;
    while (ros::ok()){
        try{
            tf_mutex.lock();
            boatTF_sub.lookupTransform(world_frame, robot_frame,
                                       ros::Time(0), currentBoatTF);
            tf::Matrix3x3 m(currentBoatTF.getRotation());
            m.getRPY(pitch, roll, yaw);
            // std::cout << "HEADING " << utils::radToDeg(heading) << std::endl;
            tf_mutex.unlock();

        }
        catch (tf::TransformException &ex) {
            tf_mutex.unlock();
            continue;
        }

        ASV_TF.pos.x = currentBoatTF.getOrigin().x();
        ASV_TF.pos.y = currentBoatTF.getOrigin().y();
        ASV_TF.pos.z = currentBoatTF.getOrigin().z();

        ASV_TF.yaw = -yaw;
        ASV_TF.pitch = pitch;
        ASV_TF.roll = roll;

        sensor_control.setCurrentASVPitch(ASV_TF.pitch);

        geometry_msgs::Twist msg_tf_orientation;
        msg_tf_orientation.angular.x = ASV_TF.roll;
        msg_tf_orientation.angular.y =  ASV_TF.pitch;
        msg_tf_orientation.angular.z = ASV_TF.yaw;
        orientation_TF_pub.publish(msg_tf_orientation);
//        if(currentMission){
//            tf_mutex.lock();
//            misi[currentMission]->setBoatTF(currentBoatTF);
//            misi[currentMission]->updateCurrentSpeed(currentSpeed);
//            tf_mutex.unlock();
//        }
        rate.sleep();
    }
    return;
}

void MissionWrapper::startMission() {

}

void MissionWrapper::stopMission() {

}

void MissionWrapper::resumeMission() {

}

void MissionWrapper::changeSteerTrim(int val) {
    param_qt_mtx.lock();
    steer_trim = val;
    param_qt_mtx.unlock();
}

void MissionWrapper::changeSpeedTrim(int val) {
    param_qt_mtx.lock();
    thrust_trim = val;
    param_qt_mtx.unlock();
}

void MissionWrapper::useSpeedControl(bool val) {
    param_qt_mtx.lock();
    use_speed_control = val;
    param_qt_mtx.unlock();
}

void MissionWrapper::useAccError(bool) {

}

void MissionWrapper::changeUseObstacleAvoidance(bool val) {
    param_qt_mtx.lock();
    use_obstacle_avoidance = val;
    param_qt_mtx.unlock();
    sensor_control.setCurrentSaveDistance(save_front_distance);
}

void MissionWrapper::changeUseObstacleStop(bool val) {
    param_qt_mtx.lock();
    use_obstacle_stop = val;
    param_qt_mtx.unlock();
}

void MissionWrapper::changeP(int idx, double val) {
    param_qt_mtx.lock();
    if(idx == 0){
        pid_wp_angle.p = val;
    }else if(idx == 1){
        pid_wp_dist.p = val;
    }else if(idx == 2){
        pid_speed_control.p = val;
    }else if(idx == 3){
        pid_obstacle_avoidance_srf.p = val;
    }else if(idx == 4){
        pid_camera_tracking.p = val;
    }else if(idx == 5){
        pid_camera_tracking_thrust.p = val;
    }
    param_qt_mtx.unlock();
}

void MissionWrapper::changeI(int idx, double val) {
    param_qt_mtx.lock();
    if(idx == 0){
        pid_wp_angle.i = val;
    }else if(idx == 1){
        pid_wp_dist.i = val;
    }else if(idx == 2){
        pid_speed_control.i = val;
    }else if(idx == 3){
        pid_obstacle_avoidance_srf.i = val;
    }else if(idx == 4){
        pid_camera_tracking.i = val;
    }else if(idx == 5){
        pid_camera_tracking_thrust.i = val;
    }
    param_qt_mtx.unlock();
}

void MissionWrapper::changeD(int idx, double val) {
    param_qt_mtx.lock();
    if(idx == 0){
        pid_wp_angle.d = val;
    }else if(idx == 1){
        pid_wp_dist.d = val;
    }else if(idx == 2){
        pid_speed_control.d = val;
    }else if(idx == 3){
        pid_obstacle_avoidance_srf.d = val;
    }else if(idx == 4){
        pid_camera_tracking.d = val;
    }else if(idx == 5){
        pid_camera_tracking_thrust.d = val;
    }
    param_qt_mtx.unlock();
}

void MissionWrapper::changeSpeed(int idx, int val) {
    target_constant_thrust = val;
}

void MissionWrapper::changeDistance(int idx, double val) {
    param_qt_mtx.lock();
    save_front_distance = val;
    param_qt_mtx.unlock();
}

void MissionWrapper::changeASVWidth(double) {

}

void MissionWrapper::changeASVOffset(double) {

}

void MissionWrapper::changeRTHAfterDone(bool val) {
    std::cout << "RTH " << val << "\n";
    is_rth_after_done = val;
    rth_called = false;
}

void MissionWrapper::changePatrolLap(int val) {
    param_qt_mtx.lock();
    patrol_lap = val;
    param_qt_mtx.unlock();
    std::cout << "PATROL LAP TO " << val << "\n";
}

void MissionWrapper::calibrateCompass(double val){
    std_msgs::Float32 msg;
    msg.data = val;
    cmpcal_pub.publish(msg);
}

void MissionWrapper::localVelocityCallback(const geometry_msgs::TwistStamped &msg) {
    double y_vec = double(msg.twist.linear.y);
    double x_vec = double(msg.twist.linear.x);

    bool is_move_forward = x_vec >= 0 ? true : false;
    double speed_direction = 90 - utils::radToDeg(atan2(y_vec,x_vec)); //rotate 90derajat
    if(speed_direction > 180)
        speed_direction -= 360;
    if(speed_direction < -180)
        speed_direction += 360;

    sensor_mtx.lock();
    asv_local_speed = sqrt(pow(y_vec,2) + pow(x_vec,2));
    if(!is_move_forward)
        asv_local_speed *= -1;
    sensor_mtx.unlock();

//    std::cout << "ASV LOCAL SPEED " << asv_local_speed << "\n";
}

void MissionWrapper::dataFromStmCallback(const comm_stm::stm_status &msg){
    if(msg.trim != -1){
        steer_trim_from_remote = msg.trim;
        emit steer_trim_changed_from_remote(steer_trim_from_remote - 1500);
    }

    status_auto_from_remote = msg.state_remote;
    //send to ui trim
}

void MissionWrapper::changeUseCompass(bool val) {
    std_msgs::UInt8 msg;
    msg.data = (int)val;
    using_compass_pub.publish(msg);
}

void MissionWrapper::publishMarkers(){
    int radius = 2.0;
    geometry_msgs::Point p0, p1, p2, p;
    TF_simplified boatTF;
    ros::Rate rate(10);
    visualization_msgs::Marker headingLine, targetLine, front_obs;
    targetLine.type = visualization_msgs::Marker::LINE_STRIP;
    front_obs.type = visualization_msgs::Marker::CUBE;
    front_obs.header.frame_id = "asv/base_link";
    headingLine.header.frame_id = targetLine.header.frame_id = "map";
    headingLine.header.stamp = targetLine.header.stamp = front_obs.header.stamp = ros::Time::now();
    headingLine.ns = targetLine.ns = front_obs.ns =  "lines";
    headingLine.action = targetLine.action = front_obs.action = visualization_msgs::Marker::ADD;
    headingLine.scale.x = targetLine.scale.x = 0.05;
    front_obs.scale.x = 2;
    front_obs.scale.y = 7;
    front_obs.scale.z = 0.05;
    headingLine.color.r = 1.0;
    targetLine.color.g = 1.0;
    headingLine.color.a = targetLine.color.a = 1.0;
    front_obs.pose.position.y = -5;
    front_obs.color.r = 1;
    front_obs.color.a = 0.3;

    targetLine.pose.orientation.x = 0;
    targetLine.pose.orientation.y = 0;
    targetLine.pose.orientation.z = 0;
    targetLine.pose.orientation.w = 1;
    headingLine.pose.orientation.x = 0;
    headingLine.pose.orientation.y = 0;
    headingLine.pose.orientation.z = 0;
    headingLine.pose.orientation.w = 1;

    visualization_msgs::MarkerArray road;
    double angle;
    geometry_msgs::Point right, left, center;
    visualization_msgs::Marker boundary;
    std_msgs::ColorRGBA c;
    boundary.type = visualization_msgs::Marker::LINE_LIST;
    boundary.header.frame_id = "/rslidar";
    boundary.action = visualization_msgs::Marker::ADD;
    boundary.color.r = 0;
    boundary.color.g = 0;
    boundary.color.b = 1;
    boundary.color.a = 1;
    c.a = .1;
    // 27,111,40
    c.r = 0.1;
    c.g = .5;
    c.b = .15;
    boundary.colors.push_back(c);
    boundary.colors.push_back(c);
    c.r = .5;
    c.g = .5;
    c.b = .15;
    boundary.colors.push_back(c);
    boundary.colors.push_back(c);
    c.r = .5;
    c.g = .5;
    c.b = .5;
    boundary.colors.push_back(c);
    boundary.colors.push_back(c);
    c.r = 1;
    c.g = 0;
    c.b = 0;
    boundary.colors.push_back(c);
    boundary.colors.push_back(c);

    boundary.pose.position.y = 0;
    boundary.pose.position.x = 0;
    boundary.pose.position.z = 0;
    boundary.pose.orientation.w = 0;
    boundary.pose.orientation.x = 0;
    boundary.pose.orientation.y = 0;
    boundary.pose.orientation.z = 0;
    boundary.scale.x = .5;

    // compass helper
    visualization_msgs::Marker compass_helper;
    compass_helper.type = visualization_msgs::Marker::LINE_LIST;
    compass_helper.header.frame_id = "map";
    compass_helper.action = visualization_msgs::Marker::ADD;
    compass_helper.color.r = 0;
    compass_helper.color.g = 0;
    compass_helper.color.b = 1;
    compass_helper.color.a = 1;
    c.a = 1;
    // 27,111,40
    c.r = 1;
    c.g = 0;
    c.b = 0;
    compass_helper.colors.push_back(c);
    compass_helper.colors.push_back(c);
    c.r = 0;
    c.g = 0;
    c.b = 1;
    compass_helper.colors.push_back(c);
    compass_helper.colors.push_back(c);

    compass_helper.pose.position.y = 0;
    compass_helper.pose.position.x = 0;
    compass_helper.pose.position.z = 2;
    compass_helper.pose.orientation.w = 0;
    compass_helper.pose.orientation.x = 0;
    compass_helper.pose.orientation.y = 0;
    compass_helper.pose.orientation.z = 0;
    compass_helper.scale.x = .2;

    std_msgs::Float32 angle_msg;
    angle_path_pub = nh.advertise<std_msgs::Float32>("/autonomous_car/path_angle", 1);

    while(ros::ok()){
        headingLine.header.stamp = targetLine.header.stamp = front_obs.header.stamp = compass_helper.header.stamp = ros::Time::now();

//        front_obs_pub.publish(front_obs);

        // compass_helper
        {
            compass_helper.points.clear();
            p0.x = currentBoatTF.getOrigin().x();
            p0.y = currentBoatTF.getOrigin().y();
            p0.z = 0;

            p1.x = p0.x;
            p1.y = p0.y - 5;
            p1.z = 0;

            compass_helper.points.push_back(p0);
            compass_helper.points.push_back(p1);

            p1.x = p0.x - 5;
            p1.y = p0.y;
            p1.z = 0;

            compass_helper.points.push_back(p0);
            compass_helper.points.push_back(p1);

            compass_helper_pub.publish(compass_helper);
        }


        if(selected_mission==-1) {
            angle_path_pub.shutdown();
            headingLine.points.clear();
            targetLine.points.clear();
            headingMarker_pub.publish(headingLine);
            targetMarker_pub.publish(targetLine);
            usleep(20000);
            continue;
        }


        {   // heading line
            headingLine.points.clear();
            p0.x = currentBoatTF.getOrigin().x();
            p0.y = currentBoatTF.getOrigin().y();
            p0.z = .05;
            boatTF = ASV_TF;
            p1.x = p0.x - radius * sin(boatTF.yaw);
            p1.y = p0.y - radius * cos(boatTF.yaw);
            p1.z = .05;
            headingLine.points.push_back(p0);
            headingLine.points.push_back(p1);
            headingMarker_pub.publish(headingLine);
        }

        { // target line
            targetLine.points.clear();
            if(wp_idx_current < 0) continue;

            p1 = waypoints[selected_mission].poses[wp_idx_current].pose.position;
            p2 = waypoints[selected_mission].poses[wp_idx_current-1].pose.position;
            targetLine.points.push_back(p0);
            targetLine.points.push_back(p1 );
            targetLine.points.push_back(p2);
            targetMarker_pub.publish(targetLine);
        }

//        if(angle_path_pub.getNumSubscribers()==0){
//            angle_path_pub.shutdown();
//            angle_path_pub = nh.advertise<std_msgs::Float32>("/autonomous_car/path_angle", 1);
//            std::cout << "angle sub 000\n";
//        }
//        else{
//            angle_msg.data = utils::radToDeg(getAnglePath());
//            angle_path_pub.publish(angle_msg);
//            std::cout << "angle sub publish " << angle_msg.data << std::endl;
//        }

        rate.sleep();
    }
}

double MissionWrapper::getAnglePath() {
    std::cout << "ANGLE PATH : " << getAngle(waypoints[selected_mission].poses[wp_idx_current].pose.position,
                                             waypoints[selected_mission].poses[wp_idx_current-1].pose.position)
        << " " << ASV_TF.yaw << std::endl;
    return getAngle(waypoints[selected_mission].poses[wp_idx_current].pose.position,
                    waypoints[selected_mission].poses[wp_idx_current-1].pose.position) - ASV_TF.yaw;
}

void MissionWrapper::startRecordPath(){
    // std::cout << "Start record path" << std::endl;
    if (is_path_recorded){
        // std::cout << "is recording... " << std::endl;
        return;
    }

    path_mtx.lock();
    is_path_recorded = true;
    path_mtx.unlock();
    path_thread = std::thread(&MissionWrapper::recordPath, this);
}

void MissionWrapper::stopRecordPath(){
    // std::cout << "Stop record path" << std::endl;
    if (!is_path_recorded){
        return;
    }

    path_mtx.lock();
    is_path_recorded = false;
    path_mtx.unlock();
    path_thread.join();
    std::cout << "Stopped record path" << std::endl;
}

void MissionWrapper::recordPath(){
    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped global_pose;
    pose.header.frame_id = "map";
    path_msg.header.frame_id = "map";
    is_path_recorded = true;
    // printf("masuk path record\n");

    while(is_path_recorded){
        // printf("masuk path record\n");
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = currentBoatTF.getOrigin().getX();
        pose.pose.position.y = currentBoatTF.getOrigin().getY();
        pose.pose.position.z = currentBoatTF.getOrigin().getZ();
        pose.pose.orientation.x = currentBoatTF.getRotation().getX();
        pose.pose.orientation.y = currentBoatTF.getRotation().getY();
        pose.pose.orientation.z = currentBoatTF.getRotation().getZ();
        pose.pose.orientation.w = currentBoatTF.getRotation().getW();

        path_msg.header.stamp = ros::Time::now();
        path_msg.poses.push_back(pose);
        path_pub.publish(path_msg);

        // Record GPS
        if(current_gps.latitude != 0.0000 && current_gps.longitude != 0.0000){
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

void MissionWrapper::clearPath(){
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

void MissionWrapper::savePath(QString filename){
    std::cout << "save path to " << filename.toStdString() << std::endl;
    if (global_path.size()==0){
        return;
    }
    std::cout << "save path" << std::endl;
    sensor_msgs::NavSatFix path_to_save;
    QFile file(filename);

    if (file.open(QIODevice::ReadWrite | QIODevice::Truncate)) {
        QTextStream stream(&file);
        char str[1024];
        auto pit=path_msg.poses.begin();
        for (auto it=++(global_path.begin()); it != global_path.end(); ++it){
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

    if (file_loc.open(QIODevice::ReadWrite | QIODevice::Truncate)) {
        QTextStream stream(&file_loc);
        char str[1024];
        for (auto pit=++(path_msg.poses.begin()); pit != path_msg.poses.end(); ++pit){
            memset(str, 0, sizeof(str));
            sprintf(str, "%.8f,%.8f",
                    pit->pose.position.x, pit->pose.position.y);
            stream << str << endl;
            std::cout << str << std::endl;
        }
        file_loc.close();
    }
}

void MissionWrapper::loadPath(QString filename){
    std::cout << "load path" << std::endl;
    geometry_msgs::PoseStamped pose;
    double lat, lon;

    loaded_path_msg.poses.clear();
    loaded_path_msg.header.frame_id = "map";

    QFile file(filename);

    if(file.open(QIODevice::ReadWrite)) {

        QTextStream in(&file);

        while(!in.atEnd()) {
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


void MissionWrapper::loadLocalPath(QString filename){
    std::cout << "load path" << std::endl;
    geometry_msgs::PoseStamped pose;
    double lat, lon;

    loaded_path_msg.poses.clear();
    loaded_path_msg.header.frame_id = "map";

    QFile file(filename);

    if(file.open(QIODevice::ReadWrite)) {

        QTextStream in(&file);

        while(!in.atEnd()) {
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

void MissionWrapper::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    gps_mtx.lock();
    current_gps = *msg;
    gps_mtx.unlock();
}

inline geometry_msgs::PoseStamped MissionWrapper::globalToPose(double lat, double lon){
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";

    pose.pose.position.x = LON_TO_METER * (LongitudeCenter - lon);
    pose.pose.position.y = LAT_TO_METER * (LatitudeCenter - lat);
    pose.pose.position.z = .5;

    return pose;
}

void MissionWrapper::initVenue(const char* venue){
    std::string filename = ros::package::getPath("map_image") + "/resource/" + venue + ".txt";
    FILE *file = fopen(filename.c_str(), "r");
    fscanf(file, "%lf, %lf %lf, %lf", &LatitudeTop, &LongitudeLeft, &LatitudeBot, &LongitudeRight);
    fclose(file);

    LatitudeCenter = .5 * (LatitudeTop + LatitudeBot);
    LongitudeCenter = .5 * (LongitudeLeft + LongitudeRight);
    std::cout << venue << LatitudeTop << " " <<  LongitudeLeft << " " <<  LatitudeBot << " " <<  LongitudeRight << " " << LatitudeCenter << " " << LongitudeCenter << std::endl;
//    sleep(10);
}

void MissionWrapper::changeUseFindKorban(bool val) {
    param_qt_mtx.lock();
    use_find_korban = val;
    param_qt_mtx.unlock();
    std::cout << "FIND KORBAN " << use_find_korban << "\n";
}

void MissionWrapper::changeCritLine(int i, int cl) {
    param_qt_mtx.lock();
    if (i) crit_line = cl;
    else horizon = cl;
    param_qt_mtx.unlock();

    sensor_control.setCurrentCritLineCamera(cl);
    std::cout << "CRIT LINE " << crit_line << " " << horizon << "\n";
}

void MissionWrapper::changeBoatSide(QPoint **) {

}
void MissionWrapper::pitchChanged(int idx, double val) {
    sensor_control.setPitchThreshold(val);
}

void MissionWrapper::critlineCallback(const std_msgs::Int32MultiArray msg) {
    changeCritLine(1, msg.data[1]); //critline
    changeCritLine(0, msg.data[2]); //horizon

    emit setCritLine(1, crit_line);
    emit setCritLine(0, horizon);
}

void MissionWrapper::boatsideCallback(const geometry_msgs::PoseArray msg) {
    param_qt_mtx.lock();
    for(int i = 0; i < 4; i++){
        boatside[i].setX(msg.poses[i].position.x);
        boatside[i].setY(msg.poses[i].position.y);
    }
    param_qt_mtx.unlock();
}